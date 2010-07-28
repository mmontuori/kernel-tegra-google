/*
 * drivers/input/touchscreen/panjit_i2c_touch.c
 *
 * Touchscreen class input driver for Panjit touch panel using I2C bus
 *
 * Copyright (c) 2010, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/slab.h>

/* touch controller registers */
#define CSR				0x00 /* Control and Status register */
#define C_FLAG				0x01 /* Interrupt Clear flag */
#define X1_H				0x03 /* High Byte of X1 Position */
#define X1_L				0x04 /* Low Byte of X1 Position */
#define Y1_H				0x05 /* High Byte of Y1 Position */
#define Y1_L				0x06 /* Low Byte of Y1 Position */
#define X2_H				0x07 /* High Byte of X2 Position */
#define X2_L				0x08 /* Low Byte of X2 Position */
#define Y2_H				0x09 /* High Byte of Y2 Position */
#define Y2_L				0x0A /* Low Byte of Y2 Position */
#define FINGERS				0x0B /* Detected finger number */
#define GESTURE				0x0C /* Interpreted gesture */

/* Control Status Register bit masks */
#define CSR_SLEEP_EN			(1 << 7)
#define CSR_SCAN_EN			(1 << 3)
#define SLEEP_ENABLE			1

/* Interrupt Clear register bit masks */
#define C_FLAG_CLEAR			0x08
#define INT_ASSERTED			(1 << C_FLAG_CLEAR)

/* Gesture register bit masks */
#define ZOOM_IN				0x0C
#define ZOOM_OUT			0x0D
#define DOUBLE_CLICK			0x0E

/* #defines */
#define TOOL_PRESSURE			100
#define TOOL_WIDTH			8
#define INVALID_TOUCH_INPUT		0xFFFF

#define MAX_FINGERS			2
#define DRIVER_NAME	 		"panjit_touch"

#define swapv(x, y) do { typeof(x) z = x; x = y; y = z; } while (0)

static struct workqueue_struct *touch_wq;

/* Defines the gesture type. */
enum pj_gesture {
	GESTURE_INVALID		= 0x0000,
	GESTURE_NONE		= 0x0001,
	GESTURE_TAP		= 0x0002,
	GESTURE_DOUBLE_TAP	= 0x0004,
	GESTURE_ZOOM_IN		= 0x0008,
	GESTURE_ZOOM_OUT	= 0x0010,
};

enum pj_orientation {
	ORIENTATION_NONE	= 0x0,
	ORIENTATION_XYSWAP	= 0x01,
	ORIENTATION_HFLIP	= 0x02,
	ORIENTATION_VFLIP	= 0x04
};

/* Defines the touch capabilities. */
struct pj_caps {
	int	nr_fingers;
	int	maxwidth;
	int	xmin;
	int	ymin;
	int	xmax;
	int	ymax;
	enum pj_gesture gesture;
	enum pj_orientation orientation;
};

/* data corresponding to a touch event */
struct pj_finger {
	unsigned short x;
	unsigned short y;
	unsigned short delta;
};

struct pj_event {
	int fingers;
	enum pj_gesture gesture;
	struct pj_finger data[MAX_FINGERS];
};

struct pj_data {
	struct input_dev	*input_dev;
	struct i2c_client	*client;
	struct hrtimer		timer;
	struct work_struct	work;
	struct pj_caps		caps;
	int			irq;
	int			chipid;
	int			maxx;
	int			minx;
	int			maxy;
	int			miny;
};

static int pj_write_i2c(struct pj_data *touch, uint8_t reg, uint8_t value)
{
	struct i2c_msg msg;
	struct i2c_client *client = touch->client;
	uint8_t data[2];
	int ret = 0;

	if (!touch)
		return -EINVAL;

	data[0] = reg;
	data[1] = value;
	msg.addr = touch->client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = data;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1) {
		dev_err(&client->dev, "%s: i2c_transfer failed (%d)\n",
			__func__, ret);
		ret = (ret<0) ?: -ETIMEDOUT;
	}
	return (ret<0) ?: 0;
}

static int pj_read_i2c(struct pj_data *touch, int len,
		       uint8_t reg, uint8_t *buf)
{
	struct i2c_msg msg[2];
	struct i2c_client *client = touch->client;
	int ret = 0;
	uint8_t r = reg;

	if (!touch || !buf)
		return -EINVAL;

	msg[0].addr = touch->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &r;

	msg[1].addr = touch->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = buf;
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret != ARRAY_SIZE(msg)) {
		dev_err(&client->dev, "%s: i2c_transfer failed (%d)\n",
			__func__, ret);
		ret = (ret<0) ?: -ETIMEDOUT;
	}
	return (ret<0) ?: 0;
}

static enum hrtimer_restart pj_poll(struct hrtimer *timer)
{
	struct pj_data *touch = container_of(timer, struct pj_data, timer);

	queue_work(touch_wq, &touch->work);
	hrtimer_start(&touch->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static irqreturn_t pj_irq(int irq, void *dev_id)
{
	struct pj_data *touch = dev_id;

	disable_irq_nosync(touch->irq);
	queue_work(touch_wq, &touch->work);
	return IRQ_HANDLED;
}

static int pj_read_event(struct pj_data *touch, struct pj_event *event)
{
	struct device *dev = &touch->client->dev;
	uint8_t raw_data[12] = { 0 };
	uint8_t *raw_finger = &raw_data[2];
	uint8_t *nr_fingers = &raw_data[10];
	int ret = 0;
	int i;

	/* read the entire register space */
	ret = pj_read_i2c(touch, 12, 0, raw_data);
	if (ret) {
		dev_err(dev, "%s: i2c read failed\n", __func__);
		return -EIO;
	}

	dev_dbg(dev, "%s: %d fingers\n", __func__, *nr_fingers);

	if (*nr_fingers > touch->caps.nr_fingers)
		return -EINVAL;

	/* get the number of fingers on the panel */
	event->fingers = *nr_fingers;

	/* decipher the gesture */
	switch (raw_data[11]) {
	case ZOOM_IN: event->gesture = GESTURE_ZOOM_IN; break;
	case ZOOM_OUT: event->gesture = GESTURE_ZOOM_OUT; break;
	case DOUBLE_CLICK: event->gesture = GESTURE_DOUBLE_TAP; break;
	default: event->gesture = GESTURE_NONE; break;
	}

	raw_finger = &raw_data[2];
	for (i=0; i<2 && i<*nr_fingers; i++, raw_finger+=4) {
		event->data[i].x = (raw_finger[0] << 8) | raw_finger[1];
		event->data[i].y = (raw_finger[2] << 8) | raw_finger[3];
		dev_dbg(dev, "%s: finger[%d]: (%u, %u)\n", __func__,
			i, event->data[i].x, event->data[i].y);
	}

	/* clear interrupt */
	ret = pj_write_i2c(touch, (uint8_t)C_FLAG, 0);
	if (ret) {
		dev_err(dev, "%s: enable interrupt failed\n", __func__);
	}

	return ret;
}

static void pj_work(struct work_struct *work)
{
	struct pj_data *touch = container_of(work, struct pj_data, work);
	struct pj_event event;
	int i;

	if (pj_read_event(touch, &event) != 0)
		goto out;

	/* Report number of fingers */
	input_report_key(touch->input_dev, BTN_TOUCH, event.fingers);

	for (i=1; i<touch->caps.nr_fingers; i++)
		input_report_key(touch->input_dev, BTN_2 + (i-1), event.fingers>i);

	for (i=0; i<event.fingers; i++) {
		int x = event.data[i].x, y = event.data[i].y;

		if (touch->caps.orientation & ORIENTATION_XYSWAP)
			swapv(x,y);

		if (touch->caps.orientation & ORIENTATION_VFLIP)
			y = touch->maxy + touch->miny - y;

		if (touch->caps.orientation & ORIENTATION_HFLIP)
			x = touch->maxx + touch->minx - x;

		if (event.fingers==1) {
			input_report_abs(touch->input_dev, ABS_X, x);
			input_report_abs(touch->input_dev, ABS_Y, y);
		} else {
			input_report_abs(touch->input_dev, ABS_HAT0X+(i*2), x);
			input_report_abs(touch->input_dev, ABS_HAT0Y+(i*2), y);
		}
	}

	input_sync(touch->input_dev);

out:
	if (touch->irq >= 0)
		enable_irq(touch->irq);
}

static int pj_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct pj_data *touch = NULL;
	struct input_dev *input_dev = NULL;
	int i = 0, ret = 0;

	dev_info(&client->dev, "%s: probe %p\n", __func__, client);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: need I2C_FUNC_I2C\n", __func__);
		return -ENODEV;
	}

	touch = kzalloc(sizeof(struct pj_data), GFP_KERNEL);
	if (!touch) {
		dev_err(&client->dev, "%s: no memory\n", __func__);
		return -ENOMEM;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "%s: no memory\n", __func__);
		kfree(touch);
		return -ENOMEM;
	}

	touch->client = client;
	i2c_set_clientdata(client, touch);

	/* set capabilities */
	touch->caps.nr_fingers = 2;
	touch->caps.gesture = GESTURE_NONE;
	touch->caps.xmin = 0;
	touch->caps.ymin = 0;
	touch->caps.xmax = 4095;
	touch->caps.ymax = 4095;
	touch->caps.orientation = ORIENTATION_NONE;

	/* clear interrupt */
	ret = pj_write_i2c(touch, (uint8_t)C_FLAG, 0);
	if (ret < 0) {
		dev_err(&client->dev, "%s: clear interrupt failed\n",
			__func__);
		goto fail_alloc;
	}

	/* enable scanning */
	ret = pj_write_i2c(touch, (uint8_t)CSR, CSR_SCAN_EN);
	if (ret < 0) {
		dev_err(&client->dev, "%s: enable interrupt failed\n",
			__func__);
		goto fail_alloc;
	}

	INIT_WORK(&touch->work, pj_work);

	/* get the irq */
	touch->irq = touch->client->irq;
	if (touch->irq >= 0) {
		ret = request_irq(touch->irq, pj_irq, IRQF_DISABLED,
				  DRIVER_NAME, touch);
		if (ret) {
			dev_err(&client->dev, "%s: request_irq(%d) failed\n",
				__func__, touch->irq);
			touch->irq = -1;
		}
	}
	if (touch->irq < 0) {
		/* start timer */
		hrtimer_init(&touch->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		touch->timer.function = pj_poll;
		hrtimer_start(&touch->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

	touch->input_dev = input_dev;
	touch->input_dev->name = DRIVER_NAME;

	set_bit(EV_SYN, touch->input_dev->evbit);
	set_bit(EV_KEY, touch->input_dev->evbit);
	set_bit(EV_ABS, touch->input_dev->evbit);
	set_bit(BTN_TOUCH, touch->input_dev->keybit);
	for (i = 1; i < touch->caps.nr_fingers; i++) {
		set_bit(BTN_2 + (i-1), touch->input_dev->keybit);
	}

	/* expose multi-touch capabilities */
	set_bit(ABS_MT_TOUCH_MAJOR, touch->input_dev->keybit);
	set_bit(ABS_MT_POSITION_X, touch->input_dev->keybit);
	set_bit(ABS_MT_POSITION_Y, touch->input_dev->keybit);
	set_bit(ABS_X, touch->input_dev->keybit);
	set_bit(ABS_Y, touch->input_dev->keybit);

	if (touch->caps.orientation & ORIENTATION_XYSWAP) {
		touch->maxx = touch->caps.ymax;
		touch->minx = touch->caps.ymin;
		touch->maxy = touch->caps.xmax;
		touch->miny = touch->caps.xmin;
	} else {
		touch->maxx = touch->caps.xmax;
		touch->minx = touch->caps.xmin;
		touch->maxy = touch->caps.ymin;
		touch->miny = touch->caps.ymin;
	}

	input_set_abs_params(touch->input_dev, ABS_X, touch->minx,
		touch->maxx, 0, 0);
	input_set_abs_params(touch->input_dev, ABS_Y, touch->miny,
		touch->maxy, 0, 0);

	for (i = 0; i < touch->caps.nr_fingers; i++) {
		input_set_abs_params(touch->input_dev, ABS_HAT0X + (i*2),
				     touch->minx, touch->maxx, 0, 0);
		input_set_abs_params(touch->input_dev, ABS_HAT0Y + (i*2),
				     touch->miny, touch->maxy, 0, 0);
	}

	input_set_abs_params(touch->input_dev, ABS_MT_POSITION_X,
		touch->minx, touch->maxx, 0, 0);
	input_set_abs_params(touch->input_dev, ABS_MT_POSITION_Y,
		touch->miny, touch->maxy, 0, 0);

	input_set_abs_params(touch->input_dev, ABS_MT_TOUCH_MAJOR,
			     0, TOOL_PRESSURE, 0, 0);
	input_set_abs_params(touch->input_dev, ABS_PRESSURE, 0,
			     TOOL_PRESSURE, 0, 0);

	input_set_abs_params(touch->input_dev, ABS_TOOL_WIDTH, 0,
			     TOOL_WIDTH, 0, 0);
	input_set_abs_params(touch->input_dev, ABS_MT_WIDTH_MAJOR, 0,
			     TOOL_WIDTH, 0, 0);

	ret = input_register_device(touch->input_dev);
	if (ret) {
		dev_err(&client->dev, "%s: input_register_device failed\n",
			__func__);
		goto fail_irq;
	}

	dev_info(&client->dev, "%s: initialized\n", __func__);
	return 0;

fail_irq:
	if (touch->irq >= 0)
		free_irq(touch->irq, touch);
	else
		hrtimer_cancel(&touch->timer);

fail_alloc:
	input_free_device(input_dev);
	kfree(touch);
	return ret;
}

static int pj_suspend(struct i2c_client *client, pm_message_t state)
{
	struct pj_data *touch = i2c_get_clientdata(client);
	int ret = 0;

	if (!touch) {
		WARN_ON(1);
		return -EINVAL;
	}

	if (client->irq >= 0)
		disable_irq(client->irq);
	else
		hrtimer_cancel(&touch->timer);

	ret = cancel_work_sync(&touch->work);
	if (ret < 0) {
		dev_err(&client->dev, "%s: cancel_work_sync failed\n",
			__func__);
		return ret;
	}

	/* disable scanning and enable deep sleep */
	return pj_write_i2c(touch, (uint8_t)CSR, CSR_SLEEP_EN);
}

static int pj_resume(struct i2c_client *client)
{
	struct pj_data *touch = i2c_get_clientdata(client);
	int ret = 0;

	if (!touch) {
		WARN_ON(1);
		return -EINVAL;
	}
	/* enable scanning and disable deep sleep */
	ret = pj_write_i2c(touch, (uint8_t)CSR, CSR_SCAN_EN);
	if (ret < 0) {
		dev_err(&client->dev, "%s: interrupt enable fail\n", __func__);
		return ret;
	}

	if (client->irq >= 0)
		enable_irq(client->irq);
	else
		ret = hrtimer_start(&touch->timer, ktime_set(1, 0),
				    HRTIMER_MODE_REL);

	return ret;
}

static int pj_remove(struct i2c_client *client)
{
	struct pj_data *touch = i2c_get_clientdata(client);

	if (!touch)
		return -EINVAL;

	if (touch->irq)
		free_irq(touch->irq, touch);
	else
		hrtimer_cancel(&touch->timer);
	input_unregister_device(touch->input_dev);
	kfree(touch);
	return 0;
}

static const struct i2c_device_id panjit_ts_id[] = {
	{ DRIVER_NAME, 0 },
	{ }
};

static struct i2c_driver panjit_driver = {
	.probe		= pj_probe,
	.remove		= pj_remove,
	.suspend	= pj_suspend,
	.resume		= pj_resume,
	.id_table	= panjit_ts_id,
	.driver		= {
		.name	= DRIVER_NAME,
	},
};

static int __devinit panjit_init(void)
{
	int e;

	touch_wq = create_singlethread_workqueue("touch_wq");
	if (!touch_wq) {
		pr_err("%s: could not allocate work queue\n", __func__);
		return -ENOMEM;
	}

	e = i2c_add_driver(&panjit_driver);
	if (e != 0) {
		pr_err("%s: failed to register with I2C bus with "
		       "error: 0x%x\n", __func__, e);
	}
	return e;
}

static void __exit panjit_exit(void)
{
	i2c_del_driver(&panjit_driver);
	if (touch_wq)
		destroy_workqueue(touch_wq);
}

module_init(panjit_init);
module_exit(panjit_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Panjit I2C touch driver");
