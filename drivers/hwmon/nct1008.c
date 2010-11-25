/*
 * drivers/hwmon/nct1008.c
 *
 * Temperature Sensor driver for NCT1008x Temperature Monitor chip
 * manufactured by ON Semiconductors (www.onsemi.com).
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
#include <linux/i2c.h>
#include <linux/i2c/nct1008.h>
#include <linux/hwmon.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>

#include <mach/gpio.h>

#define LOCAL_TEMP_RD			0x00
#define STATUS_RD			0x02
#define CONFIG_RD			0x03
#define MFR_ID_RD			0xFE

#define CONFIG_WR			0x09
#define CONV_RATE_WR			0x0A
#define LOCAL_TEMP_HI_LIMIT_WR		0x0B
#define EXT_TEMP_HI_LIMIT_HI_BYTE	0x0D
#define OFFSET_WR			0x11
#define EXT_THERM_LIMIT_WR		0x19
#define LOCAL_THERM_LIMIT_WR		0x20
#define THERM_HYSTERESIS_WR		0x21
#define CONSECUTIVE_ALERT_WR		0x22

#define CONFIG_MASK			0x1B
#define DRIVER_NAME "nct1008x"

struct nct1008_data {
	struct device *hwmon_dev;
	struct i2c_client *client;
	struct nct1008_platform_data plat_data;
};

static ssize_t nct1008_show_temp(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	signed int temp_value = 0;
	u8 data = 0;

	if (!dev || !buf || !attr)
		return -EINVAL;

	data = i2c_smbus_read_byte_data(client, LOCAL_TEMP_RD);
	if (data < 0) {
		dev_err(&client->dev, "%s: failed to read "
			"temperature\n", __func__);
		return -EINVAL;
	}

	temp_value = (signed int)data;
	return sprintf(buf, "%d\n", temp_value);
}

static DEVICE_ATTR(temperature, S_IRUGO, nct1008_show_temp, NULL);

static struct attribute *nct1008_attributes[] = {
	&dev_attr_temperature.attr,
	NULL
};

static const struct attribute_group nct1008_attr_group = {
	.attrs = nct1008_attributes,
};

static irqreturn_t nct_irq(int irq, void *dev_id)
{
	struct nct1008_data *pdata = dev_id;
	struct i2c_client *client = pdata->client;
	struct cpufreq_policy *policy;
	int status, data, min_freq = pdata->plat_data.cpu_min_freq;
	int cur_freq, err, j;

	status = i2c_smbus_read_byte_data(client, STATUS_RD);
	if (status < 0) {
		dev_err(&client->dev, "%s: read status fail\n", __func__);
		goto out;
	}

	dev_dbg(&client->dev, "%s: status = 0x%x\n", __func__, status);
	if (!(status & BIT(6)))
		goto out;

	data = i2c_smbus_read_byte_data(client, LOCAL_TEMP_RD);
	if (data < 0) {
		dev_err(&client->dev, "%s: failed to read "
			"temperature\n", __func__);
	}
	dev_dbg(&client->dev, "%s: temp = %d\n", __func__, data);

	if (data <= pdata->plat_data.cpu_throttle_lo_temp) {
		data = pdata->plat_data.cpu_throttle_hi_temp;
		err = i2c_smbus_write_byte_data(client, LOCAL_TEMP_HI_LIMIT_WR,
			data);
		if (err < 0) {
			dev_err(&client->dev,
				"%s: set LOCAL_TEMP limit fail\n", __func__);
			goto out;
		}

		err = i2c_smbus_write_byte_data(client,
			EXT_TEMP_HI_LIMIT_HI_BYTE, data);
		if (err < 0) {
			dev_err(&client->dev,
				"%s: set EXT_TEMP limit fail\n", __func__);
		}
		goto out;
	}

	data = pdata->plat_data.cpu_throttle_lo_temp;
	err = i2c_smbus_write_byte_data(client, LOCAL_TEMP_HI_LIMIT_WR,
		data);
	if (err < 0)
		dev_err(&client->dev,
			"%s: set LOCAL_TEMP limit fail\n", __func__);

	err = i2c_smbus_write_byte_data(client, EXT_TEMP_HI_LIMIT_HI_BYTE,
		data);
	if (err < 0)
		dev_err(&client->dev,
			"%s: set EXT_TEMP limit fail\n", __func__);

	for_each_online_cpu(j) {
		cur_freq = cpufreq_get(j);
		if (cur_freq > min_freq) {
			policy = cpufreq_cpu_get(j);
			cur_freq -= 100000;
			dev_dbg(&client->dev, "cpu freq = 0x%x\n", cur_freq);
			__cpufreq_driver_target(policy, cur_freq,
				CPUFREQ_RELATION_H);
		}
	}

out:
	return IRQ_HANDLED;
}

static int __devinit nct1008_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct nct1008_data *pdata;
	int err;
	int i;
	u8 data = 0;

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL, exiting\n");
		return -ENODEV;
	}

	pdata = kzalloc(sizeof(struct nct1008_data), GFP_KERNEL);
	if (!pdata) {
		dev_err(&client->dev, "%s: failed to allocate "
			"device\n", __func__);
		return -ENOMEM;
	}

	pdata->client = client;
	i2c_set_clientdata(client, pdata);

	memcpy(&pdata->plat_data, client->dev.platform_data,
		sizeof(struct nct1008_platform_data));

	/* set conversion rate (conv/sec) */
	data = pdata->plat_data.conv_rate_code;
	err = i2c_smbus_write_byte_data(client, CONV_RATE_WR, data);
	if (err < 0) {
		dev_err(&client->dev, "%s: failed to set conversion "
			"rate\n", __func__);
		goto fail_alloc;
	}

	data = 0;
	for (i = 0; i < pdata->plat_data.num_alert_reads; i++)
		data |= BIT(i);

	err = i2c_smbus_write_byte_data(client, CONSECUTIVE_ALERT_WR, data);
	if (err < 0) {
		dev_err(&client->dev, "%s: failed to set CONSECUTIVE_ALERT\n",
			__func__);
		goto fail_alloc;
	}

	data = i2c_smbus_read_byte_data(client, CONFIG_RD);
	if (data < 0) {
		dev_err(&client->dev, "%s: failed to read config\n", __func__);
		err = data;
		goto fail_alloc;
	}

	/* set config params */
	data |= (pdata->plat_data.config & ~CONFIG_MASK);
	err = i2c_smbus_write_byte_data(client, CONFIG_WR, data);
	if (err < 0) {
		dev_err(&client->dev, "%s: failed to set config\n", __func__);
		goto fail_alloc;
	}

	/* set cpu shutdown threshold */
	data = pdata->plat_data.cpu_shutdown_temp;
	err = i2c_smbus_write_byte_data(client, LOCAL_THERM_LIMIT_WR, data);
	if (err < 0) {
		dev_err(&client->dev,
			"%s: failed to set LOCAL_THERM limit\n", __func__);
		goto fail_alloc;
	}

	err = i2c_smbus_write_byte_data(client, EXT_THERM_LIMIT_WR, data);
	if (err < 0) {
		dev_err(&client->dev,
			"%s: failed to set EXT_THERM limit\n", __func__);
		goto fail_alloc;
	}

	/* set offset register */
	data = pdata->plat_data.offset;
	err = i2c_smbus_write_byte_data(client, OFFSET_WR, data);
	if (err < 0) {
		dev_err(&client->dev,
			"%s: failed to set offset\n", __func__);
		goto fail_alloc;
	}

	/* set hysteresis */
	data = pdata->plat_data.hysteresis;
	err = i2c_smbus_write_byte_data(client, THERM_HYSTERESIS_WR, data);
	if (err < 0) {
		dev_err(&client->dev,
			"%s: failed to set hysteresis\n", __func__);
		goto fail_alloc;
	}

	/* set cpu throttling threshold */
	data = pdata->plat_data.cpu_throttle_hi_temp;
	err = i2c_smbus_write_byte_data(client, LOCAL_TEMP_HI_LIMIT_WR, data);
	if (err < 0) {
		dev_err(&client->dev,
			"%s: failed to set LOCAL_TEMP limit\n", __func__);
		goto fail_alloc;
	}

	err = i2c_smbus_write_byte_data(client, EXT_TEMP_HI_LIMIT_HI_BYTE, data);
	if (err < 0) {
		dev_err(&client->dev,
			"%s: failed to set EXT_TEMP limit\n", __func__);
		goto fail_alloc;
	}

	/* irq for cpu throttling */
	err = request_threaded_irq(pdata->client->irq, NULL, nct_irq,
		IRQF_ONESHOT | IRQF_TRIGGER_LOW, DRIVER_NAME, pdata);
	if (err < 0) {
		dev_err(&client->dev,
			"%s: request_threaded_irq fail(%d)\n", __func__, err);
		goto fail_alloc;
	}

	pdata->hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(pdata->hwmon_dev)) {
		err = PTR_ERR(pdata->hwmon_dev);
		dev_err(&client->dev, "%s: hwmon_device_register "
			"failed\n", __func__);
		goto fail_irq;
	}

	data = i2c_smbus_read_byte_data(client, MFR_ID_RD);
	if (data < 0) {
		dev_err(&client->dev, "%s: failed to read manufacturer "
			"id\n", __func__);
		err = data;
		goto fail_irq;
	}

	/* register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &nct1008_attr_group);
	if (err < 0)
		goto fail_sys;

	dev_info(&client->dev, "%s:0x%x initialized\n", client->name, data);
	return 0;

fail_sys:
	hwmon_device_unregister(pdata->hwmon_dev);
fail_irq:
	free_irq(pdata->client->irq, pdata);
fail_alloc:
	kfree(pdata);
	return err;
}

static int __devexit nct1008_remove(struct i2c_client *client)
{
	struct nct1008_data *pdata = i2c_get_clientdata(client);

	if (!pdata)
		return -EINVAL;

	free_irq(pdata->client->irq, pdata);
	hwmon_device_unregister(pdata->hwmon_dev);
	kfree(pdata);
	return 0;
}

#ifdef CONFIG_PM
static int nct1008_suspend(struct i2c_client *client, pm_message_t state)
{
	u8 config;
	int err;

	disable_irq(client->irq);
	config = i2c_smbus_read_byte_data(client, CONFIG_RD);
	if (config < 0) {
		dev_err(&client->dev, "%s: failed to read config\n", __func__);
		return -EIO;
	}

	/* take device to standby state */
	config |= CONFIG_RUN_STANDBY;

	err = i2c_smbus_write_byte_data(client, CONFIG_WR, config);
	if (err < 0) {
		dev_err(&client->dev, "%s: failed to set config\n", __func__);
		return -EIO;
	}

	return 0;
}

static int nct1008_resume(struct i2c_client *client)
{
	u8 config = 0;
	int err;

	config = i2c_smbus_read_byte_data(client, CONFIG_RD);
	if (config < 0) {
		dev_err(&client->dev, "%s: failed to read config\n", __func__);
		return -EIO;
	}

	/* take device out of standby state */
	config &= ~CONFIG_RUN_STANDBY;

	err = i2c_smbus_write_byte_data(client, CONFIG_WR, config);
	if (err < 0) {
		dev_err(&client->dev, "%s: failed to set config\n", __func__);
		return -EIO;
	}
	enable_irq(client->irq);

	return 0;
}
#endif

static const struct i2c_device_id nct1008_id[] = {
	{ DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nct1008_id);

static struct i2c_driver nct1008_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= DRIVER_NAME,
	},
	.probe		= nct1008_probe,
	.remove		= __devexit_p(nct1008_remove),
	.id_table	= nct1008_id,
#ifdef CONFIG_PM
	.suspend = nct1008_suspend,
	.resume = nct1008_resume,
#endif
};

static int __init nct1008_init(void)
{
	return i2c_add_driver(&nct1008_driver);
}

static void __exit nct1008_exit(void)
{
	i2c_del_driver(&nct1008_driver);
}

module_init (nct1008_init);
module_exit (nct1008_exit);

#define DRIVER_DESC "NCT1008 temperature sensor driver"
#define DRIVER_LICENSE "GPL"

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE(DRIVER_LICENSE);
