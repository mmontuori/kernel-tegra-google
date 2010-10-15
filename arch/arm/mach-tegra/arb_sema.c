/*
 * Copyright (C) 2010, NVIDIA Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/semaphore.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/mutex.h>

#include <mach/arb_sema.h>
#include <mach/irqs.h>
#include <mach/iomap.h>

#define TEGRA_RPC_MAX_SEM 32

#define ARB_CPU_INT_EN		0x4
#define ARB_GRANT_STATUS	0x0
#define ARB_GRANT_REQUEST	0x4
#define ARB_GRANT_RELEASE	0x8

struct tegra_arb_dev {
	void __iomem	*sema_base;
	void __iomem	*gnt_base;
	spinlock_t lock;
	wait_queue_head_t arb_gnt_q;
	int arb_gnt_wake[TEGRA_RPC_MAX_SEM];
	struct mutex mutexes[TEGRA_RPC_MAX_SEM];
	int irq;
	int status;
};

static struct tegra_arb_dev *arb;

static inline u32 arb_sema_read(struct tegra_arb_dev *dev, u32 offset)
{
	return readl(dev->sema_base + offset);
}

static inline void arb_sema_write(struct tegra_arb_dev *dev, u32 offset,
	u32 value)
{
	writel(value, dev->sema_base + offset);
}

static inline u32 arb_gnt_read(struct tegra_arb_dev *dev, u32 offset)
{
	return readl(dev->gnt_base + offset);
}

static inline void arb_gnt_write(struct tegra_arb_dev *dev, u32 offset,
	u32 value)
{
	writel(value, dev->gnt_base + offset);
}

static void request_arb_sem(enum tegra_arb_module lock)
{
	unsigned long irq_state;
	u32 value;

	spin_lock_irqsave(&arb->lock, irq_state);

	arb_sema_write(arb, ARB_GRANT_REQUEST, 1 << lock);
	value = arb_gnt_read(arb, ARB_CPU_INT_EN);
	value |= (1 << lock);
	arb_gnt_write(arb, ARB_CPU_INT_EN, value);

	spin_unlock_irqrestore(&arb->lock, irq_state);
}

static void cancel_arb_sem(enum tegra_arb_module lock)
{
	unsigned long irq_state;
	u32 value;

	spin_lock_irqsave(&arb->lock, irq_state);

	value = arb_sema_read(arb, ARB_GRANT_REQUEST);
	value &= ~(1 << lock);
	arb_sema_write(arb, ARB_GRANT_REQUEST, value);
	value = arb_gnt_read(arb, ARB_CPU_INT_EN);
	value &= ~(1 << lock);
	arb_gnt_write(arb, ARB_CPU_INT_EN, value);

	spin_unlock_irqrestore(&arb->lock, irq_state);
}

int tegra_arb_mutex_lock_timeout(enum tegra_arb_module lock, u16 timeout)
{
	long jiffies;

	mutex_lock(&arb->mutexes[lock]);
	request_arb_sem(lock);
	jiffies = msecs_to_jiffies(timeout);
	if (wait_event_timeout(arb->arb_gnt_q, arb->arb_gnt_wake[lock],
		jiffies) == 0) {
		pr_err("timed out.\n");
		cancel_arb_sem(lock);
		mutex_unlock(&arb->mutexes[lock]);
		return -ETIMEDOUT;
	}
	arb->arb_gnt_wake[lock] = 0;
	return 0;
}
EXPORT_SYMBOL(tegra_arb_mutex_lock_timeout);

int tegra_arb_mutex_unlock(enum tegra_arb_module lock)
{
	u32 ier;
	unsigned long irq_state;

	arb_sema_write(arb, ARB_GRANT_RELEASE, 1 << lock);
	spin_lock_irqsave(&arb->lock, irq_state);

	ier = arb_sema_read(arb, ARB_CPU_INT_EN);
	arb_sema_write(arb, ARB_CPU_INT_EN, ier | (1 << lock));
	arb_gnt_write(arb, ARB_GRANT_REQUEST, 1 << lock);

	spin_unlock_irqrestore(&arb->lock, irq_state);
	mutex_unlock(&arb->mutexes[lock]);
	return 0;
}
EXPORT_SYMBOL(tegra_arb_mutex_unlock);

static irqreturn_t arb_gnt_isr(int irq, void *dev_id)
{
	struct tegra_arb_dev *dev = dev_id;
	unsigned long status;
	u32 cpu_int_en;
	unsigned int bit;
	unsigned long irq_state;

	status = arb_gnt_read(dev, ARB_GRANT_STATUS);

	spin_lock_irqsave(&arb->lock, irq_state);

	/* disable the arb semaphores which were signalled */
	cpu_int_en = arb_gnt_read(dev, ARB_CPU_INT_EN);
	arb_gnt_write(dev, ARB_CPU_INT_EN,
		(cpu_int_en & ~(status & cpu_int_en)));

	for_each_set_bit(bit, &status, BITS_PER_LONG) {
			dev->arb_gnt_wake[bit] = 1;
			wake_up(&dev->arb_gnt_q);
	}

	spin_unlock_irqrestore(&arb->lock, irq_state);
	return IRQ_HANDLED;
}

int tegra_arb_suspend(void)
{
	unsigned long status = arb_gnt_read(arb, ARB_GRANT_STATUS);

	if (WARN_ON(status != 0)) {
		pr_err("%s: suspending while holding arbitration "
		       "semaphore: %08lx\n", __func__, status);
	}

	return status ? -EBUSY : 0;
}

int tegra_arb_resume(void)
{
	return 0;
}

static int __init tegra_arb_init(void)
{
	struct tegra_arb_dev *dev = NULL;
	int err, i;

	dev = kzalloc(sizeof(struct tegra_arb_dev), GFP_KERNEL);
	if (dev == NULL) {
		pr_err("%s: unable to alloc data struct.\n", __func__);
		return -ENOMEM;
	}

	for (i = 0; i < TEGRA_RPC_MAX_SEM; i++) {
		init_waitqueue_head(&dev->arb_gnt_q);
		mutex_init(&dev->mutexes[i]);
	}

	dev->sema_base = IO_ADDRESS(TEGRA_ARB_SEMA_BASE);
	if (!dev->sema_base) {
		pr_err("%s: can't get arb sema_base\n", __func__);
		err = -ENOMEM;
		goto out;
	}

	dev->gnt_base = IO_ADDRESS(TEGRA_ARBGNT_ICTLR_BASE);
	if (!dev->gnt_base) {
		pr_err("%s: can't ioremap gnt_base\n", __func__);
		err = -ENOMEM;
		goto out;
	}

	dev->irq = INT_GNT_1;
	err = request_irq(dev->irq, arb_gnt_isr, 0, "rpc-arbsema", dev);
	if (err) {
		pr_err("%s: request_irq(%d) failed(%d)\n", __func__,
			dev->irq, err);
		goto out;
	}

	spin_lock_init(&dev->lock);
	arb = dev;

	pr_info("%s: initialized\n", __func__);
	return 0;

out:
	kfree(dev);
	pr_err("%s: initialization failed.\n", __func__);
	return err;
}
subsys_initcall(tegra_arb_init);

MODULE_LICENSE("GPLv2");
