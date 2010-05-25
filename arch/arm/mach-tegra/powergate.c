/*
 * drivers/powergate/tegra-powergate.c
 *
 * Copyright (c) 2010 Google, Inc
 *
 * Author:
 *	Colin Cross <ccross@google.com>
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

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/powergate.h>

#define PWRGATE_TOGGLE		0x30
# define PWRGATE_TOGGLE_START	(1 << 8)

#define REMOVE_CLAMPING		0x34

#define PWRGATE_STATUS		0x38

static DEFINE_SPINLOCK(tegra_powergate_lock);

static void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);

static u32 pmc_read(unsigned long reg)
{
	return readl(pmc + reg);
}

static void pmc_write(u32 val, unsigned long reg)
{
	writel(val, pmc + reg);
}

static int tegra_powergate_set(enum tegra_powergate_device id, bool new_state)
{
	bool status;
	unsigned long flags;

	spin_lock_irqsave(&tegra_powergate_lock, flags);

	status = pmc_read(PWRGATE_STATUS) & (1 << id);

	if (status == new_state) {
		spin_unlock_irqrestore(&tegra_powergate_lock, flags);
		return -EINVAL;
	}

	pmc_write(PWRGATE_TOGGLE, PWRGATE_TOGGLE_START | id);

	spin_unlock_irqrestore(&tegra_powergate_lock, flags);

	return 0;
}

int tegra_powergate_power_on(enum tegra_powergate_device id)
{
	if (id < 0 || id >= TEGRA_NUM_POWERGATE)
		return -EINVAL;

	return tegra_powergate_set(id, true);
}

int tegra_powergate_power_off(enum tegra_powergate_device id)
{
	if (id < 0 || id >= TEGRA_NUM_POWERGATE)
		return -EINVAL;

	return tegra_powergate_set(id, false);
}

int tegra_powergate_is_powered(enum tegra_powergate_device id)
{
	bool status;

	if (id < 0 || id >= TEGRA_NUM_POWERGATE)
		return -EINVAL;

	status = pmc_read(PWRGATE_STATUS) & (1 << id);

	return status ? 1 : 0;
}

int tegra_powergate_remove_clamping(enum tegra_powergate_device id)
{
	if (id < 0 || id >= TEGRA_NUM_POWERGATE)
		return -EINVAL;

	pmc_write(REMOVE_CLAMPING, 1 << id);

	return 0;
}

/* Must be called with clk disabled, and returns with clk enabled */
int tegra_powergate_sequence_power_up(enum tegra_powergate_device id,
	struct clk *clk)
{
	int ret;

	tegra_periph_reset_assert(clk);

	ret = tegra_powergate_power_on(id);
	if (ret)
		return ret;

	ret = clk_enable(clk);
	if (ret)
		return ret;

	udelay(10);

	ret = tegra_powergate_remove_clamping(id);
	if (ret)
		return ret;

	tegra_periph_reset_deassert(clk);

	return 0;
}

#ifdef CONFIG_DEBUG_FS

static const char *powergate_name[] = {
	[TEGRA_POWERGATE_CPU]	= "cpu",
	[TEGRA_POWERGATE_3D]	= "3d",
	[TEGRA_POWERGATE_VENC]	= "venc",
	[TEGRA_POWERGATE_VDEC]	= "vdec",
	[TEGRA_POWERGATE_PCIE]	= "pcie",
	[TEGRA_POWERGATE_L2]	= "l2",
	[TEGRA_POWERGATE_MPE]	= "mpe",
};

static int powergate_show(struct seq_file *s, void *data)
{
	int i;

	seq_printf(s, " powergate powered\n");
	seq_printf(s, "------------------\n");

	for (i = 0; i < TEGRA_NUM_POWERGATE; i++)
		seq_printf(s, " %9s %7s\n", powergate_name[i],
			tegra_powergate_is_powered(i) ? "yes" : "no");
	return 0;
}

static int powergate_open(struct inode *inode, struct file *file)
{
	return single_open(file, powergate_show, inode->i_private);
}

static const struct file_operations powergate_fops = {
	.open		= powergate_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init powergate_debugfs_init(void)
{
	struct dentry *d;
	int err = -ENOMEM;

	d = debugfs_create_file("powergate", S_IRUGO, NULL, NULL,
		&powergate_fops);
	if (!d)
		return -ENOMEM;

	return err;
}

late_initcall(powergate_debugfs_init);

#endif
