/*
 * arch/arm/mach-tegra/performance.c
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@android.com>
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
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/string.h>

#include <mach/performance.h>

#define FREQ_TABLE_END (~0L)

static unsigned long freq_table_next_higher(unsigned long *freq_table,
	unsigned long freq)
{
	unsigned long ret = FREQ_TABLE_END;
	for ( ; *freq_table != FREQ_TABLE_END; freq_table++) {
		if (*freq_table >= freq && *freq_table < ret)
			ret = *freq_table;
	}
	return ret;
}

struct performance_bus_provider {
	struct clk *clk;
	unsigned long *freq_table;
	int weight_numerator;
	int weight_denominator;
	int state;
};

static int performance_bus_set(struct performance_provider *provider, int level)
{
	struct performance_bus_provider *bus_provider = provider->data;
	unsigned long freq = level;
	if (bus_provider->weight_numerator != 0)
		freq *= bus_provider->weight_numerator;
	if (bus_provider->weight_denominator != 0)
		freq /= bus_provider->weight_denominator;

	if (bus_provider->freq_table)
		freq = freq_table_next_higher(bus_provider->freq_table, freq);
	if (freq == FREQ_TABLE_END)
		return -EINVAL;

	if (freq == 0 && bus_provider->state == 1) {
		clk_disable(bus_provider->clk);
		bus_provider->state = 0;
	} else if (freq > 0 && bus_provider->state == 0) {
		clk_enable(bus_provider->clk);
		bus_provider->state = 1;
	}

	if (freq > 0)
		return clk_set_rate(bus_provider->clk, freq);
	return 0;
}

static struct performance_provider_ops performance_bus_ops = {
	.merge = performance_provider_level_sum,
	.set = performance_bus_set,
};

unsigned long hclk_freq_table[] = {
	6750000,
	13500000,
	27000000,
	54000000,
	108000000,
	FREQ_TABLE_END,
};

struct performance_bus_provider bus_hclk_data = {
	.freq_table = hclk_freq_table,
};

struct performance_provider bus_hclk = {
	.name = "hclk",
	.ops = &performance_bus_ops,
	.min = 6750000,
	.max = 108000000,
	.data = &bus_hclk_data,
};

static int __init performance_bus_init(void)
{
	int ret;

	bus_hclk_data.clk = clk_get_sys(NULL, "pll_p_out4");
	if (!bus_hclk_data.clk) {
		pr_err("Failed to get hclk clock\n");
		return -ENODEV;
	}

	ret = performance_provider_register(&bus_hclk);
	if (ret) {
		pr_err("Failed to register hclk performance bus\n");
		clk_put(bus_hclk_data.clk);
		return ret;
	}

	return 0;
}

arch_initcall(performance_bus_init);

#ifdef CONFIG_DEBUG_FS


static struct performance_request hclk_bus_debug = {
	.name = "hclk_debug"
};

ssize_t hclk_read (struct file *file, char __user *data, size_t len,
	loff_t *off)
{
	char buf[32];
	int i;
	i = snprintf(buf, sizeof(buf), "%lu\n", clk_get_rate(bus_hclk_data.clk));
	return simple_read_from_buffer(data, len, off, buf, i);
}

ssize_t hclk_write (struct file *file, const char __user *data, size_t len,
	loff_t *off)
{
	char buf[32];
	int buf_size;
	unsigned long rate = 0;

	buf_size = min(len, (sizeof(buf)-1));
	if (copy_from_user(buf, data, buf_size))
		return -EFAULT;
	buf[buf_size] = 0;
	rate = simple_strtoul(buf, NULL, 0);
	if (rate == 0)
		return performance_request_clear(&hclk_bus_debug);
	else
		return performance_request_set(&hclk_bus_debug, rate);
}

static struct file_operations hclk_debug_ops = {
	.read = hclk_read,
	.write = hclk_write,
};

static int __init performance_bus_debug_init(void)
{
	performance_request_register(&hclk_bus_debug, "hclk");
	debugfs_create_file("hclk", S_IRUGO | S_IWUSR, NULL, NULL,
		&hclk_debug_ops);
	return 0;
}

device_initcall(performance_bus_debug_init);
#endif
