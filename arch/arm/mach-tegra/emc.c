/*
 * Copyright (C) 2010 Google, Inc.
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
#include <linux/err.h>
#include <linux/io.h>

#include <mach/iomap.h>

static void __iomem *emc = IO_ADDRESS(TEGRA_EMC_BASE);
static unsigned long max_mem_rate;
static unsigned long max_refresh_rate;

#define EMC_REFRESH 0x70

void tegra_emc_set_rate(unsigned long rate)
{
	unsigned long ratio = max_mem_rate * 2 / rate;
	unsigned long refresh;

	if (!ratio)
		return;

	refresh = (DIV_ROUND_UP(max_refresh_rate * 2, ratio) - 2) << 5 | 0x1f;

	__raw_writel(refresh, emc + EMC_REFRESH);
}

void tegra_init_emc(void)
{
	struct clk *clk;
	unsigned long refresh;

	clk = clk_get_sys(NULL, "emc");
	if (IS_ERR(clk)) {
		pr_err("%s: Failed to get emc clock\n", __func__);
		return;
	}

	max_mem_rate = clk_get_rate(clk);
	clk_put(clk);

	refresh = __raw_readl(emc + EMC_REFRESH);
	max_refresh_rate = (refresh >> 5) + 2;
}
