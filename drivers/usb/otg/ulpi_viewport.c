/*
 * Copyright (C) 2011 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
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

#include <linux/kernel.h>
#include <linux/usb.h>
#include <linux/io.h>
#include <linux/usb/otg.h>
#include <linux/usb/ulpi.h>

#define ULPI_VIEW_WAKEUP	(1 << 31)
#define ULPI_VIEW_RUN		(1 << 30)
#define ULPI_VIEW_WRITE		(1 << 29)
#define ULPI_VIEW_READ		(0 << 29)
#define ULPI_VIEW_ADDR(x)	(((x) & 0xff) << 16)
#define ULPI_VIEW_DATA_READ(x)	(((x) >> 8) & 0xff)
#define ULPI_VIEW_DATA_WRITE(x)	(((x) & 0xff) << 0)

static int ulpi_viewport_wait(void __iomem *view, u32 mask, u32 res)
{
	unsigned long timeout = 2000;

	while (timeout--) {
		if ((readl(view) & mask) == res)
			return 0;
		cpu_relax();
	};
	return -ETIMEDOUT;
}

static int ulpi_viewport_read(struct otg_transceiver *otg, u32 reg)
{
	int ret;
	void __iomem *view = otg->io_priv;

	writel(ULPI_VIEW_WAKEUP | ULPI_VIEW_WRITE, view);
	ret = ulpi_viewport_wait(view, ULPI_VIEW_WAKEUP, 0);
	if (ret)
		return ret;

	writel(ULPI_VIEW_RUN | ULPI_VIEW_READ | ULPI_VIEW_ADDR(reg), view);
	ret = ulpi_viewport_wait(view, ULPI_VIEW_RUN, 0);
	if (ret)
		return ret;

	return ULPI_VIEW_DATA_READ(readl(view));
}

static int ulpi_viewport_write(struct otg_transceiver *otg, u32 val, u32 reg)
{
	int ret;
	void __iomem *view = otg->io_priv;

	writel(ULPI_VIEW_WAKEUP | ULPI_VIEW_WRITE, view);
	ret = ulpi_viewport_wait(view, ULPI_VIEW_WAKEUP, 0);
	if (ret)
		return ret;

	writel(ULPI_VIEW_RUN | ULPI_VIEW_WRITE | ULPI_VIEW_DATA_WRITE(val) |
						 ULPI_VIEW_ADDR(reg), view);
	return ulpi_viewport_wait(view, ULPI_VIEW_RUN, 0);
}

struct otg_io_access_ops ulpi_viewport_access_ops = {
	.read	= ulpi_viewport_read,
	.write	= ulpi_viewport_write,
};
