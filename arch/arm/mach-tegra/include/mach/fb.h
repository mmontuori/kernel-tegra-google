/*
 * arch/arm/mach-tegra/include/mach/fb.h
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Erik Gilling <konkers@google.com>
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

#ifndef __MACH_TEGRA_FB_H
#define __MACH_TEGRA_FB_H

struct tegra_fb_platform_data {
	int		dc;
	int		win;

	int		xres;
	int		yres;
	int		bits_per_pixel;
};


#endif
