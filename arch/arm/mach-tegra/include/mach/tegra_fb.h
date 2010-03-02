/*
 * arch/arm/mach-tegra/include/mach/tegra_fb.h
 *
 * Copyright (C) 2010 Google, Inc.
 * Author: Colin Cross <ccross@android.com>
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

struct tegra_fb_lcd_data {
	int	fb_xres;
	int	fb_yres;
	int	lcd_xres;
	int	lcd_yres;
	int	bits_per_pixel;
};
