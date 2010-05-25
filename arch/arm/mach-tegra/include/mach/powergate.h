/*
 * drivers/regulator/tegra-regulator.c
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

#ifndef _MACH_TEGRA_POWERGATE_H_
#define _MACH_TEGRA_POWERGATE_H_

enum tegra_powergate_device {
	TEGRA_POWERGATE_CPU	= 0,
	TEGRA_POWERGATE_3D,
	TEGRA_POWERGATE_VENC,
	TEGRA_POWERGATE_VDEC,
	TEGRA_POWERGATE_PCIE,
	TEGRA_POWERGATE_L2,
	TEGRA_POWERGATE_MPE,
	TEGRA_NUM_POWERGATE,
};

int tegra_powergate_power_on(enum tegra_powergate_device id);
int tegra_powergate_power_off(enum tegra_powergate_device id);
int tegra_powergate_is_powered(enum tegra_powergate_device id);
int tegra_powergate_remove_clamping(enum tegra_powergate_device id);
int tegra_powergate_sequence_power_up(enum tegra_powergate_device id,
	struct clk *clk);

#endif /* _MACH_TEGRA_POWERGATE_H_ */
