/*
 * linux/include/i2c/nct1008.h
 *
 * Temperature monitoring device from On Semiconductors
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

#ifndef _LINUX_NCT1008_H
#define _LINUX_NCT1008_H

/* config params */
#define CONFIG_ALERT_DISABLE	BIT(7)
#define CONFIG_RUN_STANDBY	BIT(6)
#define CONFIG_ALERT_THERM2	BIT(5)
#define CONFIG_ENABLE_EXTENDED	BIT(2)

#define OFFSET_SUBTRACT		BIT(7)

struct nct1008_platform_data {
	/* temperature conversion rate */
	int conv_rate_code;

	/* configuration parameters */
	int config;

	/* cpu shut down threshold */
	int cpu_shutdown_temp;

	/* cpu throttling thresholds */
	int cpu_throttle_hi_temp;
	int cpu_throttle_lo_temp;
	int cpu_min_freq;

	/* consecutive alert# reads */
	int num_alert_reads;

	/* temperature offset value */
	int offset;

	/* therm# hysteresis value */
	int hysteresis;
};

#endif /* _LINUX_NCT1008_H */
