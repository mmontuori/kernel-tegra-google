/*
 * Copyright (C) 2010 NVIDIA, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */
#include <linux/i2c.h>
#include <linux/regulator/machine.h>

/* SM0 1.2V */
static struct regulator_consumer_supply tps658621_sm0_supply[] =
{
	/* Fixme:Change NULL to actual dev_name */
	REGULATOR_SUPPLY("vdd_core", NULL),
};
/* SM1 */
static struct regulator_consumer_supply tps658621_sm1_supply[] =
{
	/* Fixme:Change NULL to actual dev_name */
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};
/* SM2 */
static struct regulator_consumer_supply tps658621_sm2_supply[] =
{
	/* Fixme:Change NULL to actual dev_name */
	REGULATOR_SUPPLY("vdd_sm2", NULL),
};
/* LDO0 */
static struct regulator_consumer_supply tps658621_ldo0_supply[] =
{
	/* Fixme:Change NULL to actual dev_name */
	REGULATOR_SUPPLY("p_cam_avdd", NULL),
};
/* LDO1 */
static struct regulator_consumer_supply tps658621_ldo1_supply[] =
{
	/* Fixme:Change NULL to actual dev_name */
	REGULATOR_SUPPLY("avdd_pll", NULL),
};
/* LDO2 */
static struct regulator_consumer_supply tps658621_ldo2_supply[] =
{
	/* Fixme:Change NULL to actual dev_name */
	REGULATOR_SUPPLY("vdd_rtc", NULL),
};
/* LDO3 */
static struct regulator_consumer_supply tps658621_ldo3_supply[] =
{
	/* Fixme:Change NULL to actual dev_name */
	REGULATOR_SUPPLY("avdd_usb", NULL),
	REGULATOR_SUPPLY("avdd_usb_pll", NULL),
};
/* LDO4 */
static struct regulator_consumer_supply tps658621_ldo4_supply[] =
{
	/* Fixme:Change NULL to actual dev_name */
	REGULATOR_SUPPLY("avdd_osc", NULL),
	REGULATOR_SUPPLY("vddio_sys", "panjit_touch"),
};
/* LDO5 */
static struct regulator_consumer_supply tps658621_ldo5_supply[] =
{
	/* Fixme:Change NULL to actual dev_name */
	REGULATOR_SUPPLY("vcore_mmc", "sdhci-tegra.1"),
	REGULATOR_SUPPLY("vcore_mmc", "sdhci-tegra.3"),
};
/* LDO6 */
static struct regulator_consumer_supply tps658621_ldo6_supply[] =
{
	/* Fixme:Change NULL to actual dev_name */
	REGULATOR_SUPPLY("vddio_vi", NULL),
};
/* LDO7 */
static struct regulator_consumer_supply tps658621_ldo7_supply[] =
{
	/* Fixme:Change NULL to actual dev_name */
	REGULATOR_SUPPLY("avdd_hdmi", NULL),
	REGULATOR_SUPPLY("vdd_fuse", NULL),
};
/* LDO8 */
static struct regulator_consumer_supply tps658621_ldo8_supply[] =
{
	/* Fixme:Change NULL to actual dev_name */
	REGULATOR_SUPPLY("avdd_hdmi_pll",NULL),
};
/* LDO9 */
static struct regulator_consumer_supply tps658621_ldo9_supply[] =
{
	/* Fixme:Change NULL to actual dev_name */
	REGULATOR_SUPPLY("avdd_2v85", NULL),
	REGULATOR_SUPPLY("vdd_ddr_rx", NULL),
	REGULATOR_SUPPLY("avdd_amp", NULL),
};

static struct regulator_init_data tps658621_regulator_data[]= {
	/* SM0 */
	{
		.constraints = {
			.min_uV = 725000,
			.max_uV = 1500000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS
					| REGULATOR_CHANGE_VOLTAGE,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps658621_sm0_supply),
		.consumer_supplies = tps658621_sm0_supply,
	},

	/* SM1 */
	{
		.constraints = {
			.min_uV = 725000,
			.max_uV = 1500000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS
					| REGULATOR_CHANGE_VOLTAGE,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps658621_sm1_supply),
		.consumer_supplies = tps658621_sm1_supply,
	},

	/* SM2 */
	{
		.constraints = {
			.min_uV = 3000000,
			.max_uV = 4550000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS
					| REGULATOR_CHANGE_VOLTAGE,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps658621_sm2_supply),
		.consumer_supplies = tps658621_sm2_supply,
	},

	/* LDO0 */
	{
		.constraints = {
			.min_uV = 1250000,
			.max_uV = 3300000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS
					| REGULATOR_CHANGE_VOLTAGE,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps658621_ldo0_supply),
		.consumer_supplies = tps658621_ldo0_supply,
	},

	/* LDO1 */
	{
		.constraints = {
			.min_uV = 725000,
			.max_uV = 1500000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS
					| REGULATOR_CHANGE_VOLTAGE,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps658621_ldo1_supply),
		.consumer_supplies = tps658621_ldo1_supply,
	},

	/* LDO2 */
	{
		.constraints = {
			.min_uV = 725000,
			.max_uV = 1500000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS
					| REGULATOR_CHANGE_VOLTAGE,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps658621_ldo2_supply),
		.consumer_supplies = tps658621_ldo2_supply,
	},

	/* LDO3 */
	{
		.constraints = {
			.min_uV = 1250000,
			.max_uV = 3300000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS
					| REGULATOR_CHANGE_VOLTAGE,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps658621_ldo3_supply),
		.consumer_supplies = tps658621_ldo3_supply,
	},

	/* LDO4 */
	{
		.constraints = {
			.min_uV = 1700000,
			.max_uV = 2475000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS
					| REGULATOR_CHANGE_VOLTAGE,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps658621_ldo4_supply),
		.consumer_supplies = tps658621_ldo4_supply,
	},

	/* LDO5 */
	{
		.constraints = {
			.min_uV = 1250000,
			.max_uV = 3300000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS
					| REGULATOR_CHANGE_VOLTAGE,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps658621_ldo5_supply),
		.consumer_supplies = tps658621_ldo5_supply,
	},

	/* LDO6 */
	{
		.constraints = {
			.min_uV = 1250000,
			.max_uV = 3300000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS
					| REGULATOR_CHANGE_VOLTAGE,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps658621_ldo6_supply),
		.consumer_supplies = tps658621_ldo6_supply,
	},

	/* LDO7 */
	{
		.constraints = {
			.min_uV = 1250000,
			.max_uV = 3300000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS
					| REGULATOR_CHANGE_VOLTAGE,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps658621_ldo7_supply),
		.consumer_supplies = tps658621_ldo7_supply,
	},

	/* LDO8 */
	{
		.constraints = {
			.min_uV = 1250000,
			.max_uV = 3300000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS
					| REGULATOR_CHANGE_VOLTAGE,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps658621_ldo8_supply),
		.consumer_supplies = tps658621_ldo8_supply,
	},

	/* LDO9 */
	{
		.constraints = {
			.min_uV = 1250000,
			.max_uV = 3300000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS
					| REGULATOR_CHANGE_VOLTAGE,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps658621_ldo9_supply),
		.consumer_supplies = tps658621_ldo9_supply,
	},
};

static struct i2c_board_info __initdata ventana_i2c_dvc_board_info[] = {
	{
		I2C_BOARD_INFO("tps658621", 0x34),
		.platform_data = tps658621_regulator_data,
	},
};

int __init ventana_regulator_init(void)
{
	i2c_register_board_info(
		4, /* adapter number */
		ventana_i2c_dvc_board_info,
		ARRAY_SIZE(ventana_i2c_dvc_board_info));

	return 0;
}
