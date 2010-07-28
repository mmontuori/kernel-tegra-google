/*
 * TPS658621-regulator.c
 *
 * Supports TPS658621 Regulator
 *
 * Copyright (C) 2010 NVIDIA Corporation - http://www.nvidia.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>

/* Number of step-down converters available */
#define TPS658621_NUM_SM 3
/* Number of LDO voltage regulators  available */
#define TPS658621_NUM_LDO 10
/* Number of total regulators available */
#define TPS658621_NUM_REGULATOR (TPS658621_NUM_SM + TPS658621_NUM_LDO)

/* SMs : DC to DC Converters */
#define TPS658621_SM_0 0
#define TPS658621_SM_1 1
#define TPS658621_SM_2 2

/* LDOs : Linear Dropout regulator */
#define TPS658621_LDO_0 3
#define TPS658621_LDO_1 4
#define TPS658621_LDO_2 5
#define TPS658621_LDO_3 6
#define TPS658621_LDO_4 7
#define TPS658621_LDO_5 8
#define TPS658621_LDO_6 9
#define TPS658621_LDO_7 10
#define TPS658621_LDO_8 11
#define TPS658621_LDO_9 12

/* This holds the register info for enable and disable regulator */
struct tps_reg_info {
	u8 reg1;
	u8 mask1;
	u8 reg2;
	u8 mask2;
};

static const struct tps_reg_info tps_registers[] = {
	[TPS658621_SM_0] = {0x10, 0x1<<1, 0x11, 0x1<<1},
	[TPS658621_SM_1] = {0x10, 0x1<<0, 0x11, 0x1<<0},
	[TPS658621_SM_2] = {0x12, 0x1<<7, 0x13, 0x1<<7},
	[TPS658621_LDO_0] = {0x12, 0x1<<0, 0x13, 0x1<<0},
	[TPS658621_LDO_1] = {0x12, 0x1<<1, 0x13, 0x1<<1},
	[TPS658621_LDO_2] = {0x10, 0x3<<2, 0x11, 0x3<<2},
	[TPS658621_LDO_3] = {0x12, 0x1<<2, 0x13, 0x1<<2},
	[TPS658621_LDO_4] = {0x12, 0x1<<3, 0x13, 0x1<<3},
	[TPS658621_LDO_5] = {0x14, 0x1<<6, 0, 0},
	[TPS658621_LDO_6] = {0x12, 0x1<<4, 0x13, 0x1<<4},
	[TPS658621_LDO_7] = {0x12, 0x1<<5, 0x13, 0x1<<5},
	[TPS658621_LDO_8] = {0x12, 0x1<<6, 0x13, 0x1<<5},
	[TPS658621_LDO_9] = {0x14, 0x1<<7, 0, 0},
};

/* This holds the register info to set the regulator voltage */
struct tps_voltage_info {
	u8 reg;
	u8 mask;
	u8 shift;
	unsigned int enable_time;
};

static const struct tps_voltage_info tps_voltage_registers[] = {
	[TPS658621_SM_0] = {0x26, 0x1F, 0, 3750},
	[TPS658621_SM_1] = {0x23, 0x1F, 0, 3750},
	[TPS658621_SM_2] = {0x42, 0x1F, 0, 0},
	[TPS658621_LDO_0] = {0x41, 0xE0, 5, 3750},
	[TPS658621_LDO_1] = {0x41, 0x1F, 0, 3750},
	[TPS658621_LDO_2] = {0x29, 0x1F, 0, 2500},
	[TPS658621_LDO_3] = {0x44, 0x07, 0, 2500},
	[TPS658621_LDO_4] = {0x32, 0x1F, 0, 15000},
	[TPS658621_LDO_5] = {0x46, 0x07, 0, 2500},
	[TPS658621_LDO_6] = {0x43, 0x07, 0, 15000},
	[TPS658621_LDO_7] = {0x43, 0x38, 3, 15000},
	[TPS658621_LDO_8] = {0x42, 0xE0, 5, 15000},
	[TPS658621_LDO_9] = {0x46, 0x38, 3, 2500},
};

/* Supported voltage values for regulators */
static const u16 SM0_VSEL_table[] = {
	725, 750, 775, 800,
	825, 850, 875, 900,
	925, 950, 975, 1000,
	1025, 1050, 1075, 1100,
	1125, 1150, 1175, 1200,
	1225, 1250, 1275, 1300,
	1325, 1350, 1375, 1400,
	1425, 1450, 1475, 1500,
};

static const u16 SM1_VSEL_table[] = {
	725, 750, 775, 800,
	825, 850, 875, 900,
	925, 950, 975, 1000,
	1025, 1050, 1075, 1100,
	1125, 1150, 1175, 1200,
	1225, 1250, 1275, 1300,
	1325, 1350, 1375, 1400,
	1425, 1450, 1475, 1500,
};

static const u16 SM2_VSEL_table[] = {
	3000, 3050, 3100, 3150,
	3200, 3250, 3300, 3350,
	3400, 3450, 3500, 3550,
	3600, 3650, 3700, 3750,
	3800, 3850, 3900, 3950,
	4000, 4050, 4100, 4150,
	4200, 4250, 4300, 4350,
	4400, 4450, 4500, 4550,
};

static const u16 LDO0_VSEL_table[] = {
	1250, 1500, 1800, 2500,
	2700, 2850, 3100, 3300,
};

static const u16 LDO1_VSEL_table[] = {
	725, 750, 775, 800,
	825, 850, 875, 900,
	925, 950, 975, 1000,
	1025, 1050, 1075, 1100,
	1125, 1150, 1175, 1200,
	1225, 1250, 1275, 1300,
	1325, 1350, 1375, 1400,
	1425, 1450, 1475, 1500,
};

static const u16 LDO2_VSEL_table[] = {
	725, 750, 775, 800,
	825, 850, 875, 900,
	925, 950, 975, 1000,
	1025, 1050, 1075, 1100,
	1125, 1150, 1175, 1200,
	1225, 1250, 1275, 1300,
	1325, 1350, 1375, 1400,
	1425, 1450, 1475, 1500,
};

static const u16 LDO3_VSEL_table[] = {
	1250, 1500, 1800, 2500,
	2700, 2850, 3100, 3300,
};

static const u16 LDO4_VSEL_table[] = {
	1700, 1725, 1750, 1775,
	1800, 1825, 1850, 1875,
	1900, 1925, 1950, 1975,
	2000, 2025, 2050, 2075,
	2100, 2125, 2150, 2175,
	2200, 2225, 2250, 2275,
	2300, 2325, 2350, 2375,
	2400, 2425, 2450, 2475,
};

static const u16 LDO5_VSEL_table[] = {
	1250, 1500, 1800, 2500,
	2700, 2850, 3100, 3300,
};

static const u16 LDO6_VSEL_table[] = {
	1250, 1500, 1800, 2500,
	2700, 2850, 3100, 3300,
};

static const u16 LDO7_VSEL_table[] = {
	1250, 1500, 1800, 2500,
	2700, 2850, 3100, 3300,
};

static const u16 LDO8_VSEL_table[] = {
	1250, 1500, 1800, 2500,
	2700, 2850, 3100, 3300,
};

static const u16 LDO9_VSEL_table[] = {
	1250, 1500, 1800, 2500,
	2700, 2850, 3100, 3300,
};

static unsigned int num_voltages[] = {
	ARRAY_SIZE(SM0_VSEL_table),
	ARRAY_SIZE(SM1_VSEL_table),
	ARRAY_SIZE(SM2_VSEL_table),
	ARRAY_SIZE(LDO0_VSEL_table),
	ARRAY_SIZE(LDO1_VSEL_table),
	ARRAY_SIZE(LDO2_VSEL_table),
	ARRAY_SIZE(LDO3_VSEL_table),
	ARRAY_SIZE(LDO4_VSEL_table),
	ARRAY_SIZE(LDO5_VSEL_table),
	ARRAY_SIZE(LDO6_VSEL_table),
	ARRAY_SIZE(LDO7_VSEL_table),
	ARRAY_SIZE(LDO8_VSEL_table),
	ARRAY_SIZE(LDO9_VSEL_table)
};

/* Regulator specific details */
struct tps_info {
	const char *name;
	unsigned min_uV;
	unsigned max_uV;
	u8 table_len;
	const u16 *table;
};

/* PMIC details */
struct tps_pmic {
	struct regulator_desc desc[TPS658621_NUM_REGULATOR];
	struct i2c_client *client;
	struct regulator_dev *rdev[TPS658621_NUM_REGULATOR];
	const struct tps_info *info[TPS658621_NUM_REGULATOR];
	struct mutex io_lock;
};

static inline int tps_readb(struct tps_pmic *tps, u8 reg, u8 *value)
{
	struct i2c_msg msg[2];
	int ret = 0;
	uint8_t r = reg;

	if (!tps)
		return -EINVAL;

	msg[0].addr = tps->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &r;

	/* read data */
	msg[1].addr = tps->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = value;
	ret = i2c_transfer(tps->client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&tps->client->dev, "%s: i2c_transfer(0x%02x) "
			"error %d\n", __func__, reg, ret);
	} else
		ret = 0;

	return ret;
}

static inline int tps_writeb(struct tps_pmic *tps, u8 reg, u8 value)
{
	struct i2c_msg msg;
	uint8_t data[2];
	int ret = 0;

	if (!tps)
		return -EINVAL;

	data[0] = reg;
	data[1] = value;
	msg.addr = tps->client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = data;

	ret = i2c_transfer(tps->client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&tps->client->dev, "%s: i2c_transfer(0x%02x) "
			"error %d\n", __func__, reg, ret);
	} else
		ret = 0;

	return ret;
}

static int tps_658621_set_bits(struct tps_pmic *tps, u8 reg, u8 mask)
{
	int err;
	u8 data = 0;

	mutex_lock(&tps->io_lock);

	err = tps_readb(tps, reg, &data);
	if (err)
		goto out;

	data |= mask;
	err = tps_writeb(tps, reg, data);

out:
	mutex_unlock(&tps->io_lock);
	return err;
}

static int tps_658621_clear_bits(struct tps_pmic *tps, u8 reg, u8 mask)
{
	int err;
	u8 data = 0;

	mutex_lock(&tps->io_lock);

	err = tps_readb(tps, reg, &data);
	if (err)
		goto out;

	data &= ~mask;

	err = tps_writeb(tps, reg, data);

out:
	mutex_unlock(&tps->io_lock);
	return err;

}

static int tps_658621_reg_read(struct tps_pmic *tps, u8 reg)
{
	int err;
	u8 data = 0;

	mutex_lock(&tps->io_lock);

	err = tps_readb(tps, reg, &data);

	mutex_unlock(&tps->io_lock);
	return err ? err : data;
}

static int tps_658621_reg_write(struct tps_pmic *tps, u8 reg, u8 val)
{
	int err;

	mutex_lock(&tps->io_lock);

	err = tps_writeb(tps, reg, val);

	mutex_unlock(&tps->io_lock);
	return err;
}

static int tps658621_is_enabled(struct regulator_dev *dev)
{
	struct tps_pmic *tps = rdev_get_drvdata(dev);
	const struct tps_reg_info *reg;
	int data1 = 0, data2 = 0;
	int id = rdev_get_id(dev);

	if (id < 0 || id >= ARRAY_SIZE(tps_registers)) {
		dev_err(&tps->client->dev, "%s: invalid ID %d\n", __func__, id);
		return -EINVAL;
	}

	reg = &tps_registers[id];

	data1 = tps_658621_reg_read(tps, reg->reg1);
	if (reg->reg2)
		data2 = tps_658621_reg_read(tps, reg->reg2);

	if (data1 < 0)
		return data1;

	if (data2 < 0)
		return data2;

	data1 &= reg->mask1;
	data2 &= reg->mask2;

	return (data1 | data2) ? 1 : 0;
}

static int tps658621_enable(struct regulator_dev *dev)
{
	struct tps_pmic *tps = rdev_get_drvdata(dev);
	const struct tps_reg_info *reg;
	int id = rdev_get_id(dev);
	int err = 0;

	if (id < 0 || id >= ARRAY_SIZE(tps_registers)) {
		dev_err(&tps->client->dev, "%s: invalid ID %d\n", __func__, id);
		return -EINVAL;
	}

	reg = &tps_registers[id];

	err = tps_658621_set_bits(tps, reg->reg1, reg->mask1);
	if (!err && reg->reg2)
		err = tps_658621_set_bits(tps, reg->reg2, reg->mask2);

	return err;
}

static int tps658621_disable(struct regulator_dev *dev)
{
	struct tps_pmic *tps = rdev_get_drvdata(dev);
	const struct tps_reg_info *reg;
	int id = rdev_get_id(dev);
	int err = 0;

	if (id < 0 || id >= ARRAY_SIZE(tps_registers)) {
		dev_err(&tps->client->dev, "%s: invalid ID %d\n", __func__, id);
		return -EINVAL;
	}

	reg = &tps_registers[id];

	err = tps_658621_clear_bits(tps, reg->reg1, reg->mask1);
	if (!err && reg->reg2)
		err = tps_658621_clear_bits(tps, reg->reg2, reg->mask2);

	return err;
}

static int tps658621_get_voltage(struct regulator_dev *dev)
{
	struct tps_pmic *tps = rdev_get_drvdata(dev);
	const struct tps_voltage_info *reg;
	int id = rdev_get_id(dev);
	int data = 0;
	u8 val;

	if (id < 0 || id >= ARRAY_SIZE(tps_voltage_registers)) {
		dev_err(&tps->client->dev, "%s: invalid ID %d\n", __func__, id);
		return -EINVAL;
	}

	reg = &tps_voltage_registers[id];

	data = tps_658621_reg_read(tps, reg->reg);
	if (data < 0)
		return data;

	val = (u8) data;

	val &= reg->mask;
	val >>= reg->shift;
	val &= (tps->info[id]->table_len - 1);

	return tps->info[id]->table[val] * 1000;
}

static int tps658621_set_voltage(struct regulator_dev *dev,
				 int min_uV, int max_uV)
{
	struct tps_pmic *tps = rdev_get_drvdata(dev);
	const struct tps_voltage_info *reg;
	const struct tps_info *info;
	int id = rdev_get_id(dev);
	int vsel = 0;
	int data = 0;
	u8 val;

	if (id < 0 || id >= ARRAY_SIZE(tps_voltage_registers)) {
		dev_err(&tps->client->dev, "%s: invalid ID %d\n", __func__, id);
		return -EINVAL;
	}

	info = tps->info[id];
	reg = &tps_voltage_registers[id];

	if (min_uV > info->max_uV || max_uV < info->min_uV)
		return -EINVAL;

	for (vsel = 0; vsel < info->table_len; vsel++) {
		int uV = info->table[vsel] * 1000;

		if (min_uV <= uV && uV >= max_uV)
			break;
	}

	BUG_ON(vsel == info->table_len);

	data = tps_658621_reg_read(tps, reg->reg);
	if (data < 0)
		return data;

	val = (u8) data;
	val &= ~reg->mask;
	val |= (vsel << reg->shift);

	return tps_658621_reg_write(tps, reg->reg, val);
}

static int tps658621_list_voltage(struct regulator_dev *dev, unsigned selector)
{
	struct tps_pmic *tps = rdev_get_drvdata(dev);
	int id = rdev_get_id(dev);

	if (id < 0 || id >= ARRAY_SIZE(tps_voltage_registers))
		return -EINVAL;

	if (selector >= tps->info[id]->table_len)
		return -EINVAL;
	else
		return tps->info[id]->table[selector] * 1000;
}

static int tps658621_enable_time(struct regulator_dev *dev)
{
	int id = rdev_get_id(dev);

	if (id < 0 || id >= ARRAY_SIZE(tps_voltage_registers))
		return -EINVAL;

	return tps_voltage_registers[id].enable_time;
}

static struct regulator_ops tps658621_ops = {
	.is_enabled	= tps658621_is_enabled,
	.enable		= tps658621_enable,
	.disable	= tps658621_disable,
	.get_voltage	= tps658621_get_voltage,
	.set_voltage	= tps658621_set_voltage,
	.list_voltage	= tps658621_list_voltage,
	.enable_time	= tps658621_enable_time,
};

static int __devinit tps658621_probe(struct i2c_client *client,
				     const struct i2c_device_id *id)
{
	static int desc_id;
	const struct tps_info *info = (void *)id->driver_data;
	struct regulator_init_data *init_data;
	struct regulator_dev *rdev;
	struct tps_pmic *tps;
	int i;
	int error;

	init_data = client->dev.platform_data;
	if (!init_data) {
		dev_err(&client->dev, "%s: no platform data?\n", __func__);
		return -EIO;
	}

	dev_info(&client->dev, "%s: %p\n", __func__, init_data);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: I2C_FUNC_I2C not supported\n",
			__func__);
		return -EIO;
	}

	tps = kzalloc(sizeof(*tps), GFP_KERNEL);
	if (!tps) {
		dev_err(&client->dev, "%s: allocation failed\n", __func__);
		return -ENOMEM;
	}

	mutex_init(&tps->io_lock);

	/* common for all regulators */
	tps->client = client;

	for (i = 0; i < TPS658621_NUM_REGULATOR; i++, info++, init_data++) {
		/* Store regulator specific information */
		tps->info[i] = info;

		tps->desc[i].name = info->name;
		tps->desc[i].id = desc_id++;
		tps->desc[i].n_voltages = num_voltages[i];
		tps->desc[i].ops = &tps658621_ops;
		tps->desc[i].type = REGULATOR_VOLTAGE;
		tps->desc[i].owner = THIS_MODULE;

		/* Register the regulators */
		rdev = regulator_register(&tps->desc[i], &client->dev,
					  init_data, tps);
		if (IS_ERR(rdev)) {
			dev_err(&client->dev, "%s: failed to register %s\n",
				__func__, id->name);
			error = PTR_ERR(rdev);
			goto fail;
		}

		/* Save regulator for cleanup */
		tps->rdev[i] = rdev;
	}

	i2c_set_clientdata(client, tps);
	dev_info(&client->dev, "%s: registered regulators\n", __func__);
	return 0;

fail:
	while (i--)
		regulator_unregister(tps->rdev[i]);

	kfree(tps);
	return error;
}

static int __devexit tps658621_remove(struct i2c_client *client)
{
	struct tps_pmic *tps = i2c_get_clientdata(client);
	int i;

	/* clear the client data in i2c */
	i2c_set_clientdata(client, NULL);

	for (i = 0; i < TPS658621_NUM_REGULATOR; i++)
		regulator_unregister(tps->rdev[i]);

	kfree(tps);

	return 0;
}

static const struct tps_info TPS658621_regs[] = {
	{
		.name = "sm0",
		.min_uV = 725000,
		.max_uV = 1500000,
		.table_len = ARRAY_SIZE(SM0_VSEL_table),
		.table = SM0_VSEL_table,
	},
	{
		.name = "sm1",
		.min_uV = 725000,
		.max_uV = 1500000,
		.table_len = ARRAY_SIZE(SM1_VSEL_table),
		.table = SM1_VSEL_table,
	},
	{
		.name = "sm2",
		.min_uV = 3000000,
		.max_uV = 4550000,
		.table_len = ARRAY_SIZE(SM2_VSEL_table),
		.table = SM2_VSEL_table,
	},
	{
		.name = "ldo0",
		.min_uV = 1250000,
		.max_uV = 3300000,
		.table_len = ARRAY_SIZE(LDO0_VSEL_table),
		.table = LDO0_VSEL_table,
	},
	{
		.name = "ldo1",
		.min_uV = 725000,
		.max_uV = 1500000,
		.table_len = ARRAY_SIZE(LDO1_VSEL_table),
		.table = LDO1_VSEL_table,
	},
	{
		.name = "ldo2",
		.min_uV = 725000,
		.max_uV = 1500000,
		.table_len = ARRAY_SIZE(LDO2_VSEL_table),
		.table = LDO2_VSEL_table,
	},
	{
		.name = "ldo3",
		.min_uV = 1250000,
		.max_uV = 3300000,
		.table_len = ARRAY_SIZE(LDO3_VSEL_table),
		.table = LDO3_VSEL_table,
	},
	{
		.name = "ldo4",
		.min_uV = 1700000,
		.max_uV = 2475000,
		.table_len = ARRAY_SIZE(LDO4_VSEL_table),
		.table = LDO4_VSEL_table,
	},
	{
		.name = "ldo5",
		.min_uV = 1250000,
		.max_uV = 3300000,
		.table_len = ARRAY_SIZE(LDO5_VSEL_table),
		.table = LDO5_VSEL_table,
	},
	{
		.name = "ldo6",
		.min_uV = 1250000,
		.max_uV = 3300000,
		.table_len = ARRAY_SIZE(LDO6_VSEL_table),
		.table = LDO6_VSEL_table,
	},
	{
		.name = "ldo7",
		.min_uV = 1250000,
		.max_uV = 3300000,
		.table_len = ARRAY_SIZE(LDO7_VSEL_table),
		.table = LDO7_VSEL_table,
	},
	{
		.name = "ldo8",
		.min_uV = 1250000,
		.max_uV = 3300000,
		.table_len = ARRAY_SIZE(LDO8_VSEL_table),
		.table = LDO8_VSEL_table,
	},
	{
		.name = "ldo9",
		.min_uV = 1250000,
		.max_uV = 3300000,
		.table_len = ARRAY_SIZE(LDO9_VSEL_table),
		.table = LDO9_VSEL_table,
	},
};

static const struct i2c_device_id tps658621_id[] = {
	{
		.name = "tps658621",
		.driver_data = (unsigned long) TPS658621_regs,
	},
};

MODULE_DEVICE_TABLE(i2c, tps658621_id);

static struct i2c_driver tps658621_i2c_driver = {
	.driver	= {
		.name	= "tps658621",
		.owner	= THIS_MODULE,
	},
	.probe		= tps658621_probe,
	.remove		= __devexit_p(tps658621_remove),
	.id_table	= tps658621_id,
};

static int __init tps658621_init(void)
{
	return i2c_add_driver(&tps658621_i2c_driver);
}
subsys_initcall(tps658621_init);

static void __exit tps658621_cleanup(void)
{
	i2c_del_driver(&tps658621_i2c_driver);
}
module_exit(tps658621_cleanup);

MODULE_AUTHOR("NVIDIA Corporation");
MODULE_DESCRIPTION("tps658621 voltage regulator driver");
MODULE_LICENSE("GPL v2");
