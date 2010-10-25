/*
 * arch/arm/mach-tegra/tegra2_fuse.c
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

/*
 * Fuses are one time programmable bits on the chip which are used by
 * the chip manufacturer and device manufacturers to store chip/device
 * configurations. The fuse bits are encapsulated in a 32 x 64 array.
 * If a fuse bit is programmed to 1, it cannot be reverted to 0. Either
 * another fuse bit has to be used for the same purpose or a new chip
 * needs to be used.
 *
 * Each and every fuse word has its own shadow word which resides adjacent to
 * a particular fuse word. e.g. Fuse words 0-1 form a fuse-shadow pair.
 * So in theory we have only 32 fuse words to work with.
 * The shadow fuse word is a mirror of the actual fuse word at all times
 * and this is maintained while programming a particular fuse.
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include <mach/tegra2_fuse.h>

#include "fuse.h"

#define NFUSES	64
#define STATE_IDLE	(0x4 << 16)

/* since fuse burning is irreversible, use this for testing */
#define ENABLE_FUSE_BURNING 1

/* fuse registers */
#define FUSE_CTRL	0x000
#define FUSE_REG_ADDR		0x004
#define FUSE_REG_READ		0x008
#define FUSE_REG_WRITE		0x00C
#define FUSE_TIME_PGM		0x01C
#define FUSE_PRIV2INTFC		0x020
#define FUSE_DIS_PGM		0x02C
#define FUSE_PWR_GOOD_SW	0x034
#define FUSE_PRIVATE_KEY0	0x050
#define FUSE_PRIVATE_KEY1	0x054
#define FUSE_PRIVATE_KEY2	0x058
#define FUSE_PRIVATE_KEY3	0x05C
#define FUSE_PRIVATE_KEY4	0x060
#define FUSE_PROD_MODE		0x100
#define FUSE_SECURITY_MODE	0x1A0
#define FUSE_ARM_DEBUG_DIS	0x1B8
#define FUSE_BOOT_DEV_INFO	0x1BC
#define FUSE_RSVD_SW		0x1C0
#define FUSE_RSVD_ODM_BASE	0x1C8

static u32 fuse_pgm_data[NFUSES];
static u32 fuse_pgm_mask[NFUSES];
static u32 tmp_fuse_pgm_data[NFUSES];
static u8 master_enable;

DEFINE_MUTEX(fuse_lock);

static struct fuse_data fuse_info;

struct param_info {
	void *addr;
	int sz;
	u32 start_off;
	int start_bit;
	u32 end_off;
	u32 high_mask;
	u32 low_mask;
	int data_offset;
};

static struct param_info fuse_info_tbl[] = {
	[DEVKEY] = {
		.addr = fuse_info.devkey,
		.sz = ARRAY_SIZE(fuse_info.devkey),
		.start_off = 0x12,
		.start_bit = 8,
		.end_off = 0x14,
		.low_mask = 0xFFFFFF00,
		.high_mask = 0xFF,
		.data_offset = 0,
	},
	[JTAG_DIS] = {
		.addr = &fuse_info.jtag_dis,
		.sz = sizeof(fuse_info.jtag_dis),
		.start_off = 0x0,
		.start_bit = 24,
		.end_off = 0x0,
		.low_mask = 1 << 24,
		.high_mask = 0,
		.data_offset = 4,
	},
	[ODM_PROD_MODE] = {
		.addr = &fuse_info.odm_prod_mode,
		.sz = sizeof(fuse_info.odm_prod_mode),
		.start_off = 0x0,
		.start_bit = 23,
		.end_off = 0x0,
		.low_mask = 1 << 23,
		.high_mask = 0,
		.data_offset = 5,
	},
	[SEC_BOOT_DEV_CFG] = {
		.addr = fuse_info.bootdev_cfg,
		.sz = ARRAY_SIZE(fuse_info.bootdev_cfg),
		.start_off = 0x14,
		.start_bit = 8,
		.end_off = 0x14,
		.low_mask = 0xFFFF << 8,
		.high_mask = 0,
		.data_offset = 6,
	},
	[SEC_BOOT_DEV_SEL] = {
		.addr = &fuse_info.bootdev_sel,
		.sz = sizeof(fuse_info.bootdev_sel),
		.start_off = 0x14,
		.start_bit = 24,
		.end_off = 0x14,
		.low_mask = 0x7 << 24,
		.high_mask = 0,
		.data_offset = 8,
	},
	[SBK] = {
		.addr = fuse_info.sbk,
		.sz = ARRAY_SIZE(fuse_info.sbk),
		.start_off = 0x0A,
		.start_bit = 8,
		.end_off = 0x12,
		.low_mask = 0xFFFFFF00,
		.high_mask = 0xFF,
		.data_offset = 9,
	},
	[SW_RSVD] = {
		.addr = &fuse_info.sw_rsvd,
		.sz = sizeof(fuse_info.sw_rsvd),
		.start_off = 0x14,
		.start_bit = 28,
		.end_off = 0x14,
		.low_mask = 0xF << 28,
		.high_mask = 0,
		.data_offset = 25,
	},
	[IGNORE_DEV_SEL_STRAPS] = {
		.addr = &fuse_info.ignore_devsel_straps,
		.sz = sizeof(fuse_info.ignore_devsel_straps),
		.start_off = 0x14,
		.start_bit = 27,
		.end_off = 0x14,
		.low_mask = 1 << 27,
		.high_mask = 0,
		.data_offset = 26,
	},
	[ODM_RSVD] = {
		.addr = &fuse_info.odm_rsvd,
		.sz = ARRAY_SIZE(fuse_info.odm_rsvd),
		.start_off = 0x16,
		.start_bit = 4,
		.end_off = 0x26,
		.low_mask = 0xFFFFFFF0,
		.high_mask = 0xF,
		.data_offset = 27,
	},
	[MASTER_ENB] = {
		.addr = &master_enable,
		.sz = sizeof(u8),
		.start_off = 0x0,
		.start_bit = 0,
		.end_off = 0x0,
		.low_mask = 0x1,
		.high_mask = 0,
	},
};

static void wait_for_idle(void)
{
	u32 reg;

	do {
		reg = tegra_fuse_readl(FUSE_CTRL);
	} while ((reg & (0xF << 16)) != STATE_IDLE);
}

#define FUSE_READ	0x1
#define FUSE_WRITE	0x2
#define FUSE_SENSE	0x3
#define FUSE_CMD_MASK	0x3

static u32 fuse_cmd_read(u32 addr)
{
	u32 reg;

	tegra_fuse_writel(addr, FUSE_REG_ADDR);
	reg = tegra_fuse_readl(FUSE_CTRL);
	reg &= ~FUSE_CMD_MASK;
	reg |= FUSE_READ; /* read */
	tegra_fuse_writel(reg, FUSE_CTRL);
	wait_for_idle();

	reg = tegra_fuse_readl(FUSE_REG_READ);
	return reg;
}

static void fuse_cmd_write(u32 value, u32 addr)
{
	u32 reg;

	tegra_fuse_writel(addr, FUSE_REG_ADDR);
	tegra_fuse_writel(value, FUSE_REG_WRITE);

	reg = tegra_fuse_readl(FUSE_CTRL);
	reg &= ~FUSE_CMD_MASK;
	reg |= FUSE_WRITE; /* write */
	tegra_fuse_writel(reg, FUSE_CTRL);
	wait_for_idle();
}

static void fuse_cmd_sense(void)
{
	u32 reg;

	reg = tegra_fuse_readl(FUSE_CTRL);
	reg &= ~FUSE_CMD_MASK;
	reg |= FUSE_SENSE; /* sense */
	tegra_fuse_writel(reg, FUSE_CTRL);
	wait_for_idle();
}

static void fuse_reg_hide(void)
{
	u32 reg = tegra_fuse_readl(0x48);
	reg &= ~(1 << 28);
	tegra_fuse_writel(reg, 0x48);
}

static void fuse_reg_unhide(void)
{
	u32 reg = tegra_fuse_readl(0x48);
	reg |= (1 << 28);
	tegra_fuse_writel(reg, 0x48);
}

static bool fuse_odm_prod_mode(void)
{
	return (tegra_fuse_readl(FUSE_SECURITY_MODE) ? true : false);
}

static void get_fuse(enum fuse_io_param io_param)
{
	int i, start = fuse_info_tbl[io_param].start_off;
	int end = fuse_info_tbl[io_param].end_off;
	int start_bit = fuse_info_tbl[io_param].start_bit;
	void *fuse = fuse_info_tbl[io_param].addr;
	u32 tmp[NFUSES], val, *dst = (u32 *)fuse;
	u32 low_mask = fuse_info_tbl[io_param].low_mask;
	u32 high_mask = fuse_info_tbl[io_param].high_mask;

	for (i = 0; i < NFUSES; i++)
		tmp[i] = fuse_cmd_read(i);

	for (i = start; i < end; i += 2, dst++) {
		val = tmp[i];

		if (i == start)
			val &= low_mask;

		val >>= start_bit;
		if (end == start) {
			memcpy(dst, &val, fuse_info_tbl[io_param].sz);
			break;
		}

		val |= ((tmp[i + 2] & high_mask) << (31 - start_bit + 1));
		*dst = val;
	}
}

int tegra_fuse_read(enum fuse_io_param io_param, void *data, int size)
{
	int ret = 0;

	if (!data)
		return -EINVAL;

	if (size != fuse_info_tbl[io_param].sz)
		return -EINVAL;

	mutex_lock(&fuse_lock);
	fuse_reg_unhide();
	fuse_cmd_sense();

	if (io_param == SBK_DEVKEY_STATUS) {
		get_fuse(SBK);
		get_fuse(DEVKEY);
		if (fuse_info.sbk || fuse_info.devkey)
			*(u8 *)data = 1;
		else
			*(u8 *)data = 0;
	} else {
		get_fuse(io_param);
		memcpy(data, fuse_info_tbl[io_param].addr, size);
	}

	fuse_reg_hide();
	mutex_unlock(&fuse_lock);
	return ret;
}

static void set_fuse(enum fuse_io_param io_param, u32 *data)
{
	int start = fuse_info_tbl[io_param].start_off;
	int end = fuse_info_tbl[io_param].end_off;
	int start_bit = fuse_info_tbl[io_param].start_bit;
	int i, dlen = fuse_info_tbl[io_param].sz / sizeof(u32);
	u32 mask, val, overflow = 0;
	u32 low_mask = fuse_info_tbl[io_param].low_mask;
	u32 high_mask = fuse_info_tbl[io_param].high_mask;

	if (dlen > 1) {
		mask = 0xFFFFFFFF << (31 - start_bit + 1);
		overflow = data[dlen - 1] & mask;
		overflow >>= (31 - start_bit + 1);
		data[dlen - 1] <<= start_bit;
		for (i = dlen - 1; i >= 0; i--) {
			data[i] |= (be32_to_cpu(data[i - 1]) & high_mask);
			data[i - 1] <<= start_bit;
		}
	} else {
		*data <<= start_bit;
	}

	for (i = start; i <= end; i += 2) {
		mask = 0xFFFFFFFF;
		val = *data;

		if (start == end) {
			fuse_pgm_data[i] &= ~low_mask;
			val &= low_mask;
			fuse_pgm_data[i] |= val;
			fuse_pgm_data[i + 1] = fuse_pgm_data[i];
			fuse_pgm_mask[i] |= low_mask;
			fuse_pgm_mask[i + 1] = fuse_pgm_mask[i];
			break;
		}

		if (i == start) {
			mask <<= start_bit;
		} else if (i == end) {
			mask = high_mask;
			val = overflow;
		}

		fuse_pgm_data[i] &= ~mask;
		val &= mask;
		fuse_pgm_data[i] |= val;
		fuse_pgm_data[i + 1] = fuse_pgm_data[i];
		fuse_pgm_mask[i] |= mask;
		fuse_pgm_mask[i + 1] = fuse_pgm_mask[i];
		data++;
	}
}

static void populate_fuse_arrs(struct fuse_data *info, u32 flags)
{
	u32 data = 0;
	u8 *src = (u8 *)info;
	int i;

	memset(fuse_pgm_data, 0, ARRAY_SIZE(fuse_pgm_data));
	memset(fuse_pgm_mask, 0, ARRAY_SIZE(fuse_pgm_mask));

	/* enable program bit */
	data = 1;
	set_fuse(MASTER_ENB, &data);

	if ((flags & FLAGS_ODMRSVD)) {
		src = (u8 *)info->odm_rsvd;
		set_fuse(ODM_RSVD, (u32 *)src);
		flags &= ~FLAGS_ODMRSVD;
	}

	/* do not burn any more if secure mode is set */
	if (fuse_odm_prod_mode())
		goto out;

	for_each_set_bit(i, (unsigned long *)&flags, MAX_PARAMS) {
		if (fuse_info_tbl[i].sz <= sizeof(u32)) {
			data = (u32)(*(src + fuse_info_tbl[i].data_offset));
			set_fuse(i, &data);
		} else {
			set_fuse(i, (u32 *)(src +
				fuse_info_tbl[i].data_offset));
		}
	}

out:
	pr_debug("ready to program");
}

static void fuse_power_enable(void)
{
#if ENABLE_FUSE_BURNING
	tegra_fuse_writel(0x1, FUSE_PWR_GOOD_SW);
	udelay(1);
#endif
}

static void fuse_power_disable(void)
{
#if ENABLE_FUSE_BURNING
	tegra_fuse_writel(0, FUSE_PWR_GOOD_SW);
	udelay(1);
#endif
}

static void fuse_program_array(int pgm_cycles)
{
	u32 reg, fuse_val[2];
	u32 *data = tmp_fuse_pgm_data, addr = 0, *mask = fuse_pgm_mask;
	int i;

	fuse_reg_unhide();
	fuse_cmd_sense();

	/* get the first 2 fuse bytes */
	fuse_val[0] = fuse_cmd_read(0);
	fuse_val[1] = fuse_cmd_read(1);

	fuse_power_enable();

	/*
	 * The fuse macro is a high density macro. Fuses are
	 * burned using an addressing mechanism, so no need to prepare
	 * the full list, but more write to control registers are needed.
	 * The only bit that can be written at first is bit 0, a special write
	 * protection bit by assumptions all other bits are at 0
	 *
	 * The programming pulse must have a precise width of
	 * [9000, 11000] ns.
	 */
	if (pgm_cycles > 0) {
		reg = pgm_cycles;
		tegra_fuse_writel(reg, FUSE_TIME_PGM);
	}
	fuse_val[0] = (0x1 & ~fuse_val[0]);
	fuse_val[1] = (0x1 & ~fuse_val[1]);
	fuse_cmd_write(fuse_val[0], 0);
	fuse_cmd_write(fuse_val[1], 1);

	fuse_power_disable();

	/*
	 * this will allow programming of other fuses
	 * and the reading of the existing fuse values
	 */
	fuse_cmd_sense();

	/* Clear out all bits that have already been burned or masked out */
	memcpy(data, fuse_pgm_data, NFUSES * sizeof(u32));

	for (i = 0; i < NFUSES; i++, addr++, data++, mask++) {
		reg = fuse_cmd_read(addr);
		pr_debug("%d: 0x%x 0x%x 0x%x\n", i, (u32)(*data),
			~reg, (u32)(*mask));
		*data = (*data & ~reg) & *mask;
	}

	fuse_power_enable();

	/*
	 * Finally loop on all fuses, program the non zero ones.
	 * Words 0 and 1 are written last and they contain control fuses. We
	 * need to invalidate after writing to a control word (with the exception
	 * of the master enable). This is also the reason we write them last.
	 */
	addr = NFUSES;
	while (addr--) {
		if (tmp_fuse_pgm_data[addr])
			fuse_cmd_write(tmp_fuse_pgm_data[addr], addr);

		if (addr < 2) {
			fuse_power_disable();
			fuse_cmd_sense();
			fuse_power_enable();
		}
	}

	/* Read all data into the chip options */
	tegra_fuse_writel(0x1, FUSE_PRIV2INTFC);
	udelay(1);
	tegra_fuse_writel(0, FUSE_PRIV2INTFC);

	while (!(tegra_fuse_readl(FUSE_CTRL) & (1 << 30)));

	fuse_reg_hide();
	fuse_power_disable();
}

static int fuse_set(enum fuse_io_param io_param, u8 *param, int size)
{
	int i, base = fuse_info_tbl[io_param].start_off;
	int len = 0, shift = fuse_info_tbl[io_param].start_bit;
	u8 *data, *dst;
	u32 val, low_mask = fuse_info_tbl[io_param].low_mask;
	u32 high_mask = fuse_info_tbl[io_param].high_mask;

	if (io_param > MAX_PARAMS)
		return -EINVAL;

	data = (u8*)kzalloc(size, GFP_KERNEL);
	if (!data) {
		pr_err("failed to alloc %d bytes\n", size);
		return -ENOMEM;
	}
	dst = data;

	len = fuse_info_tbl[io_param].end_off - base + 1;

	/* read the actual fuse */
	for (i = 0; i < len; i++) {
		val = fuse_cmd_read(base);

		if (len == 1) {
			val &= low_mask;
			val >>= shift;
			break;
		}

		val >>= shift;
		val |= ((fuse_cmd_read(base + 2) & high_mask) << (31 - shift + 1));
		*((u32 *)dst) = val;
		base += 2;
		dst += 4;
	}

	if (len == 1) {
		size = DIV_ROUND_UP(size, 8);
		memcpy(dst, &val, size);
	}

	if (io_param == SEC_BOOT_DEV_CFG)
		*((u16*)data) = be16_to_cpu(*((u16*)data));

	for (i = 0; i < size; i++) {
		if ((*(data + i) | *(param + i)) != *(param + i)) {
			pr_info("actual: 0x%x, local: 0x%x, final: 0x%x\n",
			  *(data + i), *(param + i),
			  (*(data + i) | *(param + i)));
			*(param + i) = (*(data + i) | *(param + i));
		}
	}
	kfree(data);
	return 0;
}

int tegra_fuse_program(struct fuse_data *pgm_data, u32 flags)
{
	u32 reg;
	int i = 0;

	mutex_lock(&fuse_lock);
	reg = tegra_fuse_readl(FUSE_DIS_PGM);
	mutex_unlock(&fuse_lock);
	if (reg) {
		pr_err("fuse programming disabled");
		return -EACCES;
	}

	if (fuse_odm_prod_mode() && (flags != FLAGS_ODMRSVD)) {
		pr_err("reserved odm fuses are allowed in secure mode");
		return -EPERM;
	}

	if ((flags & FLAGS_ODM_PROD_MODE) &&
		(flags & (FLAGS_SBK | FLAGS_DEVKEY))) {
		pr_err("odm production mode and sbk/devkey not allowed");
		return -EPERM;
	}

	mutex_lock(&fuse_lock);
	memcpy(&fuse_info, pgm_data, sizeof(fuse_info));
	for_each_set_bit(i, (unsigned long *)&flags, MAX_PARAMS) {
		fuse_set((u32)i, fuse_info_tbl[i].addr,
			fuse_info_tbl[i].sz);
	}

	populate_fuse_arrs(&fuse_info, flags);
	fuse_program_array(0);

	/* disable program bit */
	reg = 0;
	set_fuse(MASTER_ENB, &reg);

	memset(&fuse_info, 0, sizeof(fuse_info));
	mutex_unlock(&fuse_lock);

	return 0;
}

int tegra_fuse_verify(void)
{
#if ENABLE_FUSE_BURNING
	u32 reg;
#endif
	u32 *data = fuse_pgm_data, addr, *mask = fuse_pgm_mask;
	int err = 0;

	mutex_lock(&fuse_lock);
	fuse_reg_unhide();
	fuse_cmd_sense();

	for (addr = 0; addr < NFUSES; addr++, data++, mask++) {
		/*
		 * Once Odm production mode fuse word (0th word) is set, we can
		 * not set any other fuse word in the fuse cell, including the
		 * redundent fuse word (1st word).
		 * Hence it makes no sense to verify it.
		 */
		if ((addr == 1) && (fuse_odm_prod_mode()))
			continue;
#if ENABLE_FUSE_BURNING
		reg = fuse_cmd_read(addr) & *mask;
		if (reg != (*data & *mask)) {
			pr_err("changing 0x%x to 0x%x is not allowed\n",
				fuse_cmd_read(addr), *data);
#else
		if (*data != (*data & *mask)) {
#endif
			err = -EINVAL;
			break;
		}
	}

	fuse_reg_hide();
	mutex_unlock(&fuse_lock);
	return err;
}

void tegra_fuse_program_disable(void)
{
	mutex_lock(&fuse_lock);
	tegra_fuse_writel(0x1, FUSE_DIS_PGM);
	mutex_unlock(&fuse_lock);
}

static int __init tegra_fuse_program_init(void)
{
	mutex_init(&fuse_lock);
	return 0;
}

module_init(tegra_fuse_program_init);
