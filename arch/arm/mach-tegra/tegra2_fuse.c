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

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include <mach/tegra2_fuse.h>

#include "fuse.h"

#define NFUSES	64
#define STATE_IDLE	0x4

/* since fuse burning is irreversible, use this for testing */
#define ENABLE_FUSE_BURNING 0

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
	int end_bit;
};

static struct param_info fuse_info_tbl[] = {
	[DEVKEY] = {
		.addr = fuse_info.devkey,
		.sz = DEVKEY_SZ,
		.start_off = 0x12,
		.start_bit = 8,
		.end_off = 0x14,
		.end_bit = 7,
	},
	[JTAG_DIS] = {
		.addr = &fuse_info.jtag_dis,
		.sz = JTAG_DIS_SZ,
		.start_off = 0x0,
		.start_bit = 24,
		.end_off = 0x0,
		.end_bit = 24,
	},
	[ODM_PROD_MODE] = {
		.addr = &fuse_info.odm_prod_mode,
		.sz = ODM_PROD_MODE_SZ,
		.start_off = 0x0,
		.start_bit = 23,
		.end_off = 0x0,
		.end_bit = 23,
	},
	[SEC_BOOT_DEV_CFG] = {
		.addr = fuse_info.bootdev_cfg,
		.sz = SEC_BOOT_DEV_CFG_SZ,
		.start_off = 0x14,
		.start_bit = 8,
		.end_off = 0x14,
		.end_bit = 23,
	},
	[SEC_BOOT_DEV_SEL] = {
		.addr = &fuse_info.bootdev_sel,
		.sz = SEC_BOOT_DEV_SEL_SZ,
		.start_off = 0x14,
		.start_bit = 24,
		.end_off = 0x14,
		.end_bit = 26,
	},
	[SBK] = {
		.addr = fuse_info.sbk,
		.sz = SBK_SZ,
		.start_off = 0x0A,
		.start_bit = 8,
		.end_off = 0x12,
		.end_bit = 7,
	},
	[SW_RSVD] = {
		.addr = &fuse_info.sw_rsvd,
		.sz = SW_RSVD_SZ,
		.start_off = 0x14,
		.start_bit = 28,
		.end_off = 0x14,
		.end_bit = 31,
	},
	[IGNORE_DEV_SEL_STRAPS] = {
		.addr = &fuse_info.ignore_devsel_straps,
		.sz = DEV_SEL_STRAPS_SZ,
		.start_off = 0x14,
		.start_bit = 27,
		.end_off = 0x14,
		.end_bit = 27,
	},
	[ODM_RSVD] = {
		.addr = &fuse_info.odm_rsvd,
		.sz = ODM_RSVD_SZ,
		.start_off = 0x16,
		.start_bit = 4,
		.end_off = 0x26,
		.end_bit = 3,
	},
	[MASTER_ENB] = {
		.addr = &master_enable,
		.sz = sizeof(bool),
		.start_off = 0x0,
		.start_bit = 0,
		.end_off = 0x0,
		.end_bit = 0,
	},
};

static void wait_for_idle(void)
{
	u32 reg;

	do {
		reg = tegra_fuse_readl(FUSE_CTRL);
	} while (((reg & (0xF << 16)) >> 16) != STATE_IDLE);
}

static u32 fuse_cmd_read(u32 addr)
{
	u32 reg;

	tegra_fuse_writel(addr, FUSE_REG_ADDR);
	reg = tegra_fuse_readl(FUSE_CTRL);
	reg &= ~0x3;
	reg |= 0x1; /* read */
	tegra_fuse_writel(reg, FUSE_CTRL);
	wait_for_idle();

	reg = tegra_fuse_readl(FUSE_REG_READ);
	return reg;
}

static void fuse_cmd_write(u32 addr, u32 data)
{
	u32 reg;

	tegra_fuse_writel(addr, FUSE_REG_ADDR);
	tegra_fuse_writel(data, FUSE_REG_WRITE);

	reg = tegra_fuse_readl(FUSE_CTRL);
	reg &= ~0x3;
	reg |= 0x2; /* write */
	tegra_fuse_writel(reg, FUSE_CTRL);
	wait_for_idle();
}

static void fuse_cmd_sense(void)
{
	u32 reg;

	reg = tegra_fuse_readl(FUSE_CTRL);
	reg &= ~0x3;
	reg |= 0x3; /* sense */
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

static void get_fuse(int io_param)
{
	int start = fuse_info_tbl[io_param].start_off;
	int end = fuse_info_tbl[io_param].end_off;
	int start_bit = fuse_info_tbl[io_param].start_bit;
	int end_bit = fuse_info_tbl[io_param].end_bit;
	void *fuse = fuse_info_tbl[io_param].addr;
	u32 *dst;
	int i;
	u32 tmp[NFUSES], val, mask;

	for (i = 0; i < NFUSES; i++)
		tmp[i] = fuse_cmd_read(i);

	if (end == start) {
		val = tmp[start];
		val <<= (31 - end_bit);
		val >>= (31 - end_bit - start_bit);
		if ((end_bit - start_bit) <= 8)
			*((u8 *)fuse) = (u8)val;
		else if ((end_bit - start_bit) <= 16)
			*((u16 *)fuse) = (u16)val;
	} else {
		mask = (0xFFFFFFFF >> (31 - start_bit))
			<< (31 - start_bit);
		dst = (u32 *)fuse;
		for (i = start; i <= end; i += 2) {
			val = tmp[i];
			val >>= start_bit;
			val |= (tmp[i + 2] & mask);
			*dst = val;
			dst++;
		}
	}
}

int tegra_fuse_read(u32 io_param_type, void *data, int size)
{
	int ret = 0;

	if (!data)
		return -EINVAL;

	if (size < fuse_info_tbl[io_param_type].sz)
		return -ENOMEM;

	mutex_lock(&fuse_lock);
	fuse_reg_unhide();
	fuse_cmd_sense();

	if (io_param_type == SBK_DEVKEY_STATUS) {
		get_fuse(SBK);
		get_fuse(DEVKEY);
		if (fuse_info.sbk || fuse_info.devkey)
			*(u8 *)data = 1;
		else
			*(u8 *)data = 0;
	} else {
		get_fuse(io_param_type);
		memcpy(data, fuse_info_tbl[io_param_type].addr, size);
	}

	fuse_reg_hide();
	mutex_unlock(&fuse_lock);
	return ret;
}

static void set_fuse(int io_param, u32 *data)
{
	int start = fuse_info_tbl[io_param].start_off;
	int end = fuse_info_tbl[io_param].end_off;
	int start_bit = fuse_info_tbl[io_param].start_bit;
	int end_bit = fuse_info_tbl[io_param].end_bit;
	u32 mask, val, overflow = 0;
	int i, dlen = fuse_info_tbl[io_param].sz / sizeof(u32);

	if (end - start == 0) {
		mask = 0xFFFFFFFF;
		for (i = 0; i < start_bit; i++)
			mask &= ~(1 << i);
		for (i = end_bit + 1; i < 32; i++)
			mask &= ~(1 << i);

		val = (*data & (mask >> end_bit));
		fuse_pgm_data[start] &= ~mask;
		fuse_pgm_data[start] |= (val << start_bit);
		fuse_pgm_data[start + 1] &= ~mask;
		fuse_pgm_data[start + 1] |= (val << start_bit);
	} else {
		mask = 0xFFFFFFFF << (31 - end_bit);
		overflow = data[dlen - 1] & mask;
		overflow >>= (31 - end_bit);
		data[dlen - 1] <<= start_bit;
		for (i = dlen - 1; i >= 0; i--) {
			data[i] |= (data[i - 1] & mask) >>
			  (31 - end_bit);
			data[i - 1] <<= start_bit;
		}

		for (i = start; i <= end; i += 2) {
			mask = 0xFFFFFFFF;

			if (i == end) {
				mask >>= (31 - end_bit);
				val = overflow;
			} else
				val = (*data & mask);

			if (i == start)
				mask <<= start_bit;

			fuse_pgm_mask[i] = mask;
			fuse_pgm_mask[i + 1] = mask;
			fuse_pgm_data[i] &= ~mask;
			fuse_pgm_data[i] |= val;
			fuse_pgm_data[i + 1] &= ~mask;
			fuse_pgm_data[i + 1] |= val;
			data++;
		}
	}
}

static void populate_fuse_arrs(struct fuse_data *info, u32 flags)
{
	u32 *src, data = 0, mask_data;
	int offset;

	memset(fuse_pgm_data, 0, ARRAY_SIZE(fuse_pgm_data));
	memset(fuse_pgm_mask, 0, ARRAY_SIZE(fuse_pgm_mask));

	/* enable program bit */
	data = 1;
	set_fuse(MASTER_ENB, &data);

	if (flags & FLAGS_ODMRSVD) {
		src = (u32*)info->odm_rsvd;
		set_fuse(ODM_RSVD, src);
	}

	/* do not burn any more if secure mode is set */
	if (fuse_odm_prod_mode())
		goto out;

	if (flags & FLAGS_JTAG_DIS) {
		data = (u32)info->jtag_dis;
		set_fuse(JTAG_DIS, &data);
	}

	if (flags & FLAGS_ODM_PROD_MODE) {
		data = (u32)info->odm_prod_mode;
		set_fuse(ODM_PROD_MODE, &data);
	}

	if (flags & FLAGS_SBK) {
		src = (u32*)info->sbk;
		set_fuse(SBK, src);
	}

	if (flags & FLAGS_DEVKEY) {
		src = (u32*)info->devkey;
		set_fuse(DEVKEY, src);
	}

	if (flags & FLAGS_SEC_BOOT_DEV_CFG) {
		data = *(u32*)info->bootdev_cfg  & 0x3FFF;
		set_fuse(SEC_BOOT_DEV_CFG, &data);
	}

	if (flags & (FLAGS_SEC_BOOT_DEV_SEL |
		 FLAGS_IGNORE_DEV_SEL_STRAPS | FLAGS_SW_RSVD)) {
		data = 0;
		mask_data = 0;

		if (flags & FLAGS_SEC_BOOT_DEV_SEL) {
			data |= (info->bootdev_sel & 0x7);
			mask_data |= 0x7;
		}

		if (flags & FLAGS_IGNORE_DEV_SEL_STRAPS) {
			data |= ((info->ignore_devsel_straps & 0x1) << 3);
			mask_data |= (0x1 << 3);
		}

		if (flags & FLAGS_SW_RSVD) {
			data |= ((info->sw_rsvd & 0xF) << 4);
			mask_data |= (0xF << 4);
		}

		set_fuse(SW_RSVD, &data);
		offset = fuse_info_tbl[SW_RSVD].start_off;
		fuse_pgm_mask[offset] = mask_data;
		fuse_pgm_mask[offset + 1] = mask_data;
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
	fuse_cmd_write(0, fuse_val[0]);
	fuse_cmd_write(1, fuse_val[1]);

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
			fuse_cmd_write(addr, tmp_fuse_pgm_data[addr]);

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

static int fuse_set(u32 io_param_type, u8 *param, int size)
{
	int i, base = fuse_info_tbl[io_param_type].start_off;
	int len = 0, shift = fuse_info_tbl[io_param_type].start_bit;
	u8 *data, *dst;
	u32 val;

	if (io_param_type > MAX_IO_PARAMS)
		return -EINVAL;

	data = (u8*)kzalloc(size, GFP_KERNEL);
	if (!data) {
		pr_err("failed to alloc %d bytes\n", size);
		return -ENOMEM;
	}
	dst = data;

	len = fuse_info_tbl[io_param_type].end_off - base + 1;

	/* read the actual fuse */
	for (i = 0; i < len; i++) {
		val = fuse_cmd_read(base);
		val >>= shift;

		if (len == 1) {
			val &= (0xFFFFFFFF >>
			(31 - fuse_info_tbl[io_param_type].end_bit));
			break;
		}

		val |= fuse_cmd_read(base + 2) & (0xFFFFFFFF << (31 - shift));
		*((u32 *)dst) = val;
		base += 2;
		dst += 4;
	}

	if (size == sizeof(u8))
		*dst = (u8)val;
	else if (size == sizeof(u16))
		*((u16*)dst) = (u16)val;

	if (io_param_type == SEC_BOOT_DEV_CFG)
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
	for_each_set_bit(i, (const long unsigned int *)&flags, MAX_IO_PARAMS) {
		fuse_set((u32)(i + 1), fuse_info_tbl[i + 1].addr,
			fuse_info_tbl[i + 1].sz);
	}

	populate_fuse_arrs(&fuse_info, flags);
	fuse_program_array(0);
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
