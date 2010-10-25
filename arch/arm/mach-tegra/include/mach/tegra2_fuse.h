/*
 * arch/arm/mach-tegra/include/mach/tegra2_fuse.h
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

#ifndef __MACH_TEGRA2_FUSE_H
#define __MACH_TEGRA2_FUSE_H

/* fuse io parameters */
enum {
	DEVKEY = 1,
	JTAG_DIS,
	SBK_DEVKEY_STATUS,
	/*
	 * Programming the odm production fuse at the same
	 * time as the sbk or dev_key is not allowed as it is not possible to
	 * verify that the sbk or dev_key were programmed correctly.
	 */
	ODM_PROD_MODE,
	SEC_BOOT_DEV_CFG,
	SEC_BOOT_DEV_SEL,
	SBK,
	SKU,
	SW_RSVD,
	IGNORE_DEV_SEL_STRAPS,
	ODM_RSVD,
	MASTER_ENB,
	_PARAMS_MAX,
	_PARAMS_U32 = 0x7FFFFFFF
};

#define MAX_IO_PARAMS (_PARAMS_MAX - 1)

/* io param sizes */
#define DEVKEY_SZ		sizeof(u32)
#define JTAG_DIS_SZ		sizeof(u8)
#define SBK_DEVKEY_STATUS_SZ	sizeof(u8)
#define ODM_PROD_MODE_SZ	sizeof(u8)
#define SEC_BOOT_DEV_CFG_SZ	sizeof(u16)
#define SEC_BOOT_DEV_SEL_SZ	sizeof(u32)
#define SEC_BOOT_DEV_RAW_SEL_SZ	sizeof(u32)
#define SBK_SZ	SZ_16
#define SKU_SZ	sizeof(32)
#define SW_RSVD_SZ	sizeof(u8)
#define DEV_SEL_STRAPS_SZ		sizeof(u8)
#define ODM_RSVD_SZ		32

struct fuse_data {
	u8 sbk[SBK_SZ];
	u8 devkey[DEVKEY_SZ];
	u8 jtag_dis;
	u8 bootdev_sel;
	u8 bootdev_cfg[SEC_BOOT_DEV_CFG_SZ];
	u8 sw_rsvd;
	u8 odm_prod_mode;
	u8 ignore_devsel_straps;
	u8 odm_rsvd[ODM_RSVD_SZ];
};

/* secondary boot device options */
enum {
	SECBOOTDEV_SDMMC,
	SECBOOTDEV_NOR,
	SECBOOTDEV_SPI,
	SECBOOTDEV_NAND,
	SECBOOTDEV_LBANAND,
	SECBOOTDEV_MUXONENAND,
	_SECBOOTDEV_MAX,
	_SECBOOTDEV_U32 = 0x7FFFFFFF
};

#define NSEC_BOOT_DEV (_SECBOOTDEV_MAX - 1)

/*
 * read the fuse settings
 * @param: io_param_type - param type enum
 * @param: size - read size in bytes
 */
int tegra_fuse_read(u32 io_param_type, void *data, int size);

#define FLAGS_DEVKEY			BIT(0)
#define FLAGS_JTAG_DIS			BIT(1)
#define FLAGS_SBK_DEVKEY_STATUS	BIT(2)
#define FLAGS_ODM_PROD_MODE		BIT(3)
#define FLAGS_SEC_BOOT_DEV_CFG	BIT(4)
#define FLAGS_SEC_BOOT_DEV_SEL	BIT(5)
#define FLAGS_SBK			BIT(6)
#define FLAGS_SKU			BIT(7)
#define FLAGS_SW_RSVD		BIT(8)
#define FLAGS_IGNORE_DEV_SEL_STRAPS	BIT(9)
#define FLAGS_SEC_BOOT_DEV_RAW_SEL	BIT(10)
#define FLAGS_ODMRSVD			BIT(11)

/*
 * Prior to invoking this routine, the caller is responsible for supplying
 * valid fuse programming voltage.
 *
 * @param: pgm_data - entire data to be programmed
 * @flags: program flags (e.g. FLAGS_DEVKEY)
 */
int tegra_fuse_program(struct fuse_data *pgm_data, u32 flags);

/* Disables the fuse programming until the next system reset */
void tegra_fuse_program_disable(void);

/* Ensure that fuse programming voltage is removed before using this api */
int tegra_fuse_verify(void);

#endif
