 /*
 * arch/arm/mach-tegra/fuse.c
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@android.com>
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

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/delay.h>

#ifdef CONFIG_TEGRA_SYSTEM_DMA
#include <linux/dma-mapping.h>
#include <linux/spinlock.h>
#include <linux/completion.h>
#include <linux/sched.h>
#include <mach/dma.h>
#endif

#include <mach/iomap.h>
#include <mach/fuse.h>

#define NFUSES	64
#define STATE_IDLE	0x4

/* since fuse burning is irreversible, use this for testing */
#define ENABLE_FUSE_BURNING 1

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
#define FUSE_UID_LOW		0x108
#define FUSE_UID_HIGH		0x10C
#define FUSE_SKU_INFO		0x110
#define FUSE_SECURITY_MODE	0x1A0
#define FUSE_ARM_DEBUG_DIS	0x1B8
#define FUSE_BOOT_DEV_INFO	0x1BC
#define FUSE_RSVD_SW		0x1C0
#define FUSE_RSVD_ODM_BASE	0x1C8
#define FUSE_SPARE_BIT		0x200

#define FLAGS_PROGRAM_SBK	0x001
#define FLAGS_PROGRAM_DEV_KEY	0x002
#define FLAGS_PROGRAM_JTAG_DIS	0x004
#define FLAGS_PROGRAM_BOOT_DEV_SEL	0x008
#define FLAGS_PROGRAM_BOOT_DEV_CFG	0x010
#define FLAGS_PROGRAM_SW_RSVD	0x020
#define FLAGS_PROGRAM_ODM_PROD_MODE	0x040
#define FLAGS_PROGRAM_SPARE_BITS	0x080
#define FLAGS_PROGRAM_IGNORE_DEV_SEL	0x100
#define FLAGS_PROGRAM_ODM_RSVD	0x200
#define FLAGS_PROGRAM_ILLEGAL	0x80000000

/* fuse offsets */
#define CONTROL_FUSE	0x0
#define SKU_FUSE		0x6
#define PRIVATE_KEY0_FUSE	0xA
#define PRIVATE_KEY1_FUSE	0xC
#define PRIVATE_KEY2_FUSE	0xE
#define PRIVATE_KEY3_FUSE	0x10
#define PRIVATE_KEY4_FUSE	0x12
#define PRIVATE_KEY5_FUSE	0x14
#define BOOT_DEVICE_INFO_FUSE	0x14
#define RSVD_ODM0		0x16
#define RSVD_ODM8		0x26

static u32 fuse_pgm_data[NFUSES];
static u32 fuse_pgm_mask[NFUSES];
static u32 tmp_fuse_pgm_data[NFUSES];

DEFINE_MUTEX(fuse_lock);

static u32 pgm_flags;
static u32 odm_rsvd_flag;

struct fuse_data {
	u8 sbk[SBK_SZ];
	u8 devkey[DEVKEY_SZ];
	bool jtag_dis;
	u8 bootdev_sel[SEC_BOOT_DEV_SEL_SZ];
	u8 bootdev_sel_raw[SEC_BOOT_DEV_RAW_SEL_SZ];
	u8 bootdev_cfg[SEC_BOOT_DEV_CFG_SZ];
	u8 sw_rsvd[SW_RSVD_SZ];
	bool odm_prod_mode;
	u8 ignore_devsel_straps[DEV_SEL_STRAPS_SZ];
	u8 odm_rsvd_state[ODM_RSVD_SZ];
};

static struct fuse_data fuse_info;

struct param_info {
	void *param;
	int param_sz;
	u32 pgm_flag;
};

static struct param_info param_info_tbl[] = {
	[DEVKEY] = {
		fuse_info.devkey,
		DEVKEY_SZ,
		FLAGS_PROGRAM_DEV_KEY
		},
	[JTAG_DIS] = {
		&fuse_info.jtag_dis,
		JTAG_DIS_SZ,
		FLAGS_PROGRAM_JTAG_DIS
		},
	[ODM_PROD_MODE] = {
		&fuse_info.odm_prod_mode,
		ODM_PROD_MODE_SZ,
		FLAGS_PROGRAM_ODM_PROD_MODE
		},
	[SEC_BOOT_DEV_CFG] = {
		fuse_info.bootdev_cfg,
		SEC_BOOT_DEV_CFG_SZ,
		FLAGS_PROGRAM_BOOT_DEV_CFG
		},
	[SEC_BOOT_DEV_SEL] = {
		fuse_info.bootdev_sel,
		SEC_BOOT_DEV_SEL_SZ,
		FLAGS_PROGRAM_BOOT_DEV_SEL
		},
	[SBK] = {
		fuse_info.sbk,
		SBK_SZ,
		FLAGS_PROGRAM_SBK
		},
	[SW_RSVD] = {
		&fuse_info.sw_rsvd,
		SW_RSVD_SZ,
		FLAGS_PROGRAM_SW_RSVD
		},
	[IGNORE_DEV_SEL_STRAPS] = {
		&fuse_info.ignore_devsel_straps,
		DEV_SEL_STRAPS_SZ,
		FLAGS_PROGRAM_IGNORE_DEV_SEL
		},
	[SEC_BOOT_DEV_RAW_SEL] = {
		&fuse_info.bootdev_sel_raw,
		SEC_BOOT_DEV_SEL_SZ,
		FLAGS_PROGRAM_BOOT_DEV_SEL
		},
	[ODM_RSVD] = {
		&fuse_info.odm_rsvd_state,
		ODM_RSVD_SZ,
		FLAGS_PROGRAM_ODM_RSVD
		},
};

#ifdef CONFIG_TEGRA_SYSTEM_DMA
struct tegra_dma_channel *dma;
u32 *fuse_bb;
dma_addr_t fuse_bb_phys;
struct completion rd_wait;
struct completion wr_wait;

static void fuse_dma_complete(struct tegra_dma_req *req)
{
	if (req) {
		if (req->to_memory)
			complete(&rd_wait);
		else
			complete(&wr_wait);
	}
}

static inline u32 fuse_readl(unsigned long offset)
{
	struct tegra_dma_req req;

	req.complete = fuse_dma_complete;
	req.to_memory = 1;
	req.dest_addr = fuse_bb_phys;
	req.dest_bus_width = 32;
	req.dest_wrap = 1;
	req.source_addr = TEGRA_FUSE_BASE + offset;
	req.source_bus_width = 32;
	req.source_wrap = 4;
	req.req_sel = 0;
	req.size = 4;

	init_completion(&rd_wait);
	tegra_dma_enqueue_req(dma, &req);
	if (wait_for_completion_timeout(&rd_wait, msecs_to_jiffies(50)) == 0) {
		WARN_ON(1);
		return 0;
	}

	return *((u32 *)fuse_bb);
}

static inline void fuse_writel(u32 value, unsigned long offset)
{
	struct tegra_dma_req req;

	*((u32 *)fuse_bb) = value;
	req.complete = fuse_dma_complete;
	req.to_memory = 0;
	req.dest_addr = TEGRA_FUSE_BASE + offset;
	req.dest_wrap = 4;
	req.dest_bus_width = 32;
	req.source_addr = fuse_bb_phys;
	req.source_bus_width = 32;
	req.source_wrap = 1;
	req.req_sel = 0;
	req.size = 4;

	init_completion(&wr_wait);
	tegra_dma_enqueue_req(dma, &req);
	if (wait_for_completion_timeout(&wr_wait, msecs_to_jiffies(50)) == 0)
		WARN_ON(1);
}
#else
static inline u32 fuse_readl(unsigned long offset)
{
	return readl(IO_TO_VIRT(TEGRA_FUSE_BASE + offset));
}

static inline void fuse_writel(u32 value, unsigned long offset)
{
	writel(value, IO_TO_VIRT(TEGRA_FUSE_BASE + offset));
}
#endif

static inline void fuse_read_bytes(unsigned long offset, u8 *data, int nbytes)
{
	u32 *tmp = (u32 *)data;
	int nwords = (nbytes/sizeof(u32));

	while (nwords-- > 0) {
		*tmp = fuse_readl(offset);
		offset += 4; tmp++;
	}
}

static void wait_for_idle(void)
{
	u32 reg;

	do {
		reg = fuse_readl(FUSE_CTRL);
	} while (((reg & (0xF << 16)) >> 16) != STATE_IDLE);
}

#define FUSE_CMD_READ  0x01
#define FUSE_CMD_WRITE 0x02
#define FUSE_CMD_SENSE 0x03

static u32 fuse_send_cmd(u32 addr, u32 data, int cmd)
{
	u32 reg;

	if (cmd > FUSE_CMD_SENSE)
		return -EINVAL;

	if ((cmd == FUSE_CMD_READ) | (cmd == FUSE_CMD_WRITE))
		fuse_writel(addr, FUSE_REG_ADDR);

	if (cmd == FUSE_CMD_WRITE)
		fuse_writel(data, FUSE_REG_WRITE);

	reg = fuse_readl(FUSE_CTRL);
	reg &= ~FUSE_CMD_SENSE;
	reg |= cmd;
	fuse_writel(reg, FUSE_CTRL);
	wait_for_idle();

	if (cmd == FUSE_CMD_READ) {
		reg = fuse_readl(FUSE_REG_READ);
		return reg;
	}

	return 0;
}

static void tegra_fuse_clear(void)
{
	memset(&fuse_info, 0, sizeof(fuse_info));
}

static void fuse_reg_hide(void)
{
	u32 reg = readl(IO_TO_VIRT(TEGRA_CLK_RESET_BASE + 0x48));
	reg &= ~(1 << 28);
	writel(reg, IO_TO_VIRT(TEGRA_CLK_RESET_BASE + 0x48));
}

static void fuse_reg_unhide(void)
{
	u32 reg = readl(IO_TO_VIRT(TEGRA_CLK_RESET_BASE + 0x48));
	reg |= (1 << 28);
	writel(reg, IO_TO_VIRT(TEGRA_CLK_RESET_BASE + 0x48));
}

static void tegra_fuse_sense(void)
{
	fuse_send_cmd(0, 0, FUSE_CMD_SENSE);
}

static bool fuse_odm_prod_mode(void)
{
	return (fuse_readl(FUSE_SECURITY_MODE) ? true : false);
}

void tegra_init_fuse(void)
{
	u32 reg = readl(IO_TO_VIRT(TEGRA_CLK_RESET_BASE + 0x48));
	reg |= 1 << 28;
	writel(reg, IO_TO_VIRT(TEGRA_CLK_RESET_BASE + 0x48));

#ifdef CONFIG_TEGRA_SYSTEM_DMA
	dma = tegra_dma_allocate_channel(TEGRA_DMA_MODE_ONESHOT);
	if (!dma) {
		pr_err("%s: can not allocate dma channel\n", __func__);
		return;
	}

	fuse_bb = dma_alloc_coherent(NULL, sizeof(u32),
		&fuse_bb_phys, GFP_KERNEL);
	if (!fuse_bb) {
		pr_err("%s: can not allocate bounce buffer\n", __func__);
		return;
	}
#endif

	pr_info("Tegra SKU: %d CPU Process: %d Core Process: %d\n",
		tegra_sku_id(), tegra_cpu_process_id(),
		tegra_core_process_id());

	mutex_init(&fuse_lock);
}

void tegra_deinit_fuse(void)
{
#ifdef CONFIG_TEGRA_SYSTEM_DMA
	dma_free_coherent(NULL, sizeof(u32), fuse_bb, fuse_bb_phys);
	tegra_dma_free_channel(dma);
#endif
	mutex_destroy(&fuse_lock);
}

unsigned long long tegra_chip_uid(void)
{
	unsigned long long lo, hi;

	lo = fuse_readl(FUSE_UID_LOW);
	hi = fuse_readl(FUSE_UID_HIGH);
	return (hi << 32ull) | lo;
}

int tegra_sku_id(void)
{
	int sku_id;
	u32 reg = fuse_readl(FUSE_SKU_INFO);
	sku_id = reg & 0xFF;
	return sku_id;
}

int tegra_cpu_process_id(void)
{
	int cpu_process_id;
	u32 reg = fuse_readl(FUSE_SPARE_BIT);
	cpu_process_id = (reg >> 6) & 3;
	return cpu_process_id;
}

int tegra_core_process_id(void)
{
	int core_process_id;
	u32 reg = fuse_readl(FUSE_SPARE_BIT);
	core_process_id = (reg >> 12) & 3;
	return core_process_id;
}

static int fuse_read(u32 io_param_type, void *data, int size)
{
	u32 reg, i;
	int ret = 0;

	if (size < param_info_tbl[io_param_type].param_sz) {
		return -ENOMEM;
	}

	switch (io_param_type) {
	case DEVKEY:
		reg = fuse_send_cmd(PRIVATE_KEY4_FUSE, 0, FUSE_CMD_READ);
		reg >>= 8;
		reg |= fuse_send_cmd(PRIVATE_KEY5_FUSE, 0, FUSE_CMD_READ) << 24;
		*((u32*)data) = be32_to_cpu(reg);
		break;
	case JTAG_DIS:
		reg = fuse_send_cmd(CONTROL_FUSE, 0, FUSE_CMD_READ);
		*((bool*)data) = (reg & (1<<24)) ? true : false;
		break;
	case SBK_DEVKEY_STATUS:
		reg = fuse_send_cmd(PRIVATE_KEY0_FUSE, 0, FUSE_CMD_READ) & ~0xFF;
		reg |= fuse_send_cmd(PRIVATE_KEY1_FUSE, 0, FUSE_CMD_READ);
		reg |= fuse_send_cmd(PRIVATE_KEY2_FUSE, 0, FUSE_CMD_READ);
		reg |= fuse_send_cmd(PRIVATE_KEY3_FUSE, 0, FUSE_CMD_READ);
		reg |= fuse_send_cmd(PRIVATE_KEY4_FUSE, 0, FUSE_CMD_READ);
		reg |= fuse_send_cmd(PRIVATE_KEY5_FUSE, 0, FUSE_CMD_READ) << 24;
		*((bool*)data) = reg ? true : false;
		break;
	case ODM_PROD_MODE:
		reg = fuse_send_cmd(CONTROL_FUSE, 0, FUSE_CMD_READ);
		*((bool*)data) = reg & (1<<23);
		break;
	case SEC_BOOT_DEV_CFG:
		reg = fuse_send_cmd(BOOT_DEVICE_INFO_FUSE, 0, FUSE_CMD_READ);
		reg &= 0x3FFF;
		*((u16*)data) = be16_to_cpu(reg);
		break;
	case SEC_BOOT_DEV_SEL:
		reg = fuse_send_cmd(BOOT_DEVICE_INFO_FUSE, 0, FUSE_CMD_READ);
		reg &= 0x7;
		*((u32*)data) = (reg > NSEC_BOOT_DEV) ? SECBOOTDEV_SDMMC : reg;
		break;
	case SEC_BOOT_DEV_RAW_SEL:
		reg = fuse_send_cmd(BOOT_DEVICE_INFO_FUSE, 0, FUSE_CMD_READ);
		reg &= 0x7;
		*((u32*)data) = reg;
		break;
	case SBK:
		for (i = 0; i < (SBK_SZ/4); i++)
			*((u32*)data + i) = fuse_send_cmd(PRIVATE_KEY0_FUSE
			  + (i * 2), 0, FUSE_CMD_READ);
		for (i = 0; i < (SBK_SZ/4) - 1; i++) {
			*((u32*)data + i) >>= 8;
			*((u32*)data + i) |= *((u32*)data + i + 1) << 24;
		}
		*((u32*)data + i) |= fuse_send_cmd(PRIVATE_KEY4_FUSE,
		  0, FUSE_CMD_READ) << 24;
		break;
	case SW_RSVD:
		*((u32*)data) = fuse_send_cmd(BOOT_DEVICE_INFO_FUSE, 0,
			FUSE_CMD_READ);
		break;
	case IGNORE_DEV_SEL_STRAPS:
		reg = fuse_send_cmd(BOOT_DEVICE_INFO_FUSE, 0, FUSE_CMD_READ);
		*((u32*)data) = reg & 0x8;
		break;
	case ODM_RSVD:
		for (i = 0; i < (ODM_RSVD_SZ/4); i++)
			*((u32*)data + i) = fuse_send_cmd(RSVD_ODM0 + (i * 2),
			  0, FUSE_CMD_READ);
		for (i = 0; i < (ODM_RSVD_SZ/4) - 1; i++) {
			*((u32*)data + i) >>= 4;
			*((u32*)data + i) |= *((u32*)data + i + 1) << 28;
		}
		*((u32*)data + i) |= fuse_send_cmd(RSVD_ODM8,
			0, FUSE_CMD_READ) << 28;
		break;
	default:
		pr_err("invalid param size");
		ret = -EINVAL;
	}

	return ret;
}

int tegra_fuse_read(u32 io_param_type, void *data, int size)
{
	int ret;

	if (!data)
		return -EINVAL;

	mutex_lock(&fuse_lock);
	fuse_reg_unhide();
	tegra_fuse_sense();
	ret = fuse_read(io_param_type, data, size);
	fuse_reg_hide();
	mutex_unlock(&fuse_lock);
	return ret;
}

static int fuse_set(u32 io_param_type, void *data, u8 *param, int size)
{
	int ret, i;

	if (io_param_type > MAX_IO_PARAMS)
		return -EINVAL;

	ret = fuse_read(io_param_type, param, size);
	if (ret < 0) {
		pr_err("read failed");
		return ret;
	}

	mutex_lock(&fuse_lock);
	if (io_param_type == SEC_BOOT_DEV_CFG)
		*((u16*)param) = be16_to_cpu(*((u16*)param));

	for (i = 0; i < size; i++) {
		if ((param[i] | *((u8*)data+i)) != *((u8*)data+i)) {
			pr_err("%s: 1->0 not allowed", __func__);
			return -EBADRQC;
		}
	}
	memcpy(param, data, size);
	mutex_unlock(&fuse_lock);
	return 0;
}

int tegra_fuse_write(u32 io_param_type, void *data, int size)
{
	u32 reg;
	int ret;

	if (!data)
		return -EINVAL;

	if (size < param_info_tbl[io_param_type].param_sz)
		return -ENOMEM;

	mutex_lock(&fuse_lock);
	reg = fuse_readl(FUSE_DIS_PGM);
	mutex_unlock(&fuse_lock);
	if (reg) {
		pr_err("fuse programming disabled");
		return -EACCES;
	}

	if ((io_param_type == ODM_PROD_MODE) &&
		(pgm_flags & (FLAGS_PROGRAM_SBK | FLAGS_PROGRAM_DEV_KEY))) {
		pr_err("odm production mode and sbk/devkey not allowed");
		pgm_flags |= FLAGS_PROGRAM_ILLEGAL;
		return -EINVAL;
	}

	if (((io_param_type == SBK) || (io_param_type == DEVKEY)) &&
		(pgm_flags & FLAGS_PROGRAM_ODM_PROD_MODE)) {
		pr_err("odm production mode and sbk/devkey not allowed");
		pgm_flags |= FLAGS_PROGRAM_ILLEGAL;
		return -EINVAL;
	}

	ret = fuse_set(io_param_type, data, param_info_tbl[io_param_type].param,
		param_info_tbl[io_param_type].param_sz);

	if (ret == 0)
		pgm_flags |= param_info_tbl[io_param_type].pgm_flag;

	odm_rsvd_flag = (io_param_type == ODM_RSVD) ?: false;

	return ret;
}

#define FIELD_SHIFT(x)	((0?x)%32)
#define FIELD_MASK(x)	(0xFFFFFFFFUL >> (31-((1?x)%32)+((0?x)%32)))
#define FIELD_SHIFTMASK(x)	(FIELD_MASK(x) << (FIELD_SHIFT(x)))

/* p = prefix, c = copy, i = index, d = data */
#define FUSE_BASE(p, c, i) FUSE_##p##_##c##_##i
#define FUSE_DATA(p, c, i) FUSE_##p##_##c##_##i##_DATA
#define FUSE_WIDTH(p, c, i) FUSE_##p##_##c##_##i##_WIDTH

#define SET_FUSE(p, c, i, d) \
	fuse_pgm_data[FUSE_BASE(p, c, i)] = \
	(fuse_pgm_data[FUSE_BASE(p, c, i)] & \
	~FIELD_SHIFTMASK(FUSE_DATA(p, c, i))) | \
	((d & (FIELD_MASK(FUSE_DATA(p, c, i)))) << \
	(FIELD_SHIFT(FUSE_DATA(p, c, i))))

#define SET_MASK(p, c, i) \
	fuse_pgm_mask[FUSE_BASE(p,c,i)] |= FIELD_SHIFTMASK(FUSE_DATA(p,c,i))

#define set_fuse_data(p, c, i, d) \
	do { \
		SET_FUSE(p,c,i,d); \
		SET_MASK(p,c,i); \
	} while (0)

#define set_both_fuses(p, i, d) \
	do { \
		set_fuse_data(p, PRI, i, d); \
		set_fuse_data(p, RED, i, d); \
	} while (0)

/* update fuses with an explicit mask */
#define set_both_fuses_with_mask(p, i, d, md) \
	do { \
		SET_FUSE(p, PRI, i, d); \
		SET_FUSE(p, RED, i, d); \
		fuse_pgm_mask[FUSE_BASE(p, PRI, i)] |= \
		  (md << FIELD_SHIFT(FUSE_DATA(p, PRI, i))); \
		fuse_pgm_mask[FUSE_BASE(p, RED, i)] |= \
		  (md << FIELD_SHIFT(FUSE_DATA(p, RED, i))); \
	} while (0)

#define set_fuse_and_update(p, d) \
	do { \
		set_both_fuses(p, 0, d); \
		(d >>= FUSE_WIDTH(p,PRI,0)); \
		set_both_fuses(p, 1, d); \
	} while (0)

static void populate_fuse_arrs(void)
{
	u32 *src, data = 0, mask_data;

	memset(fuse_pgm_data, 0, ARRAY_SIZE(fuse_pgm_data));
	memset(fuse_pgm_mask, 0, ARRAY_SIZE(fuse_pgm_mask));

	/* enable program bit */
	set_both_fuses(ENABLE_PROGRAM, 0, 1);

	if (pgm_flags & FLAGS_PROGRAM_ODM_RSVD) {
		src = (u32*)fuse_info.odm_rsvd_state;
		set_fuse_and_update(RESERVED_ODM0, *src); src++;
		set_fuse_and_update(RESERVED_ODM1, *src); src++;
		set_fuse_and_update(RESERVED_ODM2, *src); src++;
		set_fuse_and_update(RESERVED_ODM3, *src); src++;
		set_fuse_and_update(RESERVED_ODM4, *src); src++;
		set_fuse_and_update(RESERVED_ODM5, *src); src++;
		set_fuse_and_update(RESERVED_ODM6, *src); src++;
		set_fuse_and_update(RESERVED_ODM7, *src);
	}

	/* do not burn any more if secure mode is set */
	if (fuse_odm_prod_mode())
		goto out;

	if (pgm_flags & FLAGS_PROGRAM_JTAG_DIS) {
		data = (u32)fuse_info.jtag_dis;
		set_both_fuses(ARM_DEBUG_DIS, 0, data);
	}

	if (pgm_flags & FLAGS_PROGRAM_ODM_PROD_MODE) {
		data = (u32)fuse_info.odm_prod_mode;
		set_both_fuses(SECURITY_MODE, 0, data);
	}

	if (pgm_flags & FLAGS_PROGRAM_SBK) {
		src = (u32*)fuse_info.sbk;
		set_fuse_and_update(PRIVATE_KEY0, *src); src++;
		set_fuse_and_update(PRIVATE_KEY1, *src); src++;
		set_fuse_and_update(PRIVATE_KEY2, *src); src++;
		set_fuse_and_update(PRIVATE_KEY3, *src);
	}

	if (pgm_flags & FLAGS_PROGRAM_DEV_KEY) {
		src = (u32*)fuse_info.devkey;
		set_fuse_and_update(PRIVATE_KEY4, *src);
	}

	if (pgm_flags & FLAGS_PROGRAM_BOOT_DEV_CFG) {
		data = *(u32*)fuse_info.bootdev_cfg  & 0x3FFF;
		set_both_fuses(BOOT_DEVICE_INFO, 0, data);
	}

	if (pgm_flags & (FLAGS_PROGRAM_BOOT_DEV_SEL |
		 FLAGS_PROGRAM_IGNORE_DEV_SEL | FLAGS_PROGRAM_SW_RSVD)) {
		data = 0;
		mask_data = 0;

		if (pgm_flags & FLAGS_PROGRAM_BOOT_DEV_SEL) {
			data |= (*(u32*)fuse_info.bootdev_sel_raw & 0x7);
			mask_data |= 0x7;
		}

		if (pgm_flags & FLAGS_PROGRAM_IGNORE_DEV_SEL) {
			data |= ((*(u32*)fuse_info.ignore_devsel_straps & 0x1) << 3);
			mask_data |= (0x1 << 3);
		}

		if (pgm_flags & FLAGS_PROGRAM_SW_RSVD) {
			data |= ((*(u32*)fuse_info.sw_rsvd & 0xF) << 4);
			mask_data |= (0xF <<  4);
		}

		set_both_fuses_with_mask(RESERVED_SW, 0, data, mask_data);
	}

out:
	pr_debug("ready to program");
}

static void fuse_power_enable(void)
{
#if ENABLE_FUSE_BURNING
	fuse_writel(0x1, FUSE_PWR_GOOD_SW);
	udelay(1);
#endif
}

static void fuse_power_disable(void)
{
#if ENABLE_FUSE_BURNING
	fuse_writel(0, FUSE_PWR_GOOD_SW);
	udelay(1);
#endif
}

static void fuse_program_array(int pgm_cycles)
{
	u32 reg, fuse_val[2];
	u32 *data = tmp_fuse_pgm_data, addr = 0, *mask = fuse_pgm_mask;
	int i;

	fuse_reg_unhide();
	tegra_fuse_sense();

	/* get the first 2 fuse bytes */
	fuse_val[0] = fuse_send_cmd(0, 0, FUSE_CMD_READ);
	fuse_val[1] = fuse_send_cmd(1, 0, FUSE_CMD_READ);

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
		fuse_writel(reg, FUSE_TIME_PGM);
	}
	fuse_val[0] = (0x1 & ~fuse_val[0]);
	fuse_val[1] = (0x1 & ~fuse_val[1]);
	fuse_send_cmd(0, fuse_val[0], FUSE_CMD_WRITE);
	fuse_send_cmd(1, fuse_val[1], FUSE_CMD_WRITE);

	fuse_power_disable();

	/*
	 * this will allow programming of other fuses
	 * and the reading of the existing fuse values
	 */
	tegra_fuse_sense();

	/* Clear out all bits that have already been burned or masked out */
	memcpy(data, fuse_pgm_data, NFUSES * sizeof(u32));

	for (i = 0; i < NFUSES; i++, addr++, data++, mask++) {
		reg = fuse_send_cmd(addr, 0, FUSE_CMD_READ);
		pr_debug("%d: 0x%x 0x%x 0x%x\n", i, (u32)(*data), ~reg, (u32)(*mask));
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
		fuse_send_cmd(addr, tmp_fuse_pgm_data[addr], FUSE_CMD_WRITE);

		if (addr < 2) {
			fuse_power_disable();
			tegra_fuse_sense();
			fuse_power_enable();
		}
	}

	/* Read all data into the chip options */
	fuse_writel(0x1, FUSE_PRIV2INTFC);
	udelay(1);
	fuse_writel(0, FUSE_PRIV2INTFC);

	while (!(fuse_readl(FUSE_CTRL) & (1 << 30)));

	fuse_reg_hide();
	fuse_power_disable();
}

int tegra_fuse_program(void)
{
	u32 reg;

	if (fuse_odm_prod_mode() && !odm_rsvd_flag) {
		pr_err("reserved odm fuses are allowed in secure mode");
		return -EPERM;
	}

	mutex_lock(&fuse_lock);
	reg = fuse_readl(FUSE_DIS_PGM);
	mutex_unlock(&fuse_lock);
	if (reg) {
		pr_err("fuse programming disabled");
		return -EACCES;
	}

	mutex_lock(&fuse_lock);
	populate_fuse_arrs();
	fuse_program_array(0);
	tegra_fuse_clear();
	mutex_unlock(&fuse_lock);

	odm_rsvd_flag = false;
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
	tegra_fuse_sense();

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
		reg = fuse_send_cmd(addr, 0, FUSE_CMD_READ) & *mask;
		if (reg != (*data & *mask)) {
			pr_err("changing 0x%x to 0x%x is not allowed\n",
				fuse_send_cmd(addr, 0, FUSE_CMD_READ),
				*data);
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

void tegra_fuse_pgm_disable(void)
{
	mutex_lock(&fuse_lock);
	fuse_writel(0x1, FUSE_DIS_PGM);
	mutex_unlock(&fuse_lock);
}

