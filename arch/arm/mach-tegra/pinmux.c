/*
 * linux/arch/arm/mach-tegra/pinmux.c
 *
 * Copyright (C) 2010 Google, Inc.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/io.h>

#include <mach/iomap.h>
#include <mach/pinmux.h>

#define HSM_EN(reg)	(((reg) >> 2) & 0x1)
#define SCHMT_EN(reg)	(((reg) >> 3) & 0x1)
#define LPMD(reg)	(((reg) >> 4) & 0x3)
#define DRVDN(reg)	(((reg) >> 12) & 0x1f)
#define DRVUP(reg)	(((reg) >> 20) & 0x1f)
#define SLWR(reg)	(((reg) >> 28) & 0x3)
#define SLWF(reg)	(((reg) >> 30) & 0x3)

static const struct tegra_pingroup_desc *const pingroups = tegra_soc_pingroups;

static char *tegra_mux_names[TEGRA_MAX_MUX] = {
	[TEGRA_MUX_AHB_CLK] = "AHB_CLK",
	[TEGRA_MUX_APB_CLK] = "APB_CLK",
	[TEGRA_MUX_AUDIO_SYNC] = "AUDIO_SYNC",
	[TEGRA_MUX_CRT] = "CRT",
	[TEGRA_MUX_DAP1] = "DAP1",
	[TEGRA_MUX_DAP2] = "DAP2",
	[TEGRA_MUX_DAP3] = "DAP3",
	[TEGRA_MUX_DAP4] = "DAP4",
	[TEGRA_MUX_DAP5] = "DAP5",
	[TEGRA_MUX_DISPLAYA] = "DISPLAYA",
	[TEGRA_MUX_DISPLAYB] = "DISPLAYB",
	[TEGRA_MUX_EMC_TEST0_DLL] = "EMC_TEST0_DLL",
	[TEGRA_MUX_EMC_TEST1_DLL] = "EMC_TEST1_DLL",
	[TEGRA_MUX_GMI] = "GMI",
	[TEGRA_MUX_GMI_INT] = "GMI_INT",
	[TEGRA_MUX_HDMI] = "HDMI",
	[TEGRA_MUX_I2C] = "I2C",
	[TEGRA_MUX_I2C2] = "I2C2",
	[TEGRA_MUX_I2C3] = "I2C3",
	[TEGRA_MUX_IDE] = "IDE",
	[TEGRA_MUX_IRDA] = "IRDA",
	[TEGRA_MUX_KBC] = "KBC",
	[TEGRA_MUX_MIO] = "MIO",
	[TEGRA_MUX_MIPI_HS] = "MIPI_HS",
	[TEGRA_MUX_NAND] = "NAND",
	[TEGRA_MUX_OSC] = "OSC",
	[TEGRA_MUX_OWR] = "OWR",
	[TEGRA_MUX_PCIE] = "PCIE",
	[TEGRA_MUX_PLLA_OUT] = "PLLA_OUT",
	[TEGRA_MUX_PLLC_OUT1] = "PLLC_OUT1",
	[TEGRA_MUX_PLLM_OUT1] = "PLLM_OUT1",
	[TEGRA_MUX_PLLP_OUT2] = "PLLP_OUT2",
	[TEGRA_MUX_PLLP_OUT3] = "PLLP_OUT3",
	[TEGRA_MUX_PLLP_OUT4] = "PLLP_OUT4",
	[TEGRA_MUX_PWM] = "PWM",
	[TEGRA_MUX_PWR_INTR] = "PWR_INTR",
	[TEGRA_MUX_PWR_ON] = "PWR_ON",
	[TEGRA_MUX_RTCK] = "RTCK",
	[TEGRA_MUX_SDIO1] = "SDIO1",
	[TEGRA_MUX_SDIO2] = "SDIO2",
	[TEGRA_MUX_SDIO3] = "SDIO3",
	[TEGRA_MUX_SDIO4] = "SDIO4",
	[TEGRA_MUX_SFLASH] = "SFLASH",
	[TEGRA_MUX_SPDIF] = "SPDIF",
	[TEGRA_MUX_SPI1] = "SPI1",
	[TEGRA_MUX_SPI2] = "SPI2",
	[TEGRA_MUX_SPI2_ALT] = "SPI2_ALT",
	[TEGRA_MUX_SPI3] = "SPI3",
	[TEGRA_MUX_SPI4] = "SPI4",
	[TEGRA_MUX_TRACE] = "TRACE",
	[TEGRA_MUX_TWC] = "TWC",
	[TEGRA_MUX_UARTA] = "UARTA",
	[TEGRA_MUX_UARTB] = "UARTB",
	[TEGRA_MUX_UARTC] = "UARTC",
	[TEGRA_MUX_UARTD] = "UARTD",
	[TEGRA_MUX_UARTE] = "UARTE",
	[TEGRA_MUX_ULPI] = "ULPI",
	[TEGRA_MUX_VI] = "VI",
	[TEGRA_MUX_VI_SENSOR_CLK] = "VI_SENSOR_CLK",
	[TEGRA_MUX_XIO] = "XIO",
};

struct tegra_drive_pingroup_desc {
	const char *name;
	s16 reg;
};

#define DRIVE_PINGROUP(pg_name, r)				\
	[TEGRA_DRIVE_PINGROUP_ ## pg_name] = {		\
		.name = #pg_name,				\
		.reg = r 					\
	}

static const struct tegra_drive_pingroup_desc drive_pingroups[TEGRA_MAX_PINGROUP] = {
	DRIVE_PINGROUP(AO1,		0x868),
	DRIVE_PINGROUP(AO2,		0x86c),
	DRIVE_PINGROUP(AT1,		0x870),
	DRIVE_PINGROUP(AT2,		0x874),
	DRIVE_PINGROUP(CDEV1,		0x878),
	DRIVE_PINGROUP(CDEV2,		0x87c),
	DRIVE_PINGROUP(CSUS,		0x880),
	DRIVE_PINGROUP(DAP1,		0x884),
	DRIVE_PINGROUP(DAP2,		0x888),
	DRIVE_PINGROUP(DAP3,		0x88c),
	DRIVE_PINGROUP(DAP4,		0x890),
	DRIVE_PINGROUP(DBG,		0x894),
	DRIVE_PINGROUP(LCD1,		0x898),
	DRIVE_PINGROUP(LCD2,		0x89c),
	DRIVE_PINGROUP(SDMMC2,	0x8a0),
	DRIVE_PINGROUP(SDMMC3,	0x8a4),
	DRIVE_PINGROUP(SPI,		0x8a8),
	DRIVE_PINGROUP(UAA,		0x8ac),
	DRIVE_PINGROUP(UAB,		0x8b0),
	DRIVE_PINGROUP(UART2,		0x8b4),
	DRIVE_PINGROUP(UART3,		0x8b8),
	DRIVE_PINGROUP(VI1,		0x8bc),
	DRIVE_PINGROUP(VI2,		0x8c0),
	DRIVE_PINGROUP(XM2A,		0x8c4),
	DRIVE_PINGROUP(XM2C,		0x8c8),
	DRIVE_PINGROUP(XM2D,		0x8cc),
	DRIVE_PINGROUP(XM2CLK,	0x8d0),
	DRIVE_PINGROUP(MEMCOMP,	0x8d4),
};

static const char *tegra_drive_names[TEGRA_MAX_DRIVE] = {
	[TEGRA_DRIVE_DIV_8] = "DIV_8",
	[TEGRA_DRIVE_DIV_4] = "DIV_4",
	[TEGRA_DRIVE_DIV_2] = "DIV_2",
	[TEGRA_DRIVE_DIV_1] = "DIV_1",
};

static const char *tegra_slew_names[TEGRA_MAX_SLEW] = {
	[TEGRA_SLEW_FASTEST] = "FASTEST",
	[TEGRA_SLEW_FAST] = "FAST",
	[TEGRA_SLEW_SLOW] = "SLOW",
	[TEGRA_SLEW_SLOWEST] = "SLOWEST",
};

static DEFINE_SPINLOCK(mux_lock);

static const char *pingroup_name(tegra_pingroup_t pg)
{
	if (pg < 0 || pg >=  TEGRA_MAX_PINGROUP)
		return "<UNKNOWN>";

	return pingroups[pg].name;
}

static const char *func_name(tegra_mux_func_t func)
{
	if (func == TEGRA_MUX_RSVD1)
		return "RSVD1";

	if (func == TEGRA_MUX_RSVD2)
		return "RSVD2";

	if (func == TEGRA_MUX_RSVD3)
		return "RSVD3";

	if (func == TEGRA_MUX_RSVD4)
		return "RSVD4";

	if (func == TEGRA_MUX_NONE)
		return "NONE";

	if (func < 0 || func >=  TEGRA_MAX_MUX)
		return "<UNKNOWN>";

	return tegra_mux_names[func];
}


static const char *tri_name(unsigned long val)
{
	return val ? "TRISTATE" : "NORMAL";
}

static const char *pupd_name(unsigned long val)
{
	switch (val) {
	case 0:
		return "NORMAL";

	case 1:
		return "PULL_DOWN";

	case 2:
		return "PULL_UP";

	default:
		return "RSVD";
	}
}


static inline unsigned long pg_readl(unsigned long offset)
{
	return readl(IO_TO_VIRT(TEGRA_APB_MISC_BASE + offset));
}

static inline void pg_writel(unsigned long value, unsigned long offset)
{
	writel(value, IO_TO_VIRT(TEGRA_APB_MISC_BASE + offset));
}

static int tegra_pinmux_cancel_func(const struct tegra_pingroup_config *config)
{
	int mux = -1;
	int mux_safe = -1;
	int i;
	unsigned long reg;
	unsigned long flags;
	tegra_pingroup_t pg = config->pingroup;
	tegra_mux_func_t func = config->func;

	if (pg < 0 || pg >=  TEGRA_MAX_PINGROUP)
		return -ERANGE;

	if (pingroups[pg].mux_reg < 0)
		return -EINVAL;

	if (func < 0)
		return -ERANGE;

	if (func & TEGRA_MUX_RSVD) {
		return 0;
	}

	for (i = 0; i < 4; i++) {
		if (pingroups[pg].funcs[i] == func) {
			mux = i;
			break;
		}
	}

	if (pingroups[pg].func_safe & TEGRA_MUX_RSVD) {
		mux_safe = pingroups[pg].func_safe & 3;
	} else {
		for (i = 0; i < 4; i++) {
			if (pingroups[pg].funcs[i] == pingroups[pg].func_safe) {
				mux_safe = i;
				break;
			}
		}
	}

	if (mux < 0 || mux_safe < 0)
		return -EINVAL;

	spin_lock_irqsave(&mux_lock, flags);

	reg = pg_readl(pingroups[pg].mux_reg);

	if (((reg >> pingroups[pg].mux_bit) & 0x3) == mux) {
		reg &= ~(0x3 << pingroups[pg].mux_bit);
		reg |= mux_safe << pingroups[pg].mux_bit;
		pg_writel(reg, pingroups[pg].mux_reg);
	}

	spin_unlock_irqrestore(&mux_lock, flags);

	return 0;
}

static int tegra_pinmux_set_func(const struct tegra_pingroup_config *config)
{
	int mux = -1;
	int i;
	unsigned long reg;
	unsigned long flags;
	tegra_pingroup_t pg = config->pingroup;
	tegra_mux_func_t func = config->func;

	if (pg < 0 || pg >=  TEGRA_MAX_PINGROUP)
		return -ERANGE;

	if (pingroups[pg].mux_reg < 0)
		return -EINVAL;

	if (func < 0)
		return -ERANGE;

	if (func & TEGRA_MUX_RSVD) {
		mux = func & 0x3;
	} else {
		for (i = 0; i < 4; i++) {
			if (pingroups[pg].funcs[i] == func) {
				mux = i;
				break;
			}
		}
	}

	if (mux < 0)
		return -EINVAL;

	spin_lock_irqsave(&mux_lock, flags);

	reg = pg_readl(pingroups[pg].mux_reg);
	reg &= ~(0x3 << pingroups[pg].mux_bit);
	reg |= mux << pingroups[pg].mux_bit;
	pg_writel(reg, pingroups[pg].mux_reg);

	spin_unlock_irqrestore(&mux_lock, flags);

	return 0;
}

int tegra_pinmux_get_tristate(tegra_pingroup_t pg)
{
	unsigned long reg;

	if (pg < 0 || pg >=  TEGRA_MAX_PINGROUP)
		return 0;

	if (pingroups[pg].tri_reg < 0)
		return 0;

	reg = pg_readl(pingroups[pg].tri_reg);
	if (reg & (1 << pingroups[pg].tri_bit))
		return 0;
	else
		return 1;
}

int tegra_pinmux_set_tristate(tegra_pingroup_t pg, tegra_tristate_t tristate)
{
	unsigned long reg;
	unsigned long flags;

	if (pg < 0 || pg >=  TEGRA_MAX_PINGROUP)
		return -ERANGE;

	if (pingroups[pg].tri_reg < 0)
		return -EINVAL;

	spin_lock_irqsave(&mux_lock, flags);

	reg = pg_readl(pingroups[pg].tri_reg);
	reg &= ~(0x1 << pingroups[pg].tri_bit);
	if (tristate)
		reg |= 1 << pingroups[pg].tri_bit;
	pg_writel(reg, pingroups[pg].tri_reg);

	spin_unlock_irqrestore(&mux_lock, flags);

	return 0;
}

int tegra_pinmux_set_pullupdown(tegra_pingroup_t pg, tegra_pullupdown_t pupd)
{
	unsigned long reg;
	unsigned long flags;

	if (pg < 0 || pg >=  TEGRA_MAX_PINGROUP)
		return -ERANGE;

	if (pingroups[pg].pupd_reg < 0)
		return -EINVAL;

	if (pupd != TEGRA_PUPD_NORMAL &&
	    pupd != TEGRA_PUPD_PULL_DOWN &&
	    pupd != TEGRA_PUPD_PULL_UP)
		return -EINVAL;


	spin_lock_irqsave(&mux_lock, flags);

	reg = pg_readl(pingroups[pg].pupd_reg);
	reg &= ~(0x3 << pingroups[pg].pupd_bit);
	reg |= pupd << pingroups[pg].pupd_bit;
	pg_writel(reg, pingroups[pg].pupd_reg);

	spin_unlock_irqrestore(&mux_lock, flags);

	return 0;
}

static void tegra_pinmux_config_pingroup(const struct tegra_pingroup_config *config)
{
	tegra_pingroup_t pingroup = config->pingroup;
	tegra_mux_func_t func     = config->func;
	tegra_pullupdown_t pupd   = config->pupd;
	tegra_tristate_t tristate = config->tristate;
	int err;

	if (pingroups[pingroup].mux_reg >= 0) {
		err = tegra_pinmux_set_func(config);
		if (err < 0)
			pr_err("pinmux: can't set pingroup %s func to %s: %d\n",
			       pingroup_name(pingroup), func_name(func), err);
	}

	if (pingroups[pingroup].pupd_reg >= 0) {
		err = tegra_pinmux_set_pullupdown(pingroup, pupd);
		if (err < 0)
			pr_err("pinmux: can't set pingroup %s pullupdown to %s: %d\n",
			       pingroup_name(pingroup), pupd_name(pupd), err);
	}

	if (pingroups[pingroup].tri_reg >= 0) {
		err = tegra_pinmux_set_tristate(pingroup, tristate);
		if (err < 0)
			pr_err("pinmux: can't set pingroup %s tristate to %s: %d\n",
			       pingroup_name(pingroup), tri_name(func), err);
	}
}



void tegra_pinmux_config_table(const struct tegra_pingroup_config *config, int len)
{
	int i;

	for (i = 0; i < len; i++)
		tegra_pinmux_config_pingroup(&config[i]);
}

static const char *drive_pinmux_name(tegra_drive_pingroup_t pg)
{
	if (pg < 0 || pg >=  TEGRA_MAX_DRIVE_PINGROUP)
		return "<UNKNOWN>";

	return drive_pingroups[pg].name;
}

static const char *enable_name(unsigned long val)
{
	return val ? "ENABLE" : "DISABLE";
}

static const char *drive_name(unsigned long val)
{
	if (val >= TEGRA_MAX_DRIVE)
		return "<UNKNOWN>";

	return tegra_drive_names[val];
}

static const char *slew_name(unsigned long val)
{
	if (val >= TEGRA_MAX_SLEW)
		return "<UNKNOWN>";

	return tegra_slew_names[val];
}

static int tegra_drive_pinmux_set_hsm(tegra_drive_pingroup_t pg, tegra_hsm_t hsm)
{
	unsigned long flags;
	u32 reg;
	if (pg < 0 || pg >=  TEGRA_MAX_DRIVE_PINGROUP)
		return -ERANGE;

	if (hsm != TEGRA_HSM_ENABLE && hsm != TEGRA_HSM_DISABLE)
		return -EINVAL;

	spin_lock_irqsave(&mux_lock, flags);

	reg = pg_readl(drive_pingroups[pg].reg);
	if (hsm == TEGRA_HSM_ENABLE)
		reg |= (1 << 2);
	else
		reg &= ~(1 << 2);
	pg_writel(reg, drive_pingroups[pg].reg);

	spin_unlock_irqrestore(&mux_lock, flags);

	return 0;
}

static int tegra_drive_pinmux_set_schmitt(tegra_drive_pingroup_t pg,
	tegra_schmitt_t schmitt)
{
	unsigned long flags;
	u32 reg;
	if (pg < 0 || pg >=  TEGRA_MAX_DRIVE_PINGROUP)
		return -ERANGE;

	if (schmitt != TEGRA_SCHMITT_ENABLE && schmitt != TEGRA_SCHMITT_DISABLE)
		return -EINVAL;

	spin_lock_irqsave(&mux_lock, flags);

	reg = pg_readl(drive_pingroups[pg].reg);
	if (schmitt == TEGRA_SCHMITT_ENABLE)
		reg |= (1 << 3);
	else
		reg &= ~(1 << 3);
	pg_writel(reg, drive_pingroups[pg].reg);

	spin_unlock_irqrestore(&mux_lock, flags);

	return 0;
}

static int tegra_drive_pinmux_set_drive(tegra_drive_pingroup_t pg,
	tegra_drive_t drive)
{
	unsigned long flags;
	u32 reg;
	if (pg < 0 || pg >=  TEGRA_MAX_DRIVE_PINGROUP)
		return -ERANGE;

	if (drive < 0 || drive >= TEGRA_MAX_DRIVE)
		return -EINVAL;

	spin_lock_irqsave(&mux_lock, flags);

	reg = pg_readl(drive_pingroups[pg].reg);
	reg &= ~(0x3 << 4);
	reg |= drive << 4;
	pg_writel(reg, drive_pingroups[pg].reg);

	spin_unlock_irqrestore(&mux_lock, flags);

	return 0;
}

static int tegra_drive_pinmux_set_pull_down(tegra_drive_pingroup_t pg,
	tegra_pull_strength_t pull_down)
{
	unsigned long flags;
	u32 reg;
	if (pg < 0 || pg >=  TEGRA_MAX_DRIVE_PINGROUP)
		return -ERANGE;

	if (pull_down < 0 || pull_down >= TEGRA_MAX_PULL)
		return -EINVAL;

	spin_lock_irqsave(&mux_lock, flags);

	reg = pg_readl(drive_pingroups[pg].reg);
	reg &= ~(0x1f << 12);
	reg |= pull_down << 12;
	pg_writel(reg, drive_pingroups[pg].reg);

	spin_unlock_irqrestore(&mux_lock, flags);

	return 0;
}

static int tegra_drive_pinmux_set_pull_up(tegra_drive_pingroup_t pg,
	tegra_pull_strength_t pull_up)
{
	unsigned long flags;
	u32 reg;
	if (pg < 0 || pg >=  TEGRA_MAX_DRIVE_PINGROUP)
		return -ERANGE;

	if (pull_up < 0 || pull_up >= TEGRA_MAX_PULL)
		return -EINVAL;

	spin_lock_irqsave(&mux_lock, flags);

	reg = pg_readl(drive_pingroups[pg].reg);
	reg &= ~(0x1f << 12);
	reg |= pull_up << 12;
	pg_writel(reg, drive_pingroups[pg].reg);

	spin_unlock_irqrestore(&mux_lock, flags);

	return 0;
}

static int tegra_drive_pinmux_set_slew_rising(tegra_drive_pingroup_t pg,
	tegra_slew_t slew_rising)
{
	unsigned long flags;
	u32 reg;
	if (pg < 0 || pg >=  TEGRA_MAX_DRIVE_PINGROUP)
		return -ERANGE;

	if (slew_rising < 0 || slew_rising >= TEGRA_MAX_SLEW)
		return -EINVAL;

	spin_lock_irqsave(&mux_lock, flags);

	reg = pg_readl(drive_pingroups[pg].reg);
	reg &= ~(0x3 << 28);
	reg |= slew_rising << 28;
	pg_writel(reg, drive_pingroups[pg].reg);

	spin_unlock_irqrestore(&mux_lock, flags);

	return 0;
}

static int tegra_drive_pinmux_set_slew_falling(tegra_drive_pingroup_t pg,
	tegra_slew_t slew_falling)
{
	unsigned long flags;
	u32 reg;
	if (pg < 0 || pg >=  TEGRA_MAX_DRIVE_PINGROUP)
		return -ERANGE;

	if (slew_falling < 0 || slew_falling >= TEGRA_MAX_SLEW)
		return -EINVAL;

	spin_lock_irqsave(&mux_lock, flags);

	reg = pg_readl(drive_pingroups[pg].reg);
	reg &= ~(0x3 << 30);
	reg |= slew_falling << 30;
	pg_writel(reg, drive_pingroups[pg].reg);

	spin_unlock_irqrestore(&mux_lock, flags);

	return 0;
}

static void tegra_drive_pinmux_config_pingroup(tegra_drive_pingroup_t pingroup,
					  tegra_hsm_t hsm,
					  tegra_schmitt_t schmitt,
					  tegra_drive_t drive,
					  tegra_pull_strength_t pull_down,
					  tegra_pull_strength_t pull_up,
					  tegra_slew_t slew_rising,
					  tegra_slew_t slew_falling)
{
	int err;

	err = tegra_drive_pinmux_set_hsm(pingroup, hsm);
	if (err < 0)
		pr_err("pinmux: can't set pingroup %s hsm to %s: %d\n",
			drive_pinmux_name(pingroup),
			enable_name(hsm), err);

	err = tegra_drive_pinmux_set_schmitt(pingroup, schmitt);
	if (err < 0)
		pr_err("pinmux: can't set pingroup %s schmitt to %s: %d\n",
			drive_pinmux_name(pingroup),
			enable_name(schmitt), err);

	err = tegra_drive_pinmux_set_drive(pingroup, drive);
	if (err < 0)
		pr_err("pinmux: can't set pingroup %s drive to %s: %d\n",
			drive_pinmux_name(pingroup),
			drive_name(drive), err);

	err = tegra_drive_pinmux_set_pull_down(pingroup, pull_down);
	if (err < 0)
		pr_err("pinmux: can't set pingroup %s pull down to %d: %d\n",
			drive_pinmux_name(pingroup),
			pull_down, err);

	err = tegra_drive_pinmux_set_pull_up(pingroup, pull_up);
	if (err < 0)
		pr_err("pinmux: can't set pingroup %s pull up to %d: %d\n",
			drive_pinmux_name(pingroup),
			pull_up, err);

	err = tegra_drive_pinmux_set_slew_rising(pingroup, slew_rising);
	if (err < 0)
		pr_err("pinmux: can't set pingroup %s rising slew to %s: %d\n",
			drive_pinmux_name(pingroup),
			slew_name(slew_rising), err);

	err = tegra_drive_pinmux_set_slew_falling(pingroup, slew_falling);
	if (err < 0)
		pr_err("pinmux: can't set pingroup %s falling slew to %s: %d\n",
			drive_pinmux_name(pingroup),
			slew_name(slew_falling), err);
}

void tegra_drive_pinmux_config_table(struct tegra_drive_pingroup_config *config,
	int len)
{
	int i;

	for (i = 0; i < len; i++)
		tegra_drive_pinmux_config_pingroup(config[i].pingroup,
						     config[i].hsm,
						     config[i].schmitt,
						     config[i].drive,
						     config[i].pull_down,
						     config[i].pull_up,
						     config[i].slew_rising,
						     config[i].slew_falling);
}

void tegra_pinmux_config_pinmux_table(const struct tegra_pingroup_config *config,
				      int len, bool is_set)
{
	int i;
	int err;

	for (i = 0; i < len; i++) {
		if (pingroups[config[i].pingroup].mux_reg >= 0) {
			if (is_set)
				err = tegra_pinmux_set_func(&config[i]);
			else
				err = tegra_pinmux_cancel_func(&config[i]);
			if (err < 0)
				pr_err("pinmux: can't set pingroup %s func"
					" to %s: %d\n", pingroup_name(config[i].pingroup),
					func_name(config[i].func), err);
		}
	}
}

void tegra_pinmux_config_tristate_table(const struct tegra_pingroup_config *config,
					int len, tegra_tristate_t tristate)
{
	int i;
	int err;
	tegra_pingroup_t pingroup;

	for (i = 0; i < len; i++) {
		pingroup = config[i].pingroup;
		if (pingroups[pingroup].tri_reg >= 0) {
			err = tegra_pinmux_set_tristate(pingroup, tristate);
			if (err < 0)
				pr_err("pinmux: can't set pingroup %s tristate"
					" to %s: %d\n",	pingroup_name(pingroup),
					tri_name(tristate), err);
		}
	}
}

void tegra_pinmux_set_vddio_tristate(tegra_vddio_t vddio,
				     tegra_tristate_t tristate)
{
	int pg;
	for (pg = 0; pg < TEGRA_MAX_PINGROUP; ++pg) {
		if (pingroups[pg].vddio == vddio &&
		    pingroups[pg].tri_reg >= 0) {
			if (tegra_pinmux_set_tristate(pg, tristate)<0)
				pr_err("pinmux: can't set pingroup %s tristate"
				       " to %s\n", pingroup_name(pg),
				       tri_name(tristate));
		}
	}
}

void tegra_pinmux_config_pullupdown_table(const struct tegra_pingroup_config *config,
					  int len, tegra_pullupdown_t pupd)
{
	int i;
	int err;
	tegra_pingroup_t pingroup;

	for (i = 0; i < len; i++) {
		pingroup = config[i].pingroup;
		if (pingroups[pingroup].pupd_reg >= 0) {
			err = tegra_pinmux_set_pullupdown(pingroup, pupd);
			if (err < 0)
				pr_err("pinmux: can't set pingroup %s pullupdown"
					" to %s: %d\n",	pingroup_name(pingroup),
					pupd_name(pupd), err);
		}
	}
}

#ifdef	CONFIG_DEBUG_FS

#include <linux/debugfs.h>
#include <linux/seq_file.h>

static void dbg_pad_field(struct seq_file *s, int len)
{
	seq_putc(s, ',');

	while (len-- > -1)
		seq_putc(s, ' ');
}

static int dbg_pinmux_show(struct seq_file *s, void *unused)
{
	int i;
	int len;

	for (i = 0; i < TEGRA_MAX_PINGROUP; i++) {
		unsigned long tri;
		unsigned long mux;
		unsigned long pupd;

		seq_printf(s, "\t{TEGRA_PINGROUP_%s", pingroups[i].name);
		len = strlen(pingroups[i].name);
		dbg_pad_field(s, 5 - len);

		if (pingroups[i].mux_reg < 0) {
			seq_printf(s, "TEGRA_MUX_NONE");
			len = strlen("NONE");
		} else {
			mux = (pg_readl(pingroups[i].mux_reg) >>
			       pingroups[i].mux_bit) & 0x3;
			if (pingroups[i].funcs[mux] == TEGRA_MUX_RSVD) {
				seq_printf(s, "TEGRA_MUX_RSVD%1lu", mux+1);
				len = 5;
			} else {
				seq_printf(s, "TEGRA_MUX_%s",
					   tegra_mux_names[pingroups[i].funcs[mux]]);
				len = strlen(tegra_mux_names[pingroups[i].funcs[mux]]);
			}
		}
		dbg_pad_field(s, 13-len);

		if (pingroups[i].mux_reg < 0) {
			seq_printf(s, "TEGRA_PUPD_NORMAL");
			len = strlen("NORMAL");
		} else {
			pupd = (pg_readl(pingroups[i].pupd_reg) >>
				pingroups[i].pupd_bit) & 0x3;
			seq_printf(s, "TEGRA_PUPD_%s", pupd_name(pupd));
			len = strlen(pupd_name(pupd));
		}
		dbg_pad_field(s, 9 - len);

		if (pingroups[i].tri_reg < 0) {
			seq_printf(s, "TEGRA_TRI_NORMAL");
		} else {
			tri = (pg_readl(pingroups[i].tri_reg) >>
			       pingroups[i].tri_bit) & 0x1;

			seq_printf(s, "TEGRA_TRI_%s", tri_name(tri));
		}
		seq_printf(s, "},\n");
	}
	return 0;
}

static int dbg_pinmux_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_pinmux_show, &inode->i_private);
}

static const struct file_operations debug_fops = {
	.open		= dbg_pinmux_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dbg_drive_pinmux_show(struct seq_file *s, void *unused)
{
	int i;
	int len;

	for (i = 0; i < TEGRA_MAX_DRIVE_PINGROUP; i++) {
		u32 reg;

		seq_printf(s, "\t{TEGRA_DRIVE_PINGROUP_%s",
			drive_pingroups[i].name);
		len = strlen(drive_pingroups[i].name);
		dbg_pad_field(s, 7 - len);


		reg = pg_readl(drive_pingroups[i].reg);
		if (HSM_EN(reg)) {
			seq_printf(s, "TEGRA_HSM_ENABLE");
			len = 16;
		} else {
			seq_printf(s, "TEGRA_HSM_DISABLE");
			len = 17;
		}
		dbg_pad_field(s, 17 - len);

		if (SCHMT_EN(reg)) {
			seq_printf(s, "TEGRA_SCHMITT_ENABLE");
			len = 21;
		} else {
			seq_printf(s, "TEGRA_SCHMITT_DISABLE");
			len = 22;
		}
		dbg_pad_field(s, 22 - len);

		seq_printf(s, "TEGRA_DRIVE_%s", drive_name(LPMD(reg)));
		len = strlen(drive_name(LPMD(reg)));
		dbg_pad_field(s, 5 - len);

		seq_printf(s, "TEGRA_PULL_%d", DRVDN(reg));
		len = DRVDN(reg) < 10 ? 1 : 2;
		dbg_pad_field(s, 2 - len);

		seq_printf(s, "TEGRA_PULL_%d", DRVUP(reg));
		len = DRVUP(reg) < 10 ? 1 : 2;
		dbg_pad_field(s, 2 - len);

		seq_printf(s, "TEGRA_SLEW_%s", slew_name(SLWR(reg)));
		len = strlen(slew_name(SLWR(reg)));
		dbg_pad_field(s, 7 - len);

		seq_printf(s, "TEGRA_SLEW_%s", slew_name(SLWF(reg)));

		seq_printf(s, "},\n");
	}
	return 0;
}

static int dbg_drive_pinmux_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_drive_pinmux_show, &inode->i_private);
}

static const struct file_operations debug_drive_fops = {
	.open		= dbg_drive_pinmux_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init tegra_pinmux_debuginit(void)
{
	(void) debugfs_create_file("tegra_pinmux", S_IRUGO,
					NULL, NULL, &debug_fops);
	(void) debugfs_create_file("tegra_pinmux_drive", S_IRUGO,
					NULL, NULL, &debug_drive_fops);
	return 0;
}
late_initcall(tegra_pinmux_debuginit);
#endif
