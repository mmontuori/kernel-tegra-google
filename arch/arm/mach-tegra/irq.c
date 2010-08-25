/*
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@google.com>
 *
 * Copyright (C) 2010, NVIDIA Corporation
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
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>

#include <asm/hardware/gic.h>

#include <mach/iomap.h>
#include <mach/legacy_irq.h>
#include <mach/suspend.h>

#include "board.h"

#define PMC_CTRL		0x0
#define PMC_CTRL_LATCH_WAKEUPS	(1 << 5)
#define PMC_WAKE_MASK		0xc
#define PMC_WAKE_LEVEL		0x10
#define PMC_WAKE_STATUS		0x14
#define PMC_SW_WAKE_STATUS	0x18
#define PMC_DPD_SAMPLE  	0x20

static void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);

static void (*gic_mask_irq)(unsigned int irq);
static void (*gic_unmask_irq)(unsigned int irq);

static u32 tegra_lp0_wake_enb;
static u32 tegra_lp0_wake_level;
static u32 tegra_lp0_wake_level_any;

#define IRQ_MASK_SIZE DIV_ROUND_UP(NR_IRQS, 32)

static u32 tegra_lp0_irq_mask[IRQ_MASK_SIZE];
static u32 tegra_lp0_possible_irq_mask[IRQ_MASK_SIZE];

/* ensures that sufficient time is passed for a register write to
 * serialize into the 32KHz domain */
static void pmc_32kwritel(u32 val, unsigned long offs)
{
	writel(val, pmc + offs);
	udelay(130);
}

int tegra_set_lp0_wake(int irq, int enable)
{
	int wake = tegra_irq_to_wake(irq);

	if (enable)
		tegra_lp0_irq_mask[irq / 32] |= 1 << (irq % 32);
	else
		tegra_lp0_irq_mask[irq / 32] &= ~(1 << (irq % 32));

	if (wake < 0)
		return -EINVAL;

	if (enable)
		tegra_lp0_wake_enb |= 1 << wake;
	else
		tegra_lp0_wake_enb &= ~(1 << wake);

	return 0;
}

int tegra_set_lp0_wake_type(int irq, int flow_type)
{
	int wake = tegra_irq_to_wake(irq);

	if (wake < 0)
		return 0;

	switch (flow_type) {
	case IRQF_TRIGGER_FALLING:
	case IRQF_TRIGGER_LOW:
		tegra_lp0_wake_level &= ~(1 << wake);
		tegra_lp0_wake_level_any &= ~(1 << wake);
		break;
	case IRQF_TRIGGER_HIGH:
	case IRQF_TRIGGER_RISING:
		tegra_lp0_wake_level |= 1 << wake;
		tegra_lp0_wake_level_any &= ~(1 << wake);
		break;

	case IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING:
		tegra_lp0_wake_level_any |= 1 << wake;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}


int tegra_set_lp1_wake(int irq, int enable)
{
	return tegra_legacy_irq_set_wake(irq, enable);
}

void tegra_set_lp0_wake_pads(u32 wake_enb, u32 wake_level, u32 wake_any)
{
	u32 temp;
	u32 status;
	u32 lvl;

	wake_level &= wake_enb;
	wake_any &= wake_enb;

	wake_level |= (tegra_lp0_wake_level & tegra_lp0_wake_enb);
	wake_any |= (tegra_lp0_wake_level_any & tegra_lp0_wake_enb);

	wake_enb |= tegra_lp0_wake_enb;

	pmc_32kwritel(0, PMC_SW_WAKE_STATUS);
	temp = readl(pmc + PMC_CTRL);
	temp |= PMC_CTRL_LATCH_WAKEUPS;
	pmc_32kwritel(temp, PMC_CTRL);
	temp &= ~PMC_CTRL_LATCH_WAKEUPS;
	pmc_32kwritel(temp, PMC_CTRL);
	status = readl(pmc + PMC_SW_WAKE_STATUS);
	lvl = readl(pmc + PMC_WAKE_LEVEL);

	/* flip the wakeup trigger for any-edge triggered pads
	 * which are currently asserting as wakeups */
	lvl ^= status;
	lvl &= wake_any;

	wake_level |= lvl;

	writel(wake_level, pmc + PMC_WAKE_LEVEL);
	/* Enable DPD sample to trigger sampling pads data and direction
	 * in which pad will be driven during lp0 mode*/
	writel(0x1, pmc + PMC_DPD_SAMPLE);

	writel(wake_enb, pmc + PMC_WAKE_MASK);
}

void tegra_irq_handle_wake(void)
{
	int wake;
	int irq;
	struct irq_desc *desc;

	unsigned long wake_status = readl(pmc + PMC_WAKE_STATUS);
	for_each_set_bit(wake, &wake_status, sizeof(wake_status) * 8) {
		irq = tegra_wake_to_irq(wake);
		if (!irq) {
			pr_info("Resume caused by WAKE%d\n", wake);
			continue;
		}

		desc = irq_to_desc(irq);
		if (!desc || !desc->action || !desc->action->name) {
			pr_info("Resume caused by WAKE%d, irq %d\n", wake, irq);
			continue;
		}

		pr_info("Resume caused by WAKE%d, %s\n", wake,
			desc->action->name);

		generic_handle_irq(irq);
	}
}

static void tegra_mask(unsigned int irq)
{
	gic_mask_irq(irq);
	tegra_legacy_mask_irq(irq);
}

static void tegra_unmask(unsigned int irq)
{
	gic_unmask_irq(irq);
	tegra_legacy_unmask_irq(irq);
}

static int tegra_set_wake(unsigned int irq, unsigned int enable)
{
	if (tegra_get_suspend_mode() == TEGRA_SUSPEND_LP0)
		return tegra_set_lp0_wake(irq, enable);
	else
		return tegra_set_lp1_wake(irq, enable);

}

static int tegra_set_type(unsigned int irq, unsigned int flow_type)
{
	if (tegra_get_suspend_mode() == TEGRA_SUSPEND_LP0)
		return tegra_set_lp0_wake_type(irq, flow_type);

	return 0;
}

static struct irq_chip tegra_irq = {
	.name		= "PPI",
	.mask		= tegra_mask,
	.unmask		= tegra_unmask,
	.set_wake	= tegra_set_wake,
	.set_type	= tegra_set_type,
};

void __init tegra_init_irq(void)
{
	struct irq_chip *gic;
	unsigned int i;

	tegra_init_legacy_irq();

	gic_dist_init(0, IO_ADDRESS(TEGRA_ARM_INT_DIST_BASE), 29);
	gic_cpu_init(0, IO_ADDRESS(TEGRA_ARM_PERIF_BASE + 0x100));

	gic = get_irq_chip(29);
	gic_unmask_irq = gic->unmask;
	gic_mask_irq = gic->mask;
	tegra_irq.ack = gic->ack;
#ifdef CONFIG_SMP
	tegra_irq.set_affinity = gic->set_affinity;
#endif

	for (i = INT_PRI_BASE; i < INT_GPIO_BASE; i++) {
		set_irq_chip(i, &tegra_irq);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}

	for (i = 0; i < 32; i++) {
		int irq = tegra_wake_to_irq(i);
		if (irq < 0)
			continue;

		tegra_lp0_possible_irq_mask[irq / 32] |= 1 << (irq % 32);
	}
}
