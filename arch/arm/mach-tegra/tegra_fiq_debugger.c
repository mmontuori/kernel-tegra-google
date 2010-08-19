/*
 * arch/arm/mach-tegra/fiq_debugger.c
 *
 * Serial Debugger Interface for Tegra
 *
 * Copyright (C) 2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <stdarg.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/stacktrace.h>
#include <asm/hardware/fiq_debugger.h>
#include <mach/tegra_fiq_debugger.h>
#include <mach/system.h>
#include <mach/fiq.h>

#include <linux/uaccess.h>

#include <mach/legacy_irq.h>

#define UART_THR_DLAB	0x00
#define UART_IER_DLAB	0x04
#define UART_FIFO_IIR	0x08	/* control and interrupt-id */
#define UART_LCR	0x0c	/* line control */
#define UART_MCR	0x10	/* modem control */
#define UART_LSR	0x14	/* line status */
#define UART_MSR	0x18	/* modem status */
#define UART_IRDA_CSR	0x20

#define UART_ASR	0x3c	/* auto-sense baud register */

static void *debug_port_base;

static inline void tegra_write(unsigned int val, unsigned int off)
{
	__raw_writeb(val, debug_port_base + off);
}

static inline unsigned int tegra_read(unsigned int off)
{
	return __raw_readb(debug_port_base + off);
}

static void debug_port_init(void)
{
	if (tegra_read(UART_LSR) & 1)
		(void)tegra_read(UART_THR_DLAB);
	tegra_write(5, UART_IER_DLAB); /* enable rx and lsr interrupt */
	tegra_write(0, UART_FIFO_IIR); /* interrupt on every character */
}

static int debug_getc(void)
{
	unsigned int lsr = tegra_read(UART_LSR);
	if (lsr & 0x10)
		return 0x100;
	else if (lsr & 1)
		return tegra_read(UART_THR_DLAB);
	else
		return -1;
}

static void debug_putc(unsigned int c)
{
	while (!(tegra_read(UART_LSR) & (1<<5)))
		;
	tegra_write(c, UART_THR_DLAB);
}

static void debug_flush(void)
{
	while (!(tegra_read(UART_LSR) & (1<<6)))
		;
}

static void fiq_set_handler(unsigned int irq,
		void (*handler)(void *, void *, void *), void *data)
{
	tegra_fiq_select(irq, 1);
	tegra_fiq_set_handler(handler, data);
}

static void fiq_enable(unsigned int irq, bool on)
{
	if (on)
		tegra_fiq_enable(irq);
	else
		tegra_fiq_disable(irq);
}

static struct fiq_debugger init = {
	.uart_init = debug_port_init,
	.uart_getc = debug_getc,
	.uart_putc = debug_putc,
	.uart_flush = debug_flush,
	.fiq_set_handler = fiq_set_handler,
	.fiq_enable = fiq_enable,
	.force_irq = tegra_legacy_force_irq_set,
	.force_irq_ack = tegra_legacy_force_irq_clr
};

void tegra_serial_debug_init(unsigned int base, int irq,
			   struct clk *clk, int signal_irq, int wakeup_irq)
{

	init.base = base;
	init.fiq_irq = irq;
	init.uart_clk = clk;
	init.signal_irq = signal_irq;
	init.wakeup_irq = wakeup_irq;

	debug_port_base = ioremap(base, PAGE_SIZE);
	if (!debug_port_base)
		return;

	fiq_debugger_init(&init);
}
