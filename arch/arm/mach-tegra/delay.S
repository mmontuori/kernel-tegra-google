/*
 * arch/arm/mach-tegra/delay.S
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *  Colin Cross <ccross@google.com>
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

#include <linux/linkage.h>
#include <asm/assembler.h>
#include <mach/iomap.h>
#include <mach/io.h>

    .text

#define TMRUS_ASM_BASE (IO_PPSB_VIRT + 0x60005010 - IO_PPSB_PHYS)

tmrus_base:
        .word TMRUS_ASM_BASE

ENTRY(__udelay)
ENTRY(__const_udelay)
    ldr r3, tmrus_base
    ldr r1, [r3]
    adds r2, r1, r0
    bcs udelay_wrap

/* the expected final value will not wrap the counter
 * r1 - initial value of the counter
 * r2 - final value
 * end condition is r0 > r2 or r0 < r1
 * the second test catches the case when the counter wraps after reaching
 * the end value but before we get a chance to read it
 */
udelay_nowrap:
    ldr r0, [r3]
    cmp r0, r2
    movhi pc, lr
    cmp r0, r1
    movlo pc, lr
    b udelay_nowrap

/* the expected final value WILL wrap the counter
 * r1 - initial value of the counter
 * r2 - final value
 * end condition is r0 > r2 AND r0 < r1
 */
udelay_wrap:
    ldr r0, [r3]
    cmp r0, r2
    bls udelay_wrap
    cmp r0, r1
    bhs udelay_wrap
    mov pc, lr
ENDPROC(__const_udelay)
ENDPROC(__udelay)


@ Delay routine
ENTRY(__delay)
    subs  r0, r0, #1
    bhi __delay
    mov pc, lr
ENDPROC(__delay)
