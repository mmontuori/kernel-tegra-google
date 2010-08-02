/*
 * arch/arm/include/asm/attrib_alloc.h
 *
 * Page allocator with custom cache attributes
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

#ifndef __ARCH_ARM_ATTRIB_ALLOC_H
#define __ARCH_ARM_ATTRIB_ALLOC_H

#include <linux/types.h>
#include <asm/page.h>

#ifdef CONFIG_ARM_ATTRIB_ALLOCATOR
struct page *arm_attrib_alloc_pages_node(int nid, gfp_t gfp,
					 unsigned int order, pgprot_t prot);

void arm_attrib_free_pages(struct page *page, unsigned int order);
#else
static inline struct page *arm_attrib_alloc_pages_node(int, gfp_t,
						       unsigned int, pgprot_t)
{
	return NULL;
}

static inline arm_attrib_free_pages(struct page *, unsigned int)
{
}
#endif

static inline struct page *arm_attrib_alloc_pages(gfp_t gfp,
					  unsigned int order, pgprot_t prot)
{
	return arm_attrib_alloc_pages_node(-1, gfp, order, prot);
}

#define arm_attrib_alloc_page(gfp, prot)	\
	arm_attrib_alloc_pages((gfp), 0, (prot))

#define arm_attrib_free_page(page)	arm_attrib_free_pages((page), 0)

#endif /* __ARCH_ARM_ATTRIB_ALLOC_H */
