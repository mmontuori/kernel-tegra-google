/*
 * drivers/video/tegra/nvmap_handle.c
 *
 * Handle allocation and freeing routines for nvmap
 *
 * Copyright (c) 2009-2010, NVIDIA Corporation.
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

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/rbtree.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include <asm/attrib_alloc.h>
#include <asm/pgtable.h>

#include <mach/iovmm.h>
#include <mach/nvmap.h>

#include "nvmap.h"
#include "nvmap_mru.h"

#define NVMAP_SECURE_HEAPS	(NVMAP_HEAP_CARVEOUT_IRAM | NVMAP_HEAP_IOVMM)
#define PAGELIST_VMALLOC_MIN	(PAGE_SIZE * 2)
#define GFP_NVMAP		(GFP_KERNEL | __GFP_HIGHMEM | __GFP_NOWARN)

static inline void *altalloc(size_t len)
{
	if (len >= PAGELIST_VMALLOC_MIN)
		return vmalloc(len);
	else
		return kmalloc(len, GFP_KERNEL);
}

static inline void altfree(void *ptr, size_t len)
{
	if (!ptr)
		return;

	if (len >= PAGELIST_VMALLOC_MIN)
		vfree(ptr);
	else
		kfree(ptr);
}

static unsigned int heap_policy(unsigned int heaps, int nr_page)
{
	const unsigned int* policy;
	static const unsigned int big_policy[] = {
		NVMAP_HEAP_IOVMM,
		NVMAP_HEAP_CARVEOUT_MASK,
		NVMAP_HEAP_SYSMEM,
		0
	};
	static const unsigned int small_policy[] = {
		NVMAP_HEAP_SYSMEM,
		NVMAP_HEAP_CARVEOUT_MASK,
		NVMAP_HEAP_IOVMM,
		0
	};

	if (nr_page <= 4)
		policy = small_policy;
	else
		policy = big_policy;

	while (*policy) {
		unsigned int h = (*policy & heaps);
		if (h)
			return h;
		policy++;
	}
	return 0;
};

void _nvmap_handle_free(struct nvmap_handle *h)
{
	struct nvmap_client *c = h->owner;
	unsigned int i, nr_page;

	if (nvmap_handle_remove(c->dev, h) != 0)
		return;

	if (!h->alloc)
		goto out;

	if (!h->heap_pgalloc) {
		nvmap_heap_free(h->carveout);
		goto out;
	}

	nr_page = ALIGN(h->size, PAGE_SIZE) >> PAGE_SHIFT;

	BUG_ON(h->size & ~PAGE_MASK);
	BUG_ON(!h->pgalloc.pages);

	nvmap_mru_remove(c->share, h);
	if (h->pgalloc.area)
		tegra_iovmm_free_vm(h->pgalloc.area);

	for (i = 0; i < nr_page; i++)
		arm_attrib_free_page(h->pgalloc.pages[i]);

	altfree(h->pgalloc.pages, nr_page * sizeof(struct page *));

out:
	h->poison = 0xa5a5a5a5;
	kfree(h);
	nvmap_client_put(c);
}

static int handle_page_alloc(struct nvmap_client *client,
			     struct nvmap_handle *h, bool contiguous)
{
	size_t size = ALIGN(h->size, PAGE_SIZE);
	unsigned int nr_page = size >> PAGE_SHIFT;
	pgprot_t prot;
	unsigned int i = 0;
	struct page **pages;

	pages = altalloc(nr_page * sizeof(*pages));
	if (!pages)
		return -ENOMEM;

	prot = nvmap_pgprot(h, pgprot_kernel);

	if (nr_page == 1)
		contiguous = true;

	if (contiguous) {
		size_t order = get_order(size);
		struct page *page;

		BUG_ON(nr_page > (1 << order));

		page = arm_attrib_alloc_pages(GFP_NVMAP, order, prot);
		if (!page)
			goto fail;
		split_page(page, order);

		for (i = 0; i < nr_page; i++)
			pages[i] = nth_page(page, i);

		for (; i < (1 << order); i++)
			arm_attrib_free_page(nth_page(page, i));
	} else {
		for (i = 0; i < nr_page; i++) {
			pages[i] = arm_attrib_alloc_page(GFP_NVMAP, prot);
			if (!pages[i])
				goto fail;
		}
	}

	h->pgalloc.area = NULL;
#ifndef CONFIG_NVMAP_RECLAIM_UNPINNED_VM
	if (!contiguous) {
		h->pgalloc.area = tegra_iovmm_create_vm(client->share->iovmm,
							NULL, size, prot);
		if (!h->pgalloc.area)
			goto fail;
		h->pgalloc.dirty = true;
	}
#endif


	h->size = size;
	h->pgalloc.pages = pages;
	h->pgalloc.contig = contiguous;
	INIT_LIST_HEAD(&h->pgalloc.mru_list);
	return 0;

fail:
	while (i--)
		arm_attrib_free_page(pages[i]);
	altfree(pages, nr_page * sizeof(*pages));
	return -ENOMEM;
}

static unsigned int alloc_handle(struct nvmap_client *client, size_t align,
				 struct nvmap_handle *h, unsigned int type)
{
	if (type & NVMAP_HEAP_CARVEOUT_MASK) {
		struct nvmap_heap_block *b;
		unsigned int heap = 1 << __fls(type);
		type &= heap;
		b = nvmap_carveout_alloc(client, h->size, align,
					 heap, h->flags);
		if (b) {
			h->carveout = b;
			h->heap_pgalloc = false;
			h->alloc = true;
		}
	} else if (type & NVMAP_HEAP_IOVMM) {
		size_t reserved = ALIGN(h->size, PAGE_SIZE);
		int commit;
		int ret;

		BUG_ON(align > PAGE_SIZE);
		type &= NVMAP_HEAP_IOVMM;

		/* increment the committed IOVM space prior to allocation
		 * to avoid race conditions with other threads simultaneously
		 * allocating. */
		commit = atomic_add_return(reserved, &client->iovm_commit);

		if (commit < client->iovm_limit)
			ret = handle_page_alloc(client, h, false);
		else
			ret = -ENOMEM;

		if (!ret) {
			h->heap_pgalloc = true;
			h->alloc = true;
		} else
			atomic_sub(reserved, &client->iovm_commit);

	} else if (type & NVMAP_HEAP_SYSMEM) {
		type &= NVMAP_HEAP_SYSMEM;
		if (handle_page_alloc(client, h, true) == 0) {
			BUG_ON(!h->pgalloc.contig);
			h->heap_pgalloc = true;
			h->alloc = true;
		}
	}
	return type;
}

int nvmap_alloc_handle_id(struct nvmap_client *client,
			  unsigned long id, unsigned int heap_mask,
			  size_t align, unsigned int flags)
{
	struct nvmap_handle *h = NULL;
	int nr_page;
	int err = -ENOMEM;

	align = max_t(size_t, align, L1_CACHE_BYTES);

	/* can't do greater than page size alignment with page alloc */
	if (align > PAGE_SIZE)
		heap_mask &= NVMAP_HEAP_CARVEOUT_MASK;

	h = nvmap_get_handle_id(client, id);

	if (!h)
		return -EINVAL;

	if (h->alloc)
		goto out;

	nr_page = ((h->size + PAGE_SIZE - 1) >> PAGE_SHIFT);
	h->secure = !!(flags & NVMAP_HANDLE_SECURE);
	h->flags = (flags & NVMAP_HANDLE_CACHE_FLAG);

	/* secure allocations can only be served from secure heaps */
	if (h->secure)
		heap_mask &= NVMAP_SECURE_HEAPS;

	if (!heap_mask) {
		err = -EINVAL;
		goto out;
	}

	while (heap_mask && !h->alloc) {
		unsigned int heap_type;
		heap_type = heap_policy(heap_mask, nr_page);
		if (!heap_type)
			break;
		heap_mask &= ~alloc_handle(client, align, h, heap_type);
	}

out:
	err = (h->alloc) ? 0 : err;
	nvmap_handle_put(h);
	return err;
}

void nvmap_free_handle_id(struct nvmap_client *client, unsigned long id)
{
	struct nvmap_handle_ref *r;
	struct nvmap_handle *h;
	int pins;

	nvmap_ref_lock(client);

	r = _nvmap_validate_ref_locked(client, id);
	if (!r) {
		nvmap_ref_unlock(client);
		return;
	}

	BUG_ON(!r->h);
	h = r->h;

	if (atomic_dec_return(&r->refs)) {
		nvmap_ref_unlock(client);
		goto out;
	}

	smp_rmb();
	pins = atomic_read(&r->pin);
	rb_erase(&r->node, &client->handle_refs);

	if (h->alloc && h->heap_pgalloc && !h->pgalloc.contig)
		atomic_sub(h->size, &client->iovm_commit);

	nvmap_ref_unlock(client);

	if (pins)
		nvmap_err(client, "%s freeing pinned handle %p\n",
			  current->group_leader->comm, h);

	while (pins--)
		nvmap_unpin_handles(client, &r->h, 1);

	kfree(r);

out:
	BUG_ON(!atomic_read(&h->ref));
	nvmap_handle_put(h);
}

static void add_handle_ref(struct nvmap_client *client,
			   struct nvmap_handle_ref *r)
{
	struct rb_node **p, *parent = NULL;

	nvmap_ref_lock(client);
	p = &client->handle_refs.rb_node;
	while (*p) {
		struct nvmap_handle_ref *l;
		parent = *p;
		l = rb_entry(parent, struct nvmap_handle_ref, node);
		if (r->h > l->h)
			p = &parent->rb_right;
		else
			p = &parent->rb_left;
	}
	rb_link_node(&r->node, parent, p);
	rb_insert_color(&r->node, &client->handle_refs);
	nvmap_ref_unlock(client);
}

struct nvmap_handle_ref *nvmap_create_handle(struct nvmap_client *client,
					     size_t size)
{
	struct nvmap_handle *h;
	struct nvmap_handle_ref *r = NULL;

	if (!size)
		return ERR_PTR(-EINVAL);

	h = kzalloc(sizeof(*h), GFP_KERNEL);
	if (!h)
		return ERR_PTR(-ENOMEM);

	r = kzalloc(sizeof(*r), GFP_KERNEL);
	if (!r) {
		kfree(h);
		return ERR_PTR(-ENOMEM);
	}

	atomic_set(&h->ref, 1);
	atomic_set(&h->pin, 0);
	h->owner = nvmap_client_get(client);
	BUG_ON(!h->owner);
	h->size = h->orig_size = size;
	h->flags = NVMAP_HANDLE_WRITE_COMBINE;
	h->poison = NVMAP_POISON;

	nvmap_handle_add(client->dev, h);

	atomic_set(&r->refs, 1);
	r->h = h;
	atomic_set(&r->pin, 0);
	add_handle_ref(client, r);
	return r;
}

struct nvmap_handle_ref *nvmap_duplicate_handle_id(struct nvmap_client *client,
						   unsigned long id)
{
	struct nvmap_handle_ref *r = NULL;
	struct nvmap_handle *h = NULL;

	BUG_ON(!client || client->dev != nvmap_dev);
	/* on success, the reference count for the handle should be
	 * incremented, so the success paths will not call nvmap_handle_put */
	h = nvmap_validate_get(client, id);

	if (!h) {
		nvmap_debug(client, "%s duplicate handle failed\n",
			    current->group_leader->comm);
		return ERR_PTR(-EPERM);
	}

	if (!h->alloc) {
		nvmap_err(client, "%s duplicating unallocated handle\n",
			  current->group_leader->comm);
		nvmap_handle_put(h);
		return ERR_PTR(-EINVAL);
	}

	nvmap_ref_lock(client);
	r = _nvmap_validate_ref_locked(client, (unsigned long)h);

	if (r) {
		/* handle already duplicated in client; just increment
		 * the reference count rather than re-duplicating it */
		atomic_inc(&r->refs);
		nvmap_ref_unlock(client);
		return r;
	}

	nvmap_ref_unlock(client);

	/* verify that adding this handle to the process' access list
	 * won't exceed the IOVM limit */
	if (h->heap_pgalloc && !h->pgalloc.contig && !client->super) {
		int oc;
		oc = atomic_add_return(h->size, &client->iovm_commit);
		if (oc > client->iovm_limit) {
			atomic_sub(h->size, &client->iovm_commit);
			nvmap_handle_put(h);
			nvmap_err(client, "duplicating %p in %s over-commits"
				  " IOVMM space\n", (void *)id,
				  current->group_leader->comm);
			return ERR_PTR(-ENOMEM);
		}
	}

	r = kzalloc(sizeof(*r), GFP_KERNEL);
	if (!r) {
		nvmap_handle_put(h);
		return ERR_PTR(-ENOMEM);
	}

	atomic_set(&r->refs, 1);
	r->h = h;
	atomic_set(&r->pin, 0);
	add_handle_ref(client, r);
	return r;
}
