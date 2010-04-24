/*
 * arch/arm/mach-tegra/include/mach/performance.h
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

#ifndef _MACH_TEGRA_PERFORMANCE_H_
#define _MACH_TEGRA_PERFORMANCE_H_

#include <linux/mutex.h>
#include <linux/spinlock.h>

struct performance_provider {
	/* To be filled out by performance provider driver */
	const char *name;
	struct performance_provider_ops *ops;
	int level;
	int min;
	int max;
	void *data;

	/* Private to performance provider subsystem */
	struct spinlock lock;
	struct list_head node;
	struct list_head requests;
};

struct performance_request {
	const char *name;

	/* Private to performance provider subsystem */
	struct list_head node;
	struct performance_provider *provider;
	struct spinlock lock;
	int level;
	int enabled : 1;
};

struct performance_provider_ops {
	int (*merge)(struct performance_provider *provider);
	int (*set)(struct performance_provider *provider, int level);
};

int performance_provider_level_min(struct performance_provider *provider);
int performance_provider_level_max(struct performance_provider *provider);
int performance_provider_level_sum(struct performance_provider *provider);
int performance_provider_register(struct performance_provider *provider);
void performance_provider_release(struct performance_provider *provider);
int performance_request_register(struct performance_request *request,
		const char *provider_name);
void performance_request_release(struct performance_request *request);
int performance_request_set(struct performance_request *request, int level);
int performance_request_clear(struct performance_request *request);

#endif
