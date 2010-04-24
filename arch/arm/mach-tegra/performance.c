/*
 * arch/arm/mach-tegra/performance.c
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
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/string.h>

#include <mach/performance.h>

/* Locking:
	Always lock in this order:
	performance_provider_list_lock -> provider->lock
	OR
	request->lock -> provider->lock

	The only exception is on peformance_provider_release, which needs to
	lock all the provider lock followed by each request lock.
 */

static LIST_HEAD(performance_provider_list);
static DEFINE_SPINLOCK(performance_provider_list_lock);

int performance_provider_level_min(struct performance_provider *provider)
{
	struct performance_request *request;
	int min = provider->max;
	list_for_each_entry(request, &provider->requests, node) {
		if (request->enabled && request->level < min)
			min = request->level;
	}
	if (min < provider->max)
		min = provider->max;
	return min;
}

int performance_provider_level_max(struct performance_provider *provider)
{
	struct performance_request *request;
	int max = provider->min;
	list_for_each_entry(request, &provider->requests, node) {
		if (request->enabled && request->level > max)
			max = request->level;
	}
	if (max > provider->max)
		max = provider->max;
	return max;
}

int performance_provider_level_sum(struct performance_provider *provider)
{
	struct performance_request *request;
	int sum = 0;
	list_for_each_entry(request, &provider->requests, node) {
		if (request->enabled)
			sum += request->level;
	}
	return sum;
}

/* Must be called with provider->lock held */
static int performance_provider_update(struct performance_provider *provider)
{
	int level = provider->ops->merge(provider);

	if (level > provider->max)
		level = provider->max;
	if (level < provider->min)
		level = provider->min;

	if (level == provider->level)
		return 0;

	provider->level = level;
	return provider->ops->set(provider, level);
}

static struct performance_provider *get_provider_by_name(const char *name)
{
	struct performance_provider *provider;
	list_for_each_entry(provider, &performance_provider_list, node) {
		if (!strcmp(name, provider->name))
			return provider;
	}
	return ERR_PTR(EINVAL);
}

int performance_provider_register(struct performance_provider *provider)
{
	unsigned long flags;
	spin_lock_init(&provider->lock);
	INIT_LIST_HEAD(&provider->requests);
	spin_lock_irqsave(&performance_provider_list_lock, flags);
	list_add_tail(&provider->node, &performance_provider_list);
	spin_unlock_irqrestore(&performance_provider_list_lock, flags);
	return 0;
}

static int try_release_requests(struct performance_provider *provider)
{
	unsigned long flags;
	struct performance_request *request;
	struct performance_request *tmp;
	int empty;

	/* Since we are taking the provider and requests locks backwards
	   from the rest of the lockers, we have to use trylock on the
	   request lock and just skip the ones that we can't lock */
	spin_lock_irqsave(&provider->lock, flags);
	list_for_each_entry_safe(request, tmp, &provider->requests, node) {
		if (spin_trylock(&request->lock)) {
			request->provider = NULL;
			spin_unlock(&provider->lock);
		}
	}
	empty = list_empty(&provider->requests);
	spin_unlock_irqrestore(&provider->lock, flags);

	if (!empty)
		return -EAGAIN;

	return 0;
}

void performance_provider_release(struct performance_provider *provider)
{
	unsigned long flags;

	spin_lock_irqsave(&performance_provider_list_lock, flags);
	list_del(&provider->node);
	spin_unlock_irqrestore(&performance_provider_list_lock, flags);

	/* With the provider out of the provider list,
	   performance_request_register cannot find this provider,
	   and the provider->requests lists cannot grow.

	   try_release_request will release as many of the requests as it can,
	   due to locking issues.  Retry until the list is empty.
	 */
	while (!try_release_requests(provider))
		yield();
}

int performance_request_register(struct performance_request *request,
	const char *provider_name)
{
	struct performance_provider *provider;
	unsigned long flags;

	spin_lock_irqsave(&performance_provider_list_lock, flags);
	provider = get_provider_by_name(provider_name);
	if (IS_ERR(provider)) {
		spin_unlock_irqrestore(&performance_provider_list_lock, flags);
		return PTR_ERR(provider);
	}
	spin_lock(&provider->lock);

	spin_lock_init(&request->lock);
	request->enabled = 0;
	request->level = provider->level;
	request->provider = provider;

	list_add_tail(&request->node, &provider->requests);
	spin_unlock(&provider->lock);
	spin_unlock_irqrestore(&performance_provider_list_lock, flags);

	return 0;
}

void performance_request_release(struct performance_request *request)
{
	struct performance_provider *provider;
	unsigned long flags;

	spin_lock_irqsave(&request->lock, flags);

	provider = request->provider;
	if (!provider) {
		spin_unlock_irqrestore(&request->lock, flags);
		return;
	}

	request->provider = NULL;

	spin_lock(&provider->lock);
	list_del(&request->node);

	spin_unlock(&provider->lock);
	spin_unlock_irqrestore(&request->lock, flags);
}

int performance_request_set(struct performance_request *request, int level)
{
	int ret;
	struct performance_provider *provider;
	unsigned long flags;

	spin_lock_irqsave(&request->lock, flags);

	provider = request->provider;
	if (!provider) {
		spin_unlock_irqrestore(&request->lock, flags);
		return -EINVAL;
	}

	spin_lock(&provider->lock);

	request->level = level;
	request->enabled = 1;

	ret = performance_provider_update(provider);

	spin_unlock(&provider->lock);
	spin_unlock_irqrestore(&request->lock, flags);
	return ret;
}

int performance_request_clear(struct performance_request *request)
{
	int ret;
	struct performance_provider *provider;
	unsigned long flags;

	spin_lock_irqsave(&request->lock, flags);

	provider = request->provider;
	if (!provider) {
		spin_unlock_irqrestore(&request->lock, flags);
		return -EINVAL;
	}

	spin_lock(&provider->lock);

	request->enabled = 0;

	ret = performance_provider_update(provider);

	spin_unlock(&provider->lock);
	spin_unlock_irqrestore(&request->lock, flags);
	return ret;
}
