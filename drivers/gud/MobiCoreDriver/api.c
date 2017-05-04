/*
 * Copyright (c) 2013 TRUSTONIC LIMITED
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>

#include "main.h"
#include "mem.h"
#include "debug.h"

int mobicore_map_vmem(struct mc_instance *instance, void *addr,
	uint32_t len, uint32_t *handle)
{
	phys_addr_t phys;
	return mc_register_wsm_mmu(instance, addr, len,
		handle, &phys);
}
EXPORT_SYMBOL(mobicore_map_vmem);

int mobicore_unmap_vmem(struct mc_instance *instance, uint32_t handle)
{
	return mc_unregister_wsm_mmu(instance, handle);
}
EXPORT_SYMBOL(mobicore_unmap_vmem);

int mobicore_free_wsm(struct mc_instance *instance, uint32_t handle)
{
	return mc_free_buffer(instance, handle);
}
EXPORT_SYMBOL(mobicore_free_wsm);


int mobicore_allocate_wsm(struct mc_instance *instance,
	unsigned long requested_size, uint32_t *handle, void **virt_kernel_addr)
{
	struct mc_buffer *buffer = NULL;

	
	if (mc_get_buffer(instance, &buffer, requested_size))
		return -EFAULT;

	*handle = buffer->handle;
	*virt_kernel_addr = buffer->addr;
	return 0;
}
EXPORT_SYMBOL(mobicore_allocate_wsm);

struct mc_instance *mobicore_open(void)
{
	struct mc_instance *instance = mc_alloc_instance();
	if (instance)
		instance->admin = true;
	return instance;
}
EXPORT_SYMBOL(mobicore_open);

int mobicore_release(struct mc_instance *instance)
{
	return mc_release_instance(instance);
}
EXPORT_SYMBOL(mobicore_release);

bool mobicore_sleep_ready(void)
{
	return mc_sleep_ready();
}
EXPORT_SYMBOL(mobicore_sleep_ready);

