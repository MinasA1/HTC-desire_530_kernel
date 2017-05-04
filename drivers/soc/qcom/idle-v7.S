/*
 * Idle processing for ARMv7-based Qualcomm SoCs.
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007-2009, 2011-2014 The Linux Foundation. All rights reserved.
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
#include <linux/threads.h>
#include <asm/assembler.h>

#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
#include <htc_debug/mnemosyne/htc_footprint.h>
#endif

	.arm
ENTRY(msm_pm_boot_entry)
THUMB(	adr	r9, BSYM(2f)	)	/* Kernel is always entered in ARM. */
THUMB(	bx	r9		)	/* If this is a Thumb-2 kernel, */
THUMB(	.thumb			)	/* switch to Thumb now.		*/
THUMB(2:			)

#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
	MPIDR2INDEX	r0, r2
	VIRT2PHYS	r4, mnemosyne_phys, msm_pm_boot_entry, r2
	ldr		r4, [r4]					/* Keep mnemosyne_phys in r4 */

	cmp	r4, #0
	beq	1f							/* if base is NULL, skip it. */

	MNEMOSYNE_GET_ADDR_I r2, r4, kernel_footprint_cpu, r0

	mov	r3, #2							/* write footprint 2 */
	strb	r3, [r2]
	dsb	sy							/* ensure data are written. */
1:
#endif

	mrc     p15, 0, r0, c0, c0, 5    /* MPIDR                          */
	bic     r0, #0xff000000         /* what CPU am I */
	bic     r0, #0x00ff0000         /* clear bits 31-16 */

	adr	r3, 3f
	ldr	r1, [r3]
	sub	r3, r1, r3		/* r3 holds the virt to phy offset */
	ldr	r1, =msm_pc_debug_counters_phys /* phys addr for IMEM reg */
	sub	r1, r1, r3			/* translate virt to phys */
	ldr	r1,[r1]
	cmp	r1, #0
	beq	skip_pc_debug3
	mov     r2, r0, lsr #8		/* get cluster num */
	add 	r1, r1, r2, lsl #6	/* get offset of cluster */
	and	r2, r0, #0xff
	add	r1, r1, r2, lsl #4	/* get offset for the cpu */
	add	r1, #4			/* warmboot entry counter */
	ldr	r2, [r1]
	add	r2, #1
	str	r2, [r1]
skip_pc_debug3:
	ldr     r1, =msm_pm_boot_vector
	sub	r1, r1, r3		/* translate virt to phys */
	mov     r2, r0, lsr #8		/* get cluster num */
	add	r1, r1, r2, lsl #4	/* Get offset for the cluster */
	and	r0, r0, #0xff		/* cpu id */
	add	r1, r1, r0, lsl #2	/* Get offset for the cpu */

#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
	MPIDR2INDEX	r0, r2
	VIRT2PHYS	r4, mnemosyne_phys, msm_pm_boot_entry, r2
	ldr		r4, [r4]					/* Keep mnemosyne_phys in r4 */

	cmp	r4, #0
	beq	1f							/* if base is NULL, skip it. */

	MNEMOSYNE_GET_ADDR_I r2, r4, kernel_footprint_cpu, r0

	mov	r3, #3							/* write footprint 3 */
	strb	r3, [r2]
	dsb	sy							/* ensure data are written. */

	MNEMOSYNE_SET_I r1, r4, cpu_reset_vector_address, r0, r2

	ldr	r3, [r1]
	MNEMOSYNE_SET_I r3, r4, cpu_reset_vector_address_value, r0, r2
1:
#endif

	ldr     pc, [r1]                 /* jump                    */
ENDPROC(msm_pm_boot_entry)

3:	.long	.

	.data

	.globl msm_pm_boot_vector
msm_pm_boot_vector:
	.space  4 * 4 * 4

	.globl msm_pc_debug_counters_phys
msm_pc_debug_counters_phys:
	.long 0x0
