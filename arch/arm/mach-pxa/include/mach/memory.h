/*
 *  arch/arm/mach-pxa/include/mach/memory.h
 *
 * Author:	Nicolas Pitre
 * Copyright:	(C) 2001 MontaVista Software Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/*
 * Physical DRAM offset.
 */
#define PLAT_PHYS_OFFSET	UL(0xa0000000)

<<<<<<< HEAD
#if defined(CONFIG_MACH_ARMCORE) && defined(CONFIG_PCI)
#define ARM_DMA_ZONE_SIZE	SZ_64M
#endif

=======
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
#endif
