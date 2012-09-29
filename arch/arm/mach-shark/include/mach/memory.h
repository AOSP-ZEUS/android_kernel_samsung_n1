/*
 * arch/arm/mach-shark/include/mach/memory.h
 *
 * by Alexander Schulz
 *
 * derived from:
 * arch/arm/mach-ebsa110/include/mach/memory.h
 * Copyright (c) 1996-1999 Russell King.
 */
#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

#include <asm/sizes.h>

/*
 * Physical DRAM offset.
 */
#define PLAT_PHYS_OFFSET     UL(0x08000000)

<<<<<<< HEAD
#define ARM_DMA_ZONE_SIZE	SZ_4M

=======
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
/*
 * Cache flushing area
 */
#define FLUSH_BASE_PHYS		0x80000000
#define FLUSH_BASE		0xdf000000

#endif
