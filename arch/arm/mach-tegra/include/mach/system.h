/*
 * arch/arm/mach-tegra/include/mach/system.h
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@google.com>
 *	Erik Gilling <konkers@google.com>
 *
<<<<<<< HEAD
=======
 * Copyright (C) 2010-2011 NVIDIA Corporation
 *
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
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

#ifndef __MACH_TEGRA_SYSTEM_H
#define __MACH_TEGRA_SYSTEM_H

<<<<<<< HEAD
#include <mach/hardware.h>
=======
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
#include <mach/iomap.h>

extern void (*arch_reset)(char mode, const char *cmd);

static inline void arch_idle(void)
{
}

#endif
