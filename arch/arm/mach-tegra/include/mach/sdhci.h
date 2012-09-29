/*
 * include/asm-arm/arch-tegra/include/mach/sdhci.h
 *
 * Copyright (C) 2009 Palm, Inc.
 * Author: Yvonne Yip <y@palm.com>
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
#ifndef __ASM_ARM_ARCH_TEGRA_SDHCI_H
#define __ASM_ARM_ARCH_TEGRA_SDHCI_H

#include <linux/mmc/host.h>
<<<<<<< HEAD
=======
#include <asm/mach/mmc.h>
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

struct tegra_sdhci_platform_data {
	int cd_gpio;
	int wp_gpio;
	int power_gpio;
	int is_8bit;
	int pm_flags;
<<<<<<< HEAD
=======
	int pm_caps;
	unsigned int max_clk_limit;
	unsigned int tap_delay;
	struct mmc_platform_data mmc_data;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
};

#endif
