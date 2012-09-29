/*
 * Copyright (C) 2011 Google, Inc.
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

#define TEGRA_EMC_NUM_REGS 46

struct tegra_emc_table {
	unsigned long rate;
	u32 regs[TEGRA_EMC_NUM_REGS];
};

<<<<<<< HEAD
int tegra_emc_set_rate(unsigned long rate);
long tegra_emc_round_rate(unsigned long rate);
void tegra_init_emc(const struct tegra_emc_table *table, int table_size);
=======
struct tegra_emc_chip {
	const char *description;
	int mem_manufacturer_id; /* LPDDR2 MR5 or -1 to ignore */
	int mem_revision_id1;    /* LPDDR2 MR6 or -1 to ignore */
	int mem_revision_id2;    /* LPDDR2 MR7 or -1 to ignore */
	int mem_pid;             /* LPDDR2 MR8 or -1 to ignore */

	const struct tegra_emc_table *table;
	int table_size;
};

#ifdef CONFIG_MACH_N1
void tegra_init_emc(const struct tegra_emc_table *table, int table_size);
#else
void tegra_init_emc(const struct tegra_emc_chip *chips, int chips_size);
#endif
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
