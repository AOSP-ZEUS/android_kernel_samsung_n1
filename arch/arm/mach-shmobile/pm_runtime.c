/*
 * arch/arm/mach-shmobile/pm_runtime.c
 *
 * Runtime PM support code for SuperH Mobile ARM
 *
 *  Copyright (C) 2009-2010 Magnus Damm
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>
<<<<<<< HEAD
=======
#include <linux/pm_domain.h>
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/sh_clk.h>
#include <linux/bitmap.h>
#include <linux/slab.h>

#ifdef CONFIG_PM_RUNTIME

static int default_platform_runtime_idle(struct device *dev)
{
	/* suspend synchronously to disable clocks immediately */
	return pm_runtime_suspend(dev);
}

<<<<<<< HEAD
static struct dev_power_domain default_power_domain = {
	.ops = {
		.runtime_suspend = pm_runtime_clk_suspend,
		.runtime_resume = pm_runtime_clk_resume,
=======
static struct dev_pm_domain default_pm_domain = {
	.ops = {
		.runtime_suspend = pm_clk_suspend,
		.runtime_resume = pm_clk_resume,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		.runtime_idle = default_platform_runtime_idle,
		USE_PLATFORM_PM_SLEEP_OPS
	},
};

<<<<<<< HEAD
#define DEFAULT_PWR_DOMAIN_PTR	(&default_power_domain)

#else

#define DEFAULT_PWR_DOMAIN_PTR	NULL
=======
#define DEFAULT_PM_DOMAIN_PTR	(&default_pm_domain)

#else

#define DEFAULT_PM_DOMAIN_PTR	NULL
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

#endif /* CONFIG_PM_RUNTIME */

static struct pm_clk_notifier_block platform_bus_notifier = {
<<<<<<< HEAD
	.pwr_domain = DEFAULT_PWR_DOMAIN_PTR,
=======
	.pm_domain = DEFAULT_PM_DOMAIN_PTR,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	.con_ids = { NULL, },
};

static int __init sh_pm_runtime_init(void)
{
<<<<<<< HEAD
	pm_runtime_clk_add_notifier(&platform_bus_type, &platform_bus_notifier);
	return 0;
}
core_initcall(sh_pm_runtime_init);
=======
	pm_clk_add_notifier(&platform_bus_type, &platform_bus_notifier);
	return 0;
}
core_initcall(sh_pm_runtime_init);

static int __init sh_pm_runtime_late_init(void)
{
	pm_genpd_poweroff_unused();
	return 0;
}
late_initcall(sh_pm_runtime_late_init);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
