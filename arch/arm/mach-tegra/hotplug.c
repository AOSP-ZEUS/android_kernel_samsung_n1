/*
<<<<<<< HEAD
 *  linux/arch/arm/mach-realview/hotplug.c
 *
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
=======
 *  arch/arm/mach-tegra/hotplug.c
 *
 *  Copyright (C) 2010-2011 NVIDIA Corporation
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
<<<<<<< HEAD
#include <linux/errno.h>
#include <linux/smp.h>

#include <asm/cacheflush.h>

static inline void cpu_enter_lowpower(void)
{
	unsigned int v;

	flush_cache_all();
	asm volatile(
	"	mcr	p15, 0, %1, c7, c5, 0\n"
	"	mcr	p15, 0, %1, c7, c10, 4\n"
	/*
	 * Turn off coherency
	 */
	"	mrc	p15, 0, %0, c1, c0, 1\n"
	"	bic	%0, %0, #0x20\n"
	"	mcr	p15, 0, %0, c1, c0, 1\n"
	"	mrc	p15, 0, %0, c1, c0, 0\n"
	"	bic	%0, %0, %2\n"
	"	mcr	p15, 0, %0, c1, c0, 0\n"
	  : "=&r" (v)
	  : "r" (0), "Ir" (CR_C)
	  : "cc");
}

static inline void cpu_leave_lowpower(void)
{
	unsigned int v;

	asm volatile(
	"mrc	p15, 0, %0, c1, c0, 0\n"
	"	orr	%0, %0, %1\n"
	"	mcr	p15, 0, %0, c1, c0, 0\n"
	"	mrc	p15, 0, %0, c1, c0, 1\n"
	"	orr	%0, %0, #0x20\n"
	"	mcr	p15, 0, %0, c1, c0, 1\n"
	  : "=&r" (v)
	  : "Ir" (CR_C)
	  : "cc");
}

static inline void platform_do_lowpower(unsigned int cpu, int *spurious)
{
	/*
	 * there is no power-control hardware on this platform, so all
	 * we can do is put the core into WFI; this is safe as the calling
	 * code will have already disabled interrupts
	 */
	for (;;) {
		/*
		 * here's the WFI
		 */
		asm(".word	0xe320f003\n"
		    :
		    :
		    : "memory", "cc");

		/*if (pen_release == cpu) {*/
			/*
			 * OK, proper wakeup, we're done
			 */
			break;
		/*}*/

		/*
		 * Getting here, means that we have come out of WFI without
		 * having been woken up - this shouldn't happen
		 *
		 * Just note it happening - when we're woken, we can report
		 * its occurrence.
		 */
		(*spurious)++;
	}
}

int platform_cpu_kill(unsigned int cpu)
{
	return 1;
}

/*
 * platform-specific code to shutdown a CPU
 *
 * Called with IRQs disabled
 */
void platform_cpu_die(unsigned int cpu)
{
	int spurious = 0;

	/*
	 * we're ready for shutdown now, so do it
	 */
	cpu_enter_lowpower();
	platform_do_lowpower(cpu, &spurious);

	/*
	 * bring this CPU back into the world of cache
	 * coherency, and then restore interrupts
	 */
	cpu_leave_lowpower();

	if (spurious)
		pr_warn("CPU%u: %u spurious wakeup calls\n", cpu, spurious);
=======
#include <linux/io.h>
#include <linux/smp.h>

#include <asm/cpu_pm.h>
#include <asm/cacheflush.h>

#include <mach/iomap.h>

#include "gic.h"
#include "sleep.h"

#define CPU_CLOCK(cpu) (0x1<<(8+cpu))

#define CLK_RST_CONTROLLER_CLK_CPU_CMPLX \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x4c)
#define CLK_RST_CONTROLLER_RST_CPU_CMPLX_SET \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x340)
#define CLK_RST_CONTROLLER_RST_CPU_CMPLX_CLR \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x344)

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
/* For Tegra2 use the software-written value of the reset register for status.*/
#define CLK_RST_CONTROLLER_CPU_CMPLX_STATUS CLK_RST_CONTROLLER_RST_CPU_CMPLX_SET
#else
#define CLK_RST_CONTROLLER_CPU_CMPLX_STATUS \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x470)
#endif

int platform_cpu_kill(unsigned int cpu)
{
	unsigned int reg;

	do {
		reg = readl(CLK_RST_CONTROLLER_CPU_CMPLX_STATUS);
		cpu_relax();
	} while (!(reg & (1<<cpu)));

	reg = readl(CLK_RST_CONTROLLER_CLK_CPU_CMPLX);
	writel(reg | CPU_CLOCK(cpu), CLK_RST_CONTROLLER_CLK_CPU_CMPLX);

	return 1;
}

void platform_cpu_die(unsigned int cpu)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	/* Flush the L1 data cache. */
	flush_cache_all();

	/* Place the current CPU in reset. */
	tegra2_hotplug_shutdown();
#else
	/* Disable GIC CPU interface for this CPU. */
	tegra_gic_cpu_disable();

	/* Tegra3 enters LPx states via WFI - do not propagate legacy IRQs
	   to CPU core to avoid fall through WFI; then GIC output will be
	   enabled, however at this time - CPU is dying - no interrupt should
	   have affinity to this CPU. */
	tegra_gic_pass_through_disable();

	/* Flush the L1 data cache. */
	flush_cache_all();

	/* Shut down the current CPU. */
	tegra3_hotplug_shutdown();
#endif

	/* Should never return here. */
	BUG();
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
}

int platform_cpu_disable(unsigned int cpu)
{
	/*
	 * we don't allow CPU 0 to be shutdown (it is still too special
	 * e.g. clock tick interrupts)
	 */
	return cpu == 0 ? -EPERM : 0;
}
