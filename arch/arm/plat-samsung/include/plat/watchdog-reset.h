/* arch/arm/plat-s3c/include/plat/watchdog-reset.h
 *
 * Copyright (c) 2008 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * S3C2410 - System define for arch_reset() function
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

<<<<<<< HEAD
#include <plat/regs-watchdog.h>
#include <mach/map.h>

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/delay.h>
=======
#include <plat/clock.h>
#include <plat/regs-watchdog.h>
#include <mach/map.h>

#include <linux/clk.h>
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
#include <linux/err.h>
#include <linux/io.h>

static inline void arch_wdt_reset(void)
{
	printk("arch_reset: attempting watchdog reset\n");

	__raw_writel(0, S3C2410_WTCON);	  /* disable watchdog, to be safe  */

<<<<<<< HEAD
=======
	if (s3c2410_wdtclk)
		clk_enable(s3c2410_wdtclk);

>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	/* put initial values into count and data */
	__raw_writel(0x80, S3C2410_WTCNT);
	__raw_writel(0x80, S3C2410_WTDAT);

	/* set the watchdog to go and reset... */
	__raw_writel(S3C2410_WTCON_ENABLE|S3C2410_WTCON_DIV16|S3C2410_WTCON_RSTEN |
		     S3C2410_WTCON_PRESCALE(0x20), S3C2410_WTCON);

	/* wait for reset to assert... */
	mdelay(500);

	printk(KERN_ERR "Watchdog reset failed to assert reset\n");

	/* delay to allow the serial port to show the message */
	mdelay(50);
}
