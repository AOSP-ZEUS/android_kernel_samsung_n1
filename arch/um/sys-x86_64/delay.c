/*
<<<<<<< HEAD
 * Copyright 2003 PathScale, Inc.
 * Copied from arch/x86_64
 *
 * Licensed under the GPL
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <asm/processor.h>
=======
 * Copyright (C) 2011 Richard Weinberger <richrd@nod.at>
 * Mostly copied from arch/x86/lib/delay.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
#include <asm/param.h>

void __delay(unsigned long loops)
{
<<<<<<< HEAD
	unsigned long i;

        for(i = 0; i < loops; i++)
                cpu_relax();
}

void __udelay(unsigned long usecs)
{
	unsigned long i, n;

	n = (loops_per_jiffy * HZ * usecs) / MILLION;
        for(i=0;i<n;i++)
                cpu_relax();
}

EXPORT_SYMBOL(__udelay);
=======
	asm volatile(
		"test %0,%0\n"
		"jz 3f\n"
		"jmp 1f\n"

		".align 16\n"
		"1: jmp 2f\n"

		".align 16\n"
		"2: dec %0\n"
		" jnz 2b\n"
		"3: dec %0\n"

		: /* we don't need output */
		: "a" (loops)
	);
}
EXPORT_SYMBOL(__delay);

inline void __const_udelay(unsigned long xloops)
{
	int d0;

	xloops *= 4;
	asm("mull %%edx"
		: "=d" (xloops), "=&a" (d0)
		: "1" (xloops), "0"
		(loops_per_jiffy * (HZ/4)));

	__delay(++xloops);
}
EXPORT_SYMBOL(__const_udelay);

void __udelay(unsigned long usecs)
{
	__const_udelay(usecs * 0x000010c7); /* 2**32 / 1000000 (rounded up) */
}
EXPORT_SYMBOL(__udelay);

void __ndelay(unsigned long nsecs)
{
	__const_udelay(nsecs * 0x00005); /* 2**32 / 1000000000 (rounded up) */
}
EXPORT_SYMBOL(__ndelay);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
