/*
<<<<<<< HEAD
 * Generic C implementation of atomic counter operations
=======
 * Generic C implementation of atomic counter operations. Usable on
 * UP systems only. Do not include in machine independent code.
 *
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
 * Originally implemented for MN10300.
 *
 * Copyright (C) 2007 Red Hat, Inc. All Rights Reserved.
 * Written by David Howells (dhowells@redhat.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public Licence
 * as published by the Free Software Foundation; either version
 * 2 of the Licence, or (at your option) any later version.
 */
#ifndef __ASM_GENERIC_ATOMIC_H
#define __ASM_GENERIC_ATOMIC_H

#ifdef CONFIG_SMP
<<<<<<< HEAD
#error not SMP safe
=======
/* Force people to define core atomics */
# if !defined(atomic_add_return) || !defined(atomic_sub_return) || \
     !defined(atomic_clear_mask) || !defined(atomic_set_mask)
#  error "SMP requires a little arch-specific magic"
# endif
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
#endif

/*
 * Atomic operations that C can't guarantee us.  Useful for
 * resource counting etc..
 */

#define ATOMIC_INIT(i)	{ (i) }

#ifdef __KERNEL__

/**
 * atomic_read - read atomic variable
 * @v: pointer of type atomic_t
 *
 * Atomically reads the value of @v.
 */
<<<<<<< HEAD
#define atomic_read(v)	(*(volatile int *)&(v)->counter)
=======
#ifndef atomic_read
#define atomic_read(v)	(*(volatile int *)&(v)->counter)
#endif
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

/**
 * atomic_set - set atomic variable
 * @v: pointer of type atomic_t
 * @i: required value
 *
 * Atomically sets the value of @v to @i.
 */
#define atomic_set(v, i) (((v)->counter) = (i))

#include <linux/irqflags.h>
#include <asm/system.h>

/**
 * atomic_add_return - add integer to atomic variable
 * @i: integer value to add
 * @v: pointer of type atomic_t
 *
 * Atomically adds @i to @v and returns the result
 */
<<<<<<< HEAD
=======
#ifndef atomic_add_return
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
static inline int atomic_add_return(int i, atomic_t *v)
{
	unsigned long flags;
	int temp;

	raw_local_irq_save(flags); /* Don't trace it in an irqsoff handler */
	temp = v->counter;
	temp += i;
	v->counter = temp;
	raw_local_irq_restore(flags);

	return temp;
}
<<<<<<< HEAD
=======
#endif
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

/**
 * atomic_sub_return - subtract integer from atomic variable
 * @i: integer value to subtract
 * @v: pointer of type atomic_t
 *
 * Atomically subtracts @i from @v and returns the result
 */
<<<<<<< HEAD
=======
#ifndef atomic_sub_return
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
static inline int atomic_sub_return(int i, atomic_t *v)
{
	unsigned long flags;
	int temp;

	raw_local_irq_save(flags); /* Don't trace it in an irqsoff handler */
	temp = v->counter;
	temp -= i;
	v->counter = temp;
	raw_local_irq_restore(flags);

	return temp;
}
<<<<<<< HEAD
=======
#endif
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

static inline int atomic_add_negative(int i, atomic_t *v)
{
	return atomic_add_return(i, v) < 0;
}

static inline void atomic_add(int i, atomic_t *v)
{
	atomic_add_return(i, v);
}

static inline void atomic_sub(int i, atomic_t *v)
{
	atomic_sub_return(i, v);
}

static inline void atomic_inc(atomic_t *v)
{
	atomic_add_return(1, v);
}

static inline void atomic_dec(atomic_t *v)
{
	atomic_sub_return(1, v);
}

#define atomic_dec_return(v)		atomic_sub_return(1, (v))
#define atomic_inc_return(v)		atomic_add_return(1, (v))

#define atomic_sub_and_test(i, v)	(atomic_sub_return((i), (v)) == 0)
<<<<<<< HEAD
#define atomic_dec_and_test(v)		(atomic_sub_return(1, (v)) == 0)
#define atomic_inc_and_test(v)		(atomic_add_return(1, (v)) == 0)
=======
#define atomic_dec_and_test(v)		(atomic_dec_return(v) == 0)
#define atomic_inc_and_test(v)		(atomic_inc_return(v) == 0)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

#define atomic_xchg(ptr, v)		(xchg(&(ptr)->counter, (v)))
#define atomic_cmpxchg(v, old, new)	(cmpxchg(&((v)->counter), (old), (new)))

#define cmpxchg_local(ptr, o, n)				  	       \
	((__typeof__(*(ptr)))__cmpxchg_local_generic((ptr), (unsigned long)(o),\
			(unsigned long)(n), sizeof(*(ptr))))

#define cmpxchg64_local(ptr, o, n) __cmpxchg64_local_generic((ptr), (o), (n))

<<<<<<< HEAD
static inline int atomic_add_unless(atomic_t *v, int a, int u)
=======
static inline int __atomic_add_unless(atomic_t *v, int a, int u)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
{
  int c, old;
  c = atomic_read(v);
  while (c != u && (old = atomic_cmpxchg(v, c, c + a)) != c)
    c = old;
<<<<<<< HEAD
  return c != u;
}

#define atomic_inc_not_zero(v) atomic_add_unless((v), 1, 0)

static inline void atomic_clear_mask(unsigned long mask, unsigned long *addr)
=======
  return c;
}

/**
 * atomic_clear_mask - Atomically clear bits in atomic variable
 * @mask: Mask of the bits to be cleared
 * @v: pointer of type atomic_t
 *
 * Atomically clears the bits set in @mask from @v
 */
#ifndef atomic_clear_mask
static inline void atomic_clear_mask(unsigned long mask, atomic_t *v)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
{
	unsigned long flags;

	mask = ~mask;
	raw_local_irq_save(flags); /* Don't trace it in a irqsoff handler */
<<<<<<< HEAD
	*addr &= mask;
	raw_local_irq_restore(flags);
}
=======
	v->counter &= mask;
	raw_local_irq_restore(flags);
}
#endif

/**
 * atomic_set_mask - Atomically set bits in atomic variable
 * @mask: Mask of the bits to be set
 * @v: pointer of type atomic_t
 *
 * Atomically sets the bits set in @mask in @v
 */
#ifndef atomic_set_mask
static inline void atomic_set_mask(unsigned int mask, atomic_t *v)
{
	unsigned long flags;

	raw_local_irq_save(flags); /* Don't trace it in a irqsoff handler */
	v->counter |= mask;
	raw_local_irq_restore(flags);
}
#endif
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

/* Assume that atomic operations are already serializing */
#define smp_mb__before_atomic_dec()	barrier()
#define smp_mb__after_atomic_dec()	barrier()
#define smp_mb__before_atomic_inc()	barrier()
#define smp_mb__after_atomic_inc()	barrier()

<<<<<<< HEAD
#include <asm-generic/atomic-long.h>

=======
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
#endif /* __KERNEL__ */
#endif /* __ASM_GENERIC_ATOMIC_H */
