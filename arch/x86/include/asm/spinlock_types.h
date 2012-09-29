#ifndef _ASM_X86_SPINLOCK_TYPES_H
#define _ASM_X86_SPINLOCK_TYPES_H

#ifndef __LINUX_SPINLOCK_TYPES_H
# error "please don't include this file directly"
#endif

typedef struct arch_spinlock {
	unsigned int slock;
} arch_spinlock_t;

#define __ARCH_SPIN_LOCK_UNLOCKED	{ 0 }

<<<<<<< HEAD
typedef struct {
	unsigned int lock;
} arch_rwlock_t;

#define __ARCH_RW_LOCK_UNLOCKED		{ RW_LOCK_BIAS }
=======
#include <asm/rwlock.h>
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

#endif /* _ASM_X86_SPINLOCK_TYPES_H */
