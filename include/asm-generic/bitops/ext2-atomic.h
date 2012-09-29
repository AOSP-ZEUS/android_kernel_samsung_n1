#ifndef _ASM_GENERIC_BITOPS_EXT2_ATOMIC_H_
#define _ASM_GENERIC_BITOPS_EXT2_ATOMIC_H_

<<<<<<< HEAD
=======
/*
 * Spinlock based version of ext2 atomic bitops
 */

>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
#define ext2_set_bit_atomic(lock, nr, addr)		\
	({						\
		int ret;				\
		spin_lock(lock);			\
		ret = __test_and_set_bit_le(nr, addr);	\
		spin_unlock(lock);			\
		ret;					\
	})

#define ext2_clear_bit_atomic(lock, nr, addr)		\
	({						\
		int ret;				\
		spin_lock(lock);			\
		ret = __test_and_clear_bit_le(nr, addr);	\
		spin_unlock(lock);			\
		ret;					\
	})

#endif /* _ASM_GENERIC_BITOPS_EXT2_ATOMIC_H_ */
