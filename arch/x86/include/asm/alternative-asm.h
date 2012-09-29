#ifdef __ASSEMBLY__

#include <asm/asm.h>

#ifdef CONFIG_SMP
	.macro LOCK_PREFIX
1:	lock
	.section .smp_locks,"a"
	.balign 4
	.long 1b - .
	.previous
	.endm
#else
	.macro LOCK_PREFIX
	.endm
#endif

.macro altinstruction_entry orig alt feature orig_len alt_len
<<<<<<< HEAD
	.align 8
	.quad \orig
	.quad \alt
=======
	.long \orig - .
	.long \alt - .
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	.word \feature
	.byte \orig_len
	.byte \alt_len
.endm

#endif  /*  __ASSEMBLY__  */
