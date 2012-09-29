#include <linux/linkage.h>
#include <asm/assembler.h>
/*
 * Function: nommu_early_abort
 *
<<<<<<< HEAD
 * Params  : r2 = address of aborted instruction
 *         : r3 = saved SPSR
 *
 * Returns : r0 = 0 (abort address)
 *	   : r1 = 0 (FSR)
=======
 * Params  : r2 = pt_regs
 *	   : r4 = aborted context pc
 *	   : r5 = aborted context psr
 *
 * Returns : r4 - r11, r13 preserved
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
 *
 * Note: There is no FSR/FAR on !CPU_CP15_MMU cores.
 *       Just fill zero into the registers.
 */
	.align	5
ENTRY(nommu_early_abort)
	mov	r0, #0				@ clear r0, r1 (no FSR/FAR)
	mov	r1, #0
<<<<<<< HEAD
	mov	pc, lr
=======
	b	do_DataAbort
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
ENDPROC(nommu_early_abort)
