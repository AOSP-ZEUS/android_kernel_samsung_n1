<<<<<<< HEAD
#include <linux/init.h>

__INITDATA

	.globl vdso_start, vdso_end
vdso_start:
	.incbin "arch/x86/vdso/vdso.so"
vdso_end:

__FINIT
=======
#include <asm/page_types.h>
#include <linux/linkage.h>
#include <linux/init.h>

__PAGE_ALIGNED_DATA

	.globl vdso_start, vdso_end
	.align PAGE_SIZE
vdso_start:
	.incbin "arch/x86/vdso/vdso.so"
vdso_end:
	.align PAGE_SIZE /* extra data here leaks to userspace. */

.previous

	.globl vdso_pages
	.bss
	.align 8
	.type vdso_pages, @object
vdso_pages:
	.zero (vdso_end - vdso_start + PAGE_SIZE - 1) / PAGE_SIZE * 8
	.size vdso_pages, .-vdso_pages
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
