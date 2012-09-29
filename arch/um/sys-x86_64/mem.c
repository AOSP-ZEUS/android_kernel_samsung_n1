<<<<<<< HEAD
/*
 * Copyright 2003 PathScale, Inc.
 *
 * Licensed under the GPL
 */

=======
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
#include "linux/mm.h"
#include "asm/page.h"
#include "asm/mman.h"

<<<<<<< HEAD
unsigned long vm_stack_flags = __VM_STACK_FLAGS;
unsigned long vm_stack_flags32 = __VM_STACK_FLAGS;
unsigned long vm_data_default_flags = __VM_DATA_DEFAULT_FLAGS;
unsigned long vm_data_default_flags32 = __VM_DATA_DEFAULT_FLAGS;
unsigned long vm_force_exec32 = PROT_EXEC;

=======
const char *arch_vma_name(struct vm_area_struct *vma)
{
	if (vma->vm_mm && vma->vm_start == um_vdso_addr)
		return "[vdso]";

	return NULL;
}

struct vm_area_struct *get_gate_vma(struct mm_struct *mm)
{
	return NULL;
}

int in_gate_area(struct mm_struct *mm, unsigned long addr)
{
	return 0;
}

int in_gate_area_no_mm(unsigned long addr)
{
	return 0;
}
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
