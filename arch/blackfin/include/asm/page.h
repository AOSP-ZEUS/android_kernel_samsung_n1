/*
<<<<<<< HEAD
 * Copyright 2004-2009 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */
=======
 * Copyright 2004-2009 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

#ifndef _BLACKFIN_PAGE_H
#define _BLACKFIN_PAGE_H

#include <asm-generic/page.h>
#define MAP_NR(addr) (((unsigned long)(addr)-PAGE_OFFSET) >> PAGE_SHIFT)

#define VM_DATA_DEFAULT_FLAGS \
	(VM_READ | VM_WRITE | \
	((current->personality & READ_IMPLIES_EXEC) ? VM_EXEC : 0 ) | \
		 VM_MAYREAD | VM_MAYWRITE | VM_MAYEXEC)

#include <asm-generic/memory_model.h>
#include <asm-generic/getorder.h>

#endif
