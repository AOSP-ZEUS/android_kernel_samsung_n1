/*
<<<<<<< HEAD
 * Copyright 2004-2008 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */
=======
 * Copyright 2004-2008 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

#ifndef _ASM_BFIN_MODULE_H
#define _ASM_BFIN_MODULE_H

#define Elf_Shdr        Elf32_Shdr
#define Elf_Sym         Elf32_Sym
#define Elf_Ehdr        Elf32_Ehdr

struct mod_arch_specific {
	Elf_Shdr	*text_l1;
	Elf_Shdr	*data_a_l1;
	Elf_Shdr	*bss_a_l1;
	Elf_Shdr	*data_b_l1;
	Elf_Shdr	*bss_b_l1;
	Elf_Shdr	*text_l2;
	Elf_Shdr	*data_l2;
	Elf_Shdr	*bss_l2;
};
#endif				/* _ASM_BFIN_MODULE_H */
