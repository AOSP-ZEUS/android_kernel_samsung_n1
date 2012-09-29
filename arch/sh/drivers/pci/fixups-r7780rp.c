/*
 * arch/sh/drivers/pci/fixups-r7780rp.c
 *
 * Highlander R7780RP-1 PCI fixups
 *
 * Copyright (C) 2003  Lineo uSolutions, Inc.
 * Copyright (C) 2004 - 2006  Paul Mundt
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#include <linux/pci.h>
#include <linux/io.h>
#include "pci-sh4.h"

static char irq_tab[] __initdata = {
	65, 66, 67, 68,
};

<<<<<<< HEAD
int __init pcibios_map_platform_irq(struct pci_dev *pdev, u8 slot, u8 pin)
=======
int __init pcibios_map_platform_irq(const struct pci_dev *pdev, u8 slot, u8 pin)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
{
	return irq_tab[slot];
}
