#ifndef __OF_PCI_H
#define __OF_PCI_H

#include <linux/pci.h>

struct pci_dev;
struct of_irq;
int of_irq_map_pci(struct pci_dev *pdev, struct of_irq *out_irq);
<<<<<<< HEAD
=======

struct device_node;
struct device_node *of_pci_find_child_device(struct device_node *parent,
					     unsigned int devfn);

>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
#endif
