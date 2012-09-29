#ifndef __ACPI_NUMA_H
#define __ACPI_NUMA_H

#ifdef CONFIG_ACPI_NUMA
#include <linux/kernel.h>

/* Proximity bitmap length */
#if MAX_NUMNODES > 256
#define MAX_PXM_DOMAINS MAX_NUMNODES
#else
#define MAX_PXM_DOMAINS (256)	/* Old pxm spec is defined 8 bit */
#endif

extern int pxm_to_node(int);
extern int node_to_pxm(int);
extern void __acpi_map_pxm_to_node(int, int);
extern int acpi_map_pxm_to_node(int);
<<<<<<< HEAD
extern unsigned char acpi_srat_revision;
=======
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

#endif				/* CONFIG_ACPI_NUMA */
#endif				/* __ACP_NUMA_H */
