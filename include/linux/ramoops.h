#ifndef __RAMOOPS_H
#define __RAMOOPS_H

/*
 * Ramoops platform data
 * @mem_size	memory size for ramoops
 * @mem_address	physical memory address to contain ramoops
 */

struct ramoops_platform_data {
	unsigned long	mem_size;
	unsigned long	mem_address;
<<<<<<< HEAD
=======
	unsigned long	record_size;
	int		dump_oops;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
};

#endif
