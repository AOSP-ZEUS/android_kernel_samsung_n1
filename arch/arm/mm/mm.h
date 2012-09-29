#ifdef CONFIG_MMU

/* the upper-most page table pointer */
extern pmd_t *top_pmd;

#define TOP_PTE(x)	pte_offset_kernel(top_pmd, x)

static inline pmd_t *pmd_off_k(unsigned long virt)
{
	return pmd_offset(pud_offset(pgd_offset_k(virt), virt), virt);
}

struct mem_type {
	pteval_t prot_pte;
	unsigned int prot_l1;
	unsigned int prot_sect;
	unsigned int domain;
};

const struct mem_type *get_mem_type(unsigned int type);

extern void __flush_dcache_page(struct address_space *mapping, struct page *page);

#endif

<<<<<<< HEAD
=======
#ifdef CONFIG_ZONE_DMA
extern u32 arm_dma_limit;
#else
#define arm_dma_limit ((u32)~0)
#endif

>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
void __init bootmem_init(void);
void arm_mm_memblock_reserve(void);
