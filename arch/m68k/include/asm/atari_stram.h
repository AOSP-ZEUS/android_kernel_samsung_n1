#ifndef _M68K_ATARI_STRAM_H
#define _M68K_ATARI_STRAM_H

/*
 * Functions for Atari ST-RAM management
 */

/* public interface */
<<<<<<< HEAD
void *atari_stram_alloc(long size, const char *owner);
=======
void *atari_stram_alloc(unsigned long size, const char *owner);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
void atari_stram_free(void *);

/* functions called internally by other parts of the kernel */
void atari_stram_init(void);
void atari_stram_reserve_pages(void *start_mem);
<<<<<<< HEAD
void atari_stram_mem_init_hook (void);
=======
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

#endif /*_M68K_ATARI_STRAM_H */
