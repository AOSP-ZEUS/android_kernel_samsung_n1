#ifndef __UM_DELAY_H
#define __UM_DELAY_H

<<<<<<< HEAD
#define MILLION 1000000

/* Undefined on purpose */
extern void __bad_udelay(void);

extern void __udelay(unsigned long usecs);
=======
/* Undefined on purpose */
extern void __bad_udelay(void);
extern void __bad_ndelay(void);

extern void __udelay(unsigned long usecs);
extern void __ndelay(unsigned long usecs);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
extern void __delay(unsigned long loops);

#define udelay(n) ((__builtin_constant_p(n) && (n) > 20000) ? \
	__bad_udelay() : __udelay(n))

<<<<<<< HEAD
/* It appears that ndelay is not used at all for UML, and has never been
 * implemented. */
extern void __unimplemented_ndelay(void);
#define ndelay(n) __unimplemented_ndelay()
=======
#define ndelay(n) ((__builtin_constant_p(n) && (n) > 20000) ? \
	__bad_ndelay() : __ndelay(n))
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

#endif
