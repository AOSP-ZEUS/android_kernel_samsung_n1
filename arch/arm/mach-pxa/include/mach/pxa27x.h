#ifndef __MACH_PXA27x_H
#define __MACH_PXA27x_H

#include <mach/hardware.h>
#include <mach/pxa2xx-regs.h>
#include <mach/mfp-pxa27x.h>
<<<<<<< HEAD
=======
#include <mach/irqs.h>
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

#define ARB_CNTRL	__REG(0x48000048)  /* Arbiter Control Register */

#define ARB_DMA_SLV_PARK	(1<<31)	   /* Be parked with DMA slave when idle */
#define ARB_CI_PARK		(1<<30)	   /* Be parked with Camera Interface when idle */
#define ARB_EX_MEM_PARK 	(1<<29)	   /* Be parked with external MEMC when idle */
#define ARB_INT_MEM_PARK	(1<<28)	   /* Be parked with internal MEMC when idle */
#define ARB_USB_PARK		(1<<27)	   /* Be parked with USB when idle */
#define ARB_LCD_PARK		(1<<26)	   /* Be parked with LCD when idle */
#define ARB_DMA_PARK		(1<<25)	   /* Be parked with DMA when idle */
#define ARB_CORE_PARK		(1<<24)	   /* Be parked with core when idle */
#define ARB_LOCK_FLAG		(1<<23)	   /* Only Locking masters gain access to the bus */

<<<<<<< HEAD
extern int __init pxa27x_set_pwrmode(unsigned int mode);

=======
extern void __init pxa27x_map_io(void);
extern void __init pxa27x_init_irq(void);
extern int __init pxa27x_set_pwrmode(unsigned int mode);

#define pxa27x_handle_irq	ichp_handle_irq

>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
#endif /* __MACH_PXA27x_H */
