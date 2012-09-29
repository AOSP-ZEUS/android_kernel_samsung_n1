#ifndef _ASM_X86_TIME_H
#define _ASM_X86_TIME_H

<<<<<<< HEAD
extern void hpet_time_init(void);

#include <asm/mc146818rtc.h>

extern void time_init(void);

=======
#include <linux/clocksource.h>
#include <asm/mc146818rtc.h>

extern void hpet_time_init(void);
extern void time_init(void);

extern struct clock_event_device *global_clock_event;

>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
#endif /* _ASM_X86_TIME_H */
