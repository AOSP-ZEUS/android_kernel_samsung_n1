#ifndef XEN_HVC_CONSOLE_H
#define XEN_HVC_CONSOLE_H

extern struct console xenboot_console;

#ifdef CONFIG_HVC_XEN
void xen_console_resume(void);
void xen_raw_console_write(const char *str);
<<<<<<< HEAD
=======
__attribute__((format(printf, 1, 2)))
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
void xen_raw_printk(const char *fmt, ...);
#else
static inline void xen_console_resume(void) { }
static inline void xen_raw_console_write(const char *str) { }
<<<<<<< HEAD
static inline void xen_raw_printk(const char *fmt, ...) { }
=======
static inline __attribute__((format(printf, 1, 2)))
void xen_raw_printk(const char *fmt, ...) { }
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
#endif

#endif	/* XEN_HVC_CONSOLE_H */
