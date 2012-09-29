#include <linux/module.h>
<<<<<<< HEAD
=======
#include <linux/kvm_host.h>
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
#include <asm/ftrace.h>

#ifdef CONFIG_FUNCTION_TRACER
EXPORT_SYMBOL(_mcount);
#endif
<<<<<<< HEAD
=======
#if defined(CONFIG_KVM) || defined(CONFIG_KVM_MODULE)
EXPORT_SYMBOL(sie64a);
#endif
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
