#ifndef _PERF_LINUX_COMPILER_H_
#define _PERF_LINUX_COMPILER_H_

#ifndef __always_inline
#define __always_inline	inline
#endif
#define __user
<<<<<<< HEAD
#define __attribute_const__
=======
#ifndef __attribute_const__
#define __attribute_const__
#endif
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

#define __used		__attribute__((__unused__))

#endif
