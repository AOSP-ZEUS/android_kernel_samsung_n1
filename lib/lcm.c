#include <linux/kernel.h>
#include <linux/gcd.h>
#include <linux/module.h>
<<<<<<< HEAD
=======
#include <linux/lcm.h>
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

/* Lowest common multiple */
unsigned long lcm(unsigned long a, unsigned long b)
{
	if (a && b)
		return (a * b) / gcd(a, b);
	else if (b)
		return b;

	return a;
}
EXPORT_SYMBOL_GPL(lcm);
