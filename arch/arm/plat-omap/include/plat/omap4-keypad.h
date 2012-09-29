#ifndef ARCH_ARM_PLAT_OMAP4_KEYPAD_H
#define ARCH_ARM_PLAT_OMAP4_KEYPAD_H

#include <linux/input/matrix_keypad.h>

struct omap4_keypad_platform_data {
	const struct matrix_keymap_data *keymap_data;

	u8 rows;
	u8 cols;
};

<<<<<<< HEAD
extern int omap4_keyboard_init(struct omap4_keypad_platform_data *);
=======
extern int omap4_keyboard_init(struct omap4_keypad_platform_data *,
				struct omap_board_data *);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
#endif
