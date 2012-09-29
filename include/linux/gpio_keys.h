#ifndef _GPIO_KEYS_H
#define _GPIO_KEYS_H

struct gpio_keys_button {
	/* Configuration parameters */
	unsigned int code;	/* input event code (KEY_*, SW_*) */
<<<<<<< HEAD
	int gpio;
=======
	int gpio;		/* -1 if this key does not support gpio */	
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	int active_low;
	const char *desc;
	unsigned int type;	/* input event type (EV_KEY, EV_SW, EV_ABS) */
	int wakeup;		/* configure the button as a wake-up source */
	int debounce_interval;	/* debounce ticks interval in msecs */
	bool can_disable;
	int value;		/* axis value for EV_ABS */
<<<<<<< HEAD
=======
	unsigned int irq;	/* Irq number in case of interrupt keys */	
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
};

struct gpio_keys_platform_data {
	struct gpio_keys_button *buttons;
	int nbuttons;
	unsigned int poll_interval;	/* polling interval in msecs -
					   for polling driver only */
	unsigned int rep:1;		/* enable input subsystem auto repeat */
	int (*enable)(struct device *dev);
	void (*disable)(struct device *dev);
	const char *name;		/* input device name */
<<<<<<< HEAD
=======
	int (*wakeup_key)(void);
#ifdef CONFIG_MACH_N1
	int (*wakeup_key_twice)(void);
#endif
#ifdef CONFIG_SAMSUNG_LPM_MODE
    bool (*check_lpm)(void);
#endif
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
};

#endif
