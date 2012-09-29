/*
 * wm_hubs.h  --  WM899x common code
 *
 * Copyright 2009 Wolfson Microelectronics plc
 *
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _WM_HUBS_H
#define _WM_HUBS_H

<<<<<<< HEAD
=======
#include <linux/completion.h>
#include <linux/interrupt.h>

>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
struct snd_soc_codec;

extern const unsigned int wm_hubs_spkmix_tlv[];

/* This *must* be the first element of the codec->private_data struct */
struct wm_hubs_data {
	int dcs_codes;
	int dcs_readback_mode;
	int hp_startup_mode;
<<<<<<< HEAD

	bool class_w;
	u16 class_w_dcs;
=======
	int series_startup;
	int no_series_update;

	bool class_w;
	u16 class_w_dcs;

	bool dcs_done_irq;
	struct completion dcs_done;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
};

extern int wm_hubs_add_analogue_controls(struct snd_soc_codec *);
extern int wm_hubs_add_analogue_routes(struct snd_soc_codec *, int, int);
extern int wm_hubs_handle_analogue_pdata(struct snd_soc_codec *,
					 int lineout1_diff, int lineout2_diff,
					 int lineout1fb, int lineout2fb,
					 int jd_scthr, int jd_thr,
					 int micbias1_lvl, int micbias2_lvl);

<<<<<<< HEAD
=======
extern irqreturn_t wm_hubs_dcs_done(int irq, void *data);

>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
#endif
