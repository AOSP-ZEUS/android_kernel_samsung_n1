/*
 * ALSA SoC SPDIF DIT driver
 *
 *  This driver is used by controllers which can operate in DIT (SPDI/F) where
 *  no codec is needed.  This file provides stub codec that can be used
 *  in these configurations. TI DaVinci Audio controller uses this driver.
 *
 * Author:      Steve Chen,  <schen@mvista.com>
 * Copyright:   (C) 2009 MontaVista Software, Inc., <source@mvista.com>
 * Copyright:   (C) 2009  Texas Instruments, India
<<<<<<< HEAD
=======
 * Copyright:   (C) 2009-2012, NVIDIA Corporation.
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

<<<<<<< HEAD
=======

#include <asm/mach-types.h>
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/initval.h>

#define DRV_NAME "spdif-dit"

#define STUB_RATES	SNDRV_PCM_RATE_8000_96000
#define STUB_FORMATS	SNDRV_PCM_FMTBIT_S16_LE

<<<<<<< HEAD

static struct snd_soc_codec_driver soc_codec_spdif_dit;

=======
static struct snd_soc_codec_driver soc_codec_spdif_dit;

static int spdif_probe(struct snd_soc_codec *codec) {
	codec->dapm.idle_bias_off = 1;
	return 0;
}

static const struct snd_soc_dapm_widget spdif_dapm_widgets[] = {
	SND_SOC_DAPM_VMID("spdif dummy Vmid"),
};

static int spdif_write(struct snd_soc_codec * codec, unsigned int reg,
							unsigned int val){
	return 0;
}

static int spdif_read(struct snd_soc_codec * codec, unsigned int reg){
	return 0;
}

static struct snd_soc_codec_driver soc_codec_spdif_dit1 = {
	.probe = spdif_probe,
	.dapm_widgets = spdif_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(spdif_dapm_widgets),
	.read = spdif_read,
	.write = spdif_write,
};

>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
static struct snd_soc_dai_driver dit_stub_dai = {
	.name		= "dit-hifi",
	.playback 	= {
		.stream_name	= "Playback",
		.channels_min	= 1,
		.channels_max	= 384,
		.rates		= STUB_RATES,
		.formats	= STUB_FORMATS,
	},
<<<<<<< HEAD
=======
	.capture	= {
		.stream_name	= "Capture",
		.channels_min	= 1,
		.channels_max	= 384,
		.rates		= STUB_RATES,
		.formats	= STUB_FORMATS,
	},
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
};

static int spdif_dit_probe(struct platform_device *pdev)
{
<<<<<<< HEAD
	return snd_soc_register_codec(&pdev->dev, &soc_codec_spdif_dit,
			&dit_stub_dai, 1);
=======
	if(machine_is_kai())
		return snd_soc_register_codec(&pdev->dev,
			&soc_codec_spdif_dit1, &dit_stub_dai, 1);
	else
		return snd_soc_register_codec(&pdev->dev,
			 &soc_codec_spdif_dit, &dit_stub_dai, 1);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
}

static int spdif_dit_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver spdif_dit_driver = {
	.probe		= spdif_dit_probe,
	.remove		= spdif_dit_remove,
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init dit_modinit(void)
{
	return platform_driver_register(&spdif_dit_driver);
}

static void __exit dit_exit(void)
{
	platform_driver_unregister(&spdif_dit_driver);
}

module_init(dit_modinit);
module_exit(dit_exit);

MODULE_AUTHOR("Steve Chen <schen@mvista.com>");
MODULE_DESCRIPTION("SPDIF dummy codec driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
