<<<<<<< HEAD
#ifndef __SH_MOBILE_SDHI_H__
#define __SH_MOBILE_SDHI_H__
=======
#ifndef LINUX_MMC_SH_MOBILE_SDHI_H
#define LINUX_MMC_SH_MOBILE_SDHI_H
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

#include <linux/types.h>

struct platform_device;
struct tmio_mmc_data;

struct sh_mobile_sdhi_info {
	int dma_slave_tx;
	int dma_slave_rx;
	unsigned long tmio_flags;
	unsigned long tmio_caps;
	u32 tmio_ocr_mask;	/* available MMC voltages */
	struct tmio_mmc_data *pdata;
	void (*set_pwr)(struct platform_device *pdev, int state);
	int (*get_cd)(struct platform_device *pdev);
};

<<<<<<< HEAD
#endif /* __SH_MOBILE_SDHI_H__ */
=======
#endif /* LINUX_MMC_SH_MOBILE_SDHI_H */
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
