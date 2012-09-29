/*
 * Copyright (c) 2010 Broadcom Corporation
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
<<<<<<< HEAD
/* ****************** BCMSDH Interface Functions *************************** */

#include <linux/types.h>
#include <linux/netdevice.h>
#include <linux/pci_ids.h>
#include <bcmdefs.h>
#include <bcmdevs.h>
#include <bcmutils.h>
#include <hndsoc.h>

#include <bcmsdh.h>		/* BRCM API for SDIO
			 clients (such as wl, dhd) */
#include <bcmsdbus.h>		/* common SDIO/controller interface */
#include <sbsdio.h>		/* BRCM sdio device core */

#include <sdio.h>		/* sdio spec */
#include "dngl_stats.h"
#include "dhd.h"

#define SDIOH_API_ACCESS_RETRY_LIMIT	2
const uint bcmsdh_msglevel = BCMSDH_ERROR_VAL;

struct bcmsdh_info {
=======
/* ****************** SDIO CARD Interface Functions **************************/

#include <linux/types.h>
#include <linux/netdevice.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/sched.h>
#include <linux/completion.h>

#include <defs.h>
#include <brcm_hw_ids.h>
#include <brcmu_utils.h>
#include <brcmu_wifi.h>
#include <soc.h>
#include "dhd.h"
#include "dhd_bus.h"
#include "sdio_host.h"

#define SDIOH_API_ACCESS_RETRY_LIMIT	2

#define BRCMF_SD_ERROR_VAL	0x0001	/* Error */
#define BRCMF_SD_INFO_VAL		0x0002	/* Info */


#ifdef BCMDBG
#define BRCMF_SD_ERROR(x) \
	do { \
		if ((brcmf_sdio_msglevel & BRCMF_SD_ERROR_VAL) && \
		    net_ratelimit()) \
			printk x; \
	} while (0)
#define BRCMF_SD_INFO(x)	\
	do { \
		if ((brcmf_sdio_msglevel & BRCMF_SD_INFO_VAL) && \
		    net_ratelimit()) \
			printk x; \
	} while (0)
#else				/* BCMDBG */
#define BRCMF_SD_ERROR(x)
#define BRCMF_SD_INFO(x)
#endif				/* BCMDBG */

/* debugging macros */
#define SDLX_MSG(x)

#define SDIOH_CMD_TYPE_NORMAL   0	/* Normal command */
#define SDIOH_CMD_TYPE_APPEND   1	/* Append command */
#define SDIOH_CMD_TYPE_CUTTHRU  2	/* Cut-through command */

#define SDIOH_DATA_PIO          0	/* PIO mode */
#define SDIOH_DATA_DMA          1	/* DMA mode */

struct brcmf_sdio_card {
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	bool init_success;	/* underlying driver successfully attached */
	void *sdioh;		/* handler for sdioh */
	u32 vendevid;	/* Target Vendor and Device ID on SD bus */
	bool regfail;		/* Save status of last
				 reg_read/reg_write call */
	u32 sbwad;		/* Save backplane window address */
};
<<<<<<< HEAD
/* local copy of bcm sd handler */
bcmsdh_info_t *l_bcmsdh;

#if defined(OOB_INTR_ONLY) && defined(HW_OOB)
extern int sdioh_enable_hw_oob_intr(void *sdioh, bool enable);

void bcmsdh_enable_hw_oob_intr(bcmsdh_info_t *sdh, bool enable)
{
	sdioh_enable_hw_oob_intr(sdh->sdioh, enable);
}
#endif

bcmsdh_info_t *bcmsdh_attach(void *cfghdl, void **regsva, uint irq)
{
	bcmsdh_info_t *bcmsdh;

	bcmsdh = kzalloc(sizeof(bcmsdh_info_t), GFP_ATOMIC);
	if (bcmsdh == NULL) {
		BCMSDH_ERROR(("bcmsdh_attach: out of memory"));
=======

/**
 * SDIO Host Controller info
 */
struct sdio_hc {
	struct sdio_hc *next;
	struct device *dev;	/* platform device handle */
	void *regs;		/* SDIO Host Controller address */
	struct brcmf_sdio_card *card;
	void *ch;
	unsigned int oob_irq;
	unsigned long oob_flags;	/* OOB Host specifiction
					as edge and etc */
	bool oob_irq_registered;
};

/* local copy of bcm sd handler */
static struct brcmf_sdio_card *l_card;

const uint brcmf_sdio_msglevel = BRCMF_SD_ERROR_VAL;

static struct sdio_hc *sdhcinfo;

/* driver info, initialized when brcmf_sdio_register is called */
static struct brcmf_sdioh_driver drvinfo = { NULL, NULL };

/* Module parameters specific to each host-controller driver */

module_param(sd_msglevel, uint, 0);

extern uint sd_f2_blocksize;
module_param(sd_f2_blocksize, int, 0);

/* forward declarations */
int brcmf_sdio_probe(struct device *dev);
EXPORT_SYMBOL(brcmf_sdio_probe);

int brcmf_sdio_remove(struct device *dev);
EXPORT_SYMBOL(brcmf_sdio_remove);

struct brcmf_sdio_card*
brcmf_sdcard_attach(void *cfghdl, u32 *regsva, uint irq)
{
	struct brcmf_sdio_card *card;

	card = kzalloc(sizeof(struct brcmf_sdio_card), GFP_ATOMIC);
	if (card == NULL) {
		BRCMF_SD_ERROR(("sdcard_attach: out of memory"));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		return NULL;
	}

	/* save the handler locally */
<<<<<<< HEAD
	l_bcmsdh = bcmsdh;

	bcmsdh->sdioh = sdioh_attach(cfghdl, irq);
	if (!bcmsdh->sdioh) {
		bcmsdh_detach(bcmsdh);
		return NULL;
	}

	bcmsdh->init_success = true;

	*regsva = (u32 *) SI_ENUM_BASE;

	/* Report the BAR, to fix if needed */
	bcmsdh->sbwad = SI_ENUM_BASE;
	return bcmsdh;
}

int bcmsdh_detach(void *sdh)
{
	bcmsdh_info_t *bcmsdh = (bcmsdh_info_t *) sdh;

	if (bcmsdh != NULL) {
		if (bcmsdh->sdioh) {
			sdioh_detach(bcmsdh->sdioh);
			bcmsdh->sdioh = NULL;
		}
		kfree(bcmsdh);
	}

	l_bcmsdh = NULL;
=======
	l_card = card;

	card->sdioh = brcmf_sdioh_attach(cfghdl, irq);
	if (!card->sdioh) {
		brcmf_sdcard_detach(card);
		return NULL;
	}

	card->init_success = true;

	*regsva = SI_ENUM_BASE;

	/* Report the BAR, to fix if needed */
	card->sbwad = SI_ENUM_BASE;
	return card;
}

int brcmf_sdcard_detach(struct brcmf_sdio_card *card)
{
	if (card != NULL) {
		if (card->sdioh) {
			brcmf_sdioh_detach(card->sdioh);
			card->sdioh = NULL;
		}
		kfree(card);
	}

	l_card = NULL;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	return 0;
}

int
<<<<<<< HEAD
bcmsdh_iovar_op(void *sdh, const char *name,
		void *params, int plen, void *arg, int len, bool set)
{
	bcmsdh_info_t *bcmsdh = (bcmsdh_info_t *) sdh;
	return sdioh_iovar_op(bcmsdh->sdioh, name, params, plen, arg, len, set);
}

bool bcmsdh_intr_query(void *sdh)
{
	bcmsdh_info_t *bcmsdh = (bcmsdh_info_t *) sdh;
	SDIOH_API_RC status;
	bool on;

	ASSERT(bcmsdh);
	status = sdioh_interrupt_query(bcmsdh->sdioh, &on);
	if (SDIOH_API_SUCCESS(status))
		return false;
	else
		return on;
}

int bcmsdh_intr_enable(void *sdh)
{
	bcmsdh_info_t *bcmsdh = (bcmsdh_info_t *) sdh;
	SDIOH_API_RC status;
	ASSERT(bcmsdh);

	status = sdioh_interrupt_set(bcmsdh->sdioh, true);
	return SDIOH_API_SUCCESS(status) ? 0 : -EIO;
}

int bcmsdh_intr_disable(void *sdh)
{
	bcmsdh_info_t *bcmsdh = (bcmsdh_info_t *) sdh;
	SDIOH_API_RC status;
	ASSERT(bcmsdh);

	status = sdioh_interrupt_set(bcmsdh->sdioh, false);
	return SDIOH_API_SUCCESS(status) ? 0 : -EIO;
}

int bcmsdh_intr_reg(void *sdh, bcmsdh_cb_fn_t fn, void *argh)
{
	bcmsdh_info_t *bcmsdh = (bcmsdh_info_t *) sdh;
	SDIOH_API_RC status;
	ASSERT(bcmsdh);

	status = sdioh_interrupt_register(bcmsdh->sdioh, fn, argh);
	return SDIOH_API_SUCCESS(status) ? 0 : -EIO;
}

int bcmsdh_intr_dereg(void *sdh)
{
	bcmsdh_info_t *bcmsdh = (bcmsdh_info_t *) sdh;
	SDIOH_API_RC status;
	ASSERT(bcmsdh);

	status = sdioh_interrupt_deregister(bcmsdh->sdioh);
	return SDIOH_API_SUCCESS(status) ? 0 : -EIO;
}

#if defined(DHD_DEBUG)
bool bcmsdh_intr_pending(void *sdh)
{
	bcmsdh_info_t *bcmsdh = (bcmsdh_info_t *) sdh;

	ASSERT(sdh);
	return sdioh_interrupt_pending(bcmsdh->sdioh);
}
#endif

int bcmsdh_devremove_reg(void *sdh, bcmsdh_cb_fn_t fn, void *argh)
{
	ASSERT(sdh);

	/* don't support yet */
	return -ENOTSUPP;
}

u8 bcmsdh_cfg_read(void *sdh, uint fnc_num, u32 addr, int *err)
{
	bcmsdh_info_t *bcmsdh = (bcmsdh_info_t *) sdh;
	SDIOH_API_RC status;
#ifdef SDIOH_API_ACCESS_RETRY_LIMIT
	s32 retry = 0;
#endif
	u8 data = 0;

	if (!bcmsdh)
		bcmsdh = l_bcmsdh;

	ASSERT(bcmsdh->init_success);

#ifdef SDIOH_API_ACCESS_RETRY_LIMIT
	do {
		if (retry)	/* wait for 1 ms till bus get settled down */
			udelay(1000);
#endif
		status =
		    sdioh_cfg_read(bcmsdh->sdioh, fnc_num, addr,
				   (u8 *) &data);
#ifdef SDIOH_API_ACCESS_RETRY_LIMIT
	} while (!SDIOH_API_SUCCESS(status)
		 && (retry++ < SDIOH_API_ACCESS_RETRY_LIMIT));
#endif
	if (err)
		*err = (SDIOH_API_SUCCESS(status) ? 0 : -EIO);

	BCMSDH_INFO(("%s:fun = %d, addr = 0x%x, u8data = 0x%x\n",
=======
brcmf_sdcard_iovar_op(struct brcmf_sdio_card *card, const char *name,
		void *params, int plen, void *arg, int len, bool set)
{
	return brcmf_sdioh_iovar_op(card->sdioh, name, params, plen, arg,
				    len, set);
}

int brcmf_sdcard_intr_enable(struct brcmf_sdio_card *card)
{
	return brcmf_sdioh_interrupt_set(card->sdioh, true);
}

int brcmf_sdcard_intr_disable(struct brcmf_sdio_card *card)
{
	return brcmf_sdioh_interrupt_set(card->sdioh, false);
}

int brcmf_sdcard_intr_reg(struct brcmf_sdio_card *card,
			  void (*fn)(void *), void *argh)
{
	return brcmf_sdioh_interrupt_register(card->sdioh, fn, argh);
}

int brcmf_sdcard_intr_dereg(struct brcmf_sdio_card *card)
{
	return brcmf_sdioh_interrupt_deregister(card->sdioh);
}

u8 brcmf_sdcard_cfg_read(struct brcmf_sdio_card *card, uint fnc_num, u32 addr,
			 int *err)
{
	int status;
	s32 retry = 0;
	u8 data = 0;

	if (!card)
		card = l_card;

	do {
		if (retry)	/* wait for 1 ms till bus get settled down */
			udelay(1000);
		status =
		    brcmf_sdioh_cfg_read(card->sdioh, fnc_num, addr,
				   (u8 *) &data);
	} while (status != 0
		 && (retry++ < SDIOH_API_ACCESS_RETRY_LIMIT));
	if (err)
		*err = status;

	BRCMF_SD_INFO(("%s:fun = %d, addr = 0x%x, u8data = 0x%x\n",
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		     __func__, fnc_num, addr, data));

	return data;
}

void
<<<<<<< HEAD
bcmsdh_cfg_write(void *sdh, uint fnc_num, u32 addr, u8 data, int *err)
{
	bcmsdh_info_t *bcmsdh = (bcmsdh_info_t *) sdh;
	SDIOH_API_RC status;
#ifdef SDIOH_API_ACCESS_RETRY_LIMIT
	s32 retry = 0;
#endif

	if (!bcmsdh)
		bcmsdh = l_bcmsdh;

	ASSERT(bcmsdh->init_success);

#ifdef SDIOH_API_ACCESS_RETRY_LIMIT
	do {
		if (retry)	/* wait for 1 ms till bus get settled down */
			udelay(1000);
#endif
		status =
		    sdioh_cfg_write(bcmsdh->sdioh, fnc_num, addr,
				    (u8 *) &data);
#ifdef SDIOH_API_ACCESS_RETRY_LIMIT
	} while (!SDIOH_API_SUCCESS(status)
		 && (retry++ < SDIOH_API_ACCESS_RETRY_LIMIT));
#endif
	if (err)
		*err = SDIOH_API_SUCCESS(status) ? 0 : -EIO;

	BCMSDH_INFO(("%s:fun = %d, addr = 0x%x, u8data = 0x%x\n",
		     __func__, fnc_num, addr, data));
}

u32 bcmsdh_cfg_read_word(void *sdh, uint fnc_num, u32 addr, int *err)
{
	bcmsdh_info_t *bcmsdh = (bcmsdh_info_t *) sdh;
	SDIOH_API_RC status;
	u32 data = 0;

	if (!bcmsdh)
		bcmsdh = l_bcmsdh;

	ASSERT(bcmsdh->init_success);

	status =
	    sdioh_request_word(bcmsdh->sdioh, SDIOH_CMD_TYPE_NORMAL, SDIOH_READ,
			       fnc_num, addr, &data, 4);

	if (err)
		*err = (SDIOH_API_SUCCESS(status) ? 0 : -EIO);

	BCMSDH_INFO(("%s:fun = %d, addr = 0x%x, u32data = 0x%x\n",
=======
brcmf_sdcard_cfg_write(struct brcmf_sdio_card *card, uint fnc_num, u32 addr,
		       u8 data, int *err)
{
	int status;
	s32 retry = 0;

	if (!card)
		card = l_card;

	do {
		if (retry)	/* wait for 1 ms till bus get settled down */
			udelay(1000);
		status =
		    brcmf_sdioh_cfg_write(card->sdioh, fnc_num, addr,
				    (u8 *) &data);
	} while (status != 0
		 && (retry++ < SDIOH_API_ACCESS_RETRY_LIMIT));
	if (err)
		*err = status;

	BRCMF_SD_INFO(("%s:fun = %d, addr = 0x%x, u8data = 0x%x\n",
		     __func__, fnc_num, addr, data));
}

u32 brcmf_sdcard_cfg_read_word(struct brcmf_sdio_card *card, uint fnc_num,
			       u32 addr, int *err)
{
	int status;
	u32 data = 0;

	if (!card)
		card = l_card;

	status = brcmf_sdioh_request_word(card->sdioh, SDIOH_CMD_TYPE_NORMAL,
		SDIOH_READ, fnc_num, addr, &data, 4);

	if (err)
		*err = status;

	BRCMF_SD_INFO(("%s:fun = %d, addr = 0x%x, u32data = 0x%x\n",
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		     __func__, fnc_num, addr, data));

	return data;
}

void
<<<<<<< HEAD
bcmsdh_cfg_write_word(void *sdh, uint fnc_num, u32 addr, u32 data,
		      int *err)
{
	bcmsdh_info_t *bcmsdh = (bcmsdh_info_t *) sdh;
	SDIOH_API_RC status;

	if (!bcmsdh)
		bcmsdh = l_bcmsdh;

	ASSERT(bcmsdh->init_success);

	status =
	    sdioh_request_word(bcmsdh->sdioh, SDIOH_CMD_TYPE_NORMAL,
			       SDIOH_WRITE, fnc_num, addr, &data, 4);

	if (err)
		*err = (SDIOH_API_SUCCESS(status) ? 0 : -EIO);

	BCMSDH_INFO(("%s:fun = %d, addr = 0x%x, u32data = 0x%x\n",
		     __func__, fnc_num, addr, data));
}

int bcmsdh_cis_read(void *sdh, uint func, u8 * cis, uint length)
{
	bcmsdh_info_t *bcmsdh = (bcmsdh_info_t *) sdh;
	SDIOH_API_RC status;
=======
brcmf_sdcard_cfg_write_word(struct brcmf_sdio_card *card, uint fnc_num,
			    u32 addr, u32 data, int *err)
{
	int status;

	if (!card)
		card = l_card;

	status =
	    brcmf_sdioh_request_word(card->sdioh, SDIOH_CMD_TYPE_NORMAL,
			       SDIOH_WRITE, fnc_num, addr, &data, 4);

	if (err)
		*err = status;

	BRCMF_SD_INFO(("%s:fun = %d, addr = 0x%x, u32data = 0x%x\n",
		     __func__, fnc_num, addr, data));
}

int brcmf_sdcard_cis_read(struct brcmf_sdio_card *card, uint func, u8 * cis,
			  uint length)
{
	int status;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	u8 *tmp_buf, *tmp_ptr;
	u8 *ptr;
	bool ascii = func & ~0xf;
	func &= 0x7;

<<<<<<< HEAD
	if (!bcmsdh)
		bcmsdh = l_bcmsdh;

	ASSERT(bcmsdh->init_success);
	ASSERT(cis);
	ASSERT(length <= SBSDIO_CIS_SIZE_LIMIT);

	status = sdioh_cis_read(bcmsdh->sdioh, func, cis, length);
=======
	if (!card)
		card = l_card;

	status = brcmf_sdioh_cis_read(card->sdioh, func, cis, length);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	if (ascii) {
		/* Move binary bits to tmp and format them
			 into the provided buffer. */
		tmp_buf = kmalloc(length, GFP_ATOMIC);
		if (tmp_buf == NULL) {
<<<<<<< HEAD
			BCMSDH_ERROR(("%s: out of memory\n", __func__));
=======
			BRCMF_SD_ERROR(("%s: out of memory\n", __func__));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			return -ENOMEM;
		}
		memcpy(tmp_buf, cis, length);
		for (tmp_ptr = tmp_buf, ptr = cis; ptr < (cis + length - 4);
		     tmp_ptr++) {
			ptr += sprintf((char *)ptr, "%.2x ", *tmp_ptr & 0xff);
			if ((((tmp_ptr - tmp_buf) + 1) & 0xf) == 0)
				ptr += sprintf((char *)ptr, "\n");
		}
		kfree(tmp_buf);
	}

<<<<<<< HEAD
	return SDIOH_API_SUCCESS(status) ? 0 : -EIO;
}

static int bcmsdhsdio_set_sbaddr_window(void *sdh, u32 address)
{
	int err = 0;
	bcmsdh_info_t *bcmsdh = (bcmsdh_info_t *) sdh;
	bcmsdh_cfg_write(bcmsdh, SDIO_FUNC_1, SBSDIO_FUNC1_SBADDRLOW,
			 (address >> 8) & SBSDIO_SBADDRLOW_MASK, &err);
	if (!err)
		bcmsdh_cfg_write(bcmsdh, SDIO_FUNC_1, SBSDIO_FUNC1_SBADDRMID,
				 (address >> 16) & SBSDIO_SBADDRMID_MASK, &err);
	if (!err)
		bcmsdh_cfg_write(bcmsdh, SDIO_FUNC_1, SBSDIO_FUNC1_SBADDRHIGH,
				 (address >> 24) & SBSDIO_SBADDRHIGH_MASK,
				 &err);
=======
	return status;
}

static int
brcmf_sdcard_set_sbaddr_window(struct brcmf_sdio_card *card, u32 address)
{
	int err = 0;
	brcmf_sdcard_cfg_write(card, SDIO_FUNC_1, SBSDIO_FUNC1_SBADDRLOW,
			 (address >> 8) & SBSDIO_SBADDRLOW_MASK, &err);
	if (!err)
		brcmf_sdcard_cfg_write(card, SDIO_FUNC_1,
				       SBSDIO_FUNC1_SBADDRMID,
				       (address >> 16) & SBSDIO_SBADDRMID_MASK,
				       &err);
	if (!err)
		brcmf_sdcard_cfg_write(card, SDIO_FUNC_1,
				       SBSDIO_FUNC1_SBADDRHIGH,
				       (address >> 24) & SBSDIO_SBADDRHIGH_MASK,
				       &err);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	return err;
}

<<<<<<< HEAD
u32 bcmsdh_reg_read(void *sdh, u32 addr, uint size)
{
	bcmsdh_info_t *bcmsdh = (bcmsdh_info_t *) sdh;
	SDIOH_API_RC status;
	u32 word = 0;
	uint bar0 = addr & ~SBSDIO_SB_OFT_ADDR_MASK;

	BCMSDH_INFO(("%s:fun = 1, addr = 0x%x, ", __func__, addr));

	if (!bcmsdh)
		bcmsdh = l_bcmsdh;

	ASSERT(bcmsdh->init_success);

	if (bar0 != bcmsdh->sbwad) {
		if (bcmsdhsdio_set_sbaddr_window(bcmsdh, bar0))
			return 0xFFFFFFFF;

		bcmsdh->sbwad = bar0;
=======
u32 brcmf_sdcard_reg_read(struct brcmf_sdio_card *card, u32 addr, uint size)
{
	int status;
	u32 word = 0;
	uint bar0 = addr & ~SBSDIO_SB_OFT_ADDR_MASK;

	BRCMF_SD_INFO(("%s:fun = 1, addr = 0x%x, ", __func__, addr));

	if (!card)
		card = l_card;

	if (bar0 != card->sbwad) {
		if (brcmf_sdcard_set_sbaddr_window(card, bar0))
			return 0xFFFFFFFF;

		card->sbwad = bar0;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	}

	addr &= SBSDIO_SB_OFT_ADDR_MASK;
	if (size == 4)
		addr |= SBSDIO_SB_ACCESS_2_4B_FLAG;

<<<<<<< HEAD
	status = sdioh_request_word(bcmsdh->sdioh, SDIOH_CMD_TYPE_NORMAL,
				    SDIOH_READ, SDIO_FUNC_1, addr, &word, size);

	bcmsdh->regfail = !(SDIOH_API_SUCCESS(status));

	BCMSDH_INFO(("u32data = 0x%x\n", word));

	/* if ok, return appropriately masked word */
	if (SDIOH_API_SUCCESS(status)) {
=======
	status = brcmf_sdioh_request_word(card->sdioh, SDIOH_CMD_TYPE_NORMAL,
				    SDIOH_READ, SDIO_FUNC_1, addr, &word, size);

	card->regfail = (status != 0);

	BRCMF_SD_INFO(("u32data = 0x%x\n", word));

	/* if ok, return appropriately masked word */
	if (status == 0) {
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		switch (size) {
		case sizeof(u8):
			return word & 0xff;
		case sizeof(u16):
			return word & 0xffff;
		case sizeof(u32):
			return word;
		default:
<<<<<<< HEAD
			bcmsdh->regfail = true;
=======
			card->regfail = true;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

		}
	}

	/* otherwise, bad sdio access or invalid size */
<<<<<<< HEAD
	BCMSDH_ERROR(("%s: error reading addr 0x%04x size %d\n", __func__,
=======
	BRCMF_SD_ERROR(("%s: error reading addr 0x%04x size %d\n", __func__,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		      addr, size));
	return 0xFFFFFFFF;
}

<<<<<<< HEAD
u32 bcmsdh_reg_write(void *sdh, u32 addr, uint size, u32 data)
{
	bcmsdh_info_t *bcmsdh = (bcmsdh_info_t *) sdh;
	SDIOH_API_RC status;
	uint bar0 = addr & ~SBSDIO_SB_OFT_ADDR_MASK;
	int err = 0;

	BCMSDH_INFO(("%s:fun = 1, addr = 0x%x, uint%ddata = 0x%x\n",
		     __func__, addr, size * 8, data));

	if (!bcmsdh)
		bcmsdh = l_bcmsdh;

	ASSERT(bcmsdh->init_success);

	if (bar0 != bcmsdh->sbwad) {
		err = bcmsdhsdio_set_sbaddr_window(bcmsdh, bar0);
		if (err)
			return err;

		bcmsdh->sbwad = bar0;
=======
u32 brcmf_sdcard_reg_write(struct brcmf_sdio_card *card, u32 addr, uint size,
			   u32 data)
{
	int status;
	uint bar0 = addr & ~SBSDIO_SB_OFT_ADDR_MASK;
	int err = 0;

	BRCMF_SD_INFO(("%s:fun = 1, addr = 0x%x, uint%ddata = 0x%x\n",
		     __func__, addr, size * 8, data));

	if (!card)
		card = l_card;

	if (bar0 != card->sbwad) {
		err = brcmf_sdcard_set_sbaddr_window(card, bar0);
		if (err)
			return err;

		card->sbwad = bar0;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	}

	addr &= SBSDIO_SB_OFT_ADDR_MASK;
	if (size == 4)
		addr |= SBSDIO_SB_ACCESS_2_4B_FLAG;
	status =
<<<<<<< HEAD
	    sdioh_request_word(bcmsdh->sdioh, SDIOH_CMD_TYPE_NORMAL,
			       SDIOH_WRITE, SDIO_FUNC_1, addr, &data, size);
	bcmsdh->regfail = !(SDIOH_API_SUCCESS(status));

	if (SDIOH_API_SUCCESS(status))
		return 0;

	BCMSDH_ERROR(("%s: error writing 0x%08x to addr 0x%04x size %d\n",
=======
	    brcmf_sdioh_request_word(card->sdioh, SDIOH_CMD_TYPE_NORMAL,
			       SDIOH_WRITE, SDIO_FUNC_1, addr, &data, size);
	card->regfail = (status != 0);

	if (status == 0)
		return 0;

	BRCMF_SD_ERROR(("%s: error writing 0x%08x to addr 0x%04x size %d\n",
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		      __func__, data, addr, size));
	return 0xFFFFFFFF;
}

<<<<<<< HEAD
bool bcmsdh_regfail(void *sdh)
{
	return ((bcmsdh_info_t *) sdh)->regfail;
}

int
bcmsdh_recv_buf(void *sdh, u32 addr, uint fn, uint flags,
		u8 *buf, uint nbytes, struct sk_buff *pkt,
		bcmsdh_cmplt_fn_t complete, void *handle)
{
	bcmsdh_info_t *bcmsdh = (bcmsdh_info_t *) sdh;
	SDIOH_API_RC status;
=======
bool brcmf_sdcard_regfail(struct brcmf_sdio_card *card)
{
	return card->regfail;
}

int
brcmf_sdcard_recv_buf(struct brcmf_sdio_card *card, u32 addr, uint fn,
		      uint flags,
		      u8 *buf, uint nbytes, struct sk_buff *pkt,
		      void (*complete)(void *handle, int status,
				       bool sync_waiting),
		      void *handle)
{
	int status;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	uint incr_fix;
	uint width;
	uint bar0 = addr & ~SBSDIO_SB_OFT_ADDR_MASK;
	int err = 0;

<<<<<<< HEAD
	ASSERT(bcmsdh);
	ASSERT(bcmsdh->init_success);

	BCMSDH_INFO(("%s:fun = %d, addr = 0x%x, size = %d\n",
		     __func__, fn, addr, nbytes));

	/* Async not implemented yet */
	ASSERT(!(flags & SDIO_REQ_ASYNC));
	if (flags & SDIO_REQ_ASYNC)
		return -ENOTSUPP;

	if (bar0 != bcmsdh->sbwad) {
		err = bcmsdhsdio_set_sbaddr_window(bcmsdh, bar0);
		if (err)
			return err;

		bcmsdh->sbwad = bar0;
=======
	BRCMF_SD_INFO(("%s:fun = %d, addr = 0x%x, size = %d\n",
		     __func__, fn, addr, nbytes));

	/* Async not implemented yet */
	if (flags & SDIO_REQ_ASYNC)
		return -ENOTSUPP;

	if (bar0 != card->sbwad) {
		err = brcmf_sdcard_set_sbaddr_window(card, bar0);
		if (err)
			return err;

		card->sbwad = bar0;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	}

	addr &= SBSDIO_SB_OFT_ADDR_MASK;

	incr_fix = (flags & SDIO_REQ_FIXED) ? SDIOH_DATA_FIX : SDIOH_DATA_INC;
	width = (flags & SDIO_REQ_4BYTE) ? 4 : 2;
	if (width == 4)
		addr |= SBSDIO_SB_ACCESS_2_4B_FLAG;

<<<<<<< HEAD
	status = sdioh_request_buffer(bcmsdh->sdioh, SDIOH_DATA_PIO, incr_fix,
				      SDIOH_READ, fn, addr, width, nbytes, buf,
				      pkt);

	return SDIOH_API_SUCCESS(status) ? 0 : -EIO;
}

int
bcmsdh_send_buf(void *sdh, u32 addr, uint fn, uint flags,
		u8 *buf, uint nbytes, void *pkt,
		bcmsdh_cmplt_fn_t complete, void *handle)
{
	bcmsdh_info_t *bcmsdh = (bcmsdh_info_t *) sdh;
	SDIOH_API_RC status;
=======
	status = brcmf_sdioh_request_buffer(card->sdioh, SDIOH_DATA_PIO,
		incr_fix, SDIOH_READ, fn, addr, width, nbytes, buf, pkt);

	return status;
}

int
brcmf_sdcard_send_buf(struct brcmf_sdio_card *card, u32 addr, uint fn,
		      uint flags, u8 *buf, uint nbytes, void *pkt,
		      void (*complete)(void *handle, int status,
				       bool sync_waiting),
		      void *handle)
{
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	uint incr_fix;
	uint width;
	uint bar0 = addr & ~SBSDIO_SB_OFT_ADDR_MASK;
	int err = 0;

<<<<<<< HEAD
	ASSERT(bcmsdh);
	ASSERT(bcmsdh->init_success);

	BCMSDH_INFO(("%s:fun = %d, addr = 0x%x, size = %d\n",
		     __func__, fn, addr, nbytes));

	/* Async not implemented yet */
	ASSERT(!(flags & SDIO_REQ_ASYNC));
	if (flags & SDIO_REQ_ASYNC)
		return -ENOTSUPP;

	if (bar0 != bcmsdh->sbwad) {
		err = bcmsdhsdio_set_sbaddr_window(bcmsdh, bar0);
		if (err)
			return err;

		bcmsdh->sbwad = bar0;
=======
	BRCMF_SD_INFO(("%s:fun = %d, addr = 0x%x, size = %d\n",
		     __func__, fn, addr, nbytes));

	/* Async not implemented yet */
	if (flags & SDIO_REQ_ASYNC)
		return -ENOTSUPP;

	if (bar0 != card->sbwad) {
		err = brcmf_sdcard_set_sbaddr_window(card, bar0);
		if (err)
			return err;

		card->sbwad = bar0;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	}

	addr &= SBSDIO_SB_OFT_ADDR_MASK;

	incr_fix = (flags & SDIO_REQ_FIXED) ? SDIOH_DATA_FIX : SDIOH_DATA_INC;
	width = (flags & SDIO_REQ_4BYTE) ? 4 : 2;
	if (width == 4)
		addr |= SBSDIO_SB_ACCESS_2_4B_FLAG;

<<<<<<< HEAD
	status = sdioh_request_buffer(bcmsdh->sdioh, SDIOH_DATA_PIO, incr_fix,
				      SDIOH_WRITE, fn, addr, width, nbytes, buf,
				      pkt);

	return SDIOH_API_SUCCESS(status) ? 0 : -EIO;
}

int bcmsdh_rwdata(void *sdh, uint rw, u32 addr, u8 *buf, uint nbytes)
{
	bcmsdh_info_t *bcmsdh = (bcmsdh_info_t *) sdh;
	SDIOH_API_RC status;

	ASSERT(bcmsdh);
	ASSERT(bcmsdh->init_success);
	ASSERT((addr & SBSDIO_SBWINDOW_MASK) == 0);

	addr &= SBSDIO_SB_OFT_ADDR_MASK;
	addr |= SBSDIO_SB_ACCESS_2_4B_FLAG;

	status =
	    sdioh_request_buffer(bcmsdh->sdioh, SDIOH_DATA_PIO, SDIOH_DATA_INC,
				 (rw ? SDIOH_WRITE : SDIOH_READ), SDIO_FUNC_1,
				 addr, 4, nbytes, buf, NULL);

	return SDIOH_API_SUCCESS(status) ? 0 : -EIO;
}

int bcmsdh_abort(void *sdh, uint fn)
{
	bcmsdh_info_t *bcmsdh = (bcmsdh_info_t *) sdh;

	return sdioh_abort(bcmsdh->sdioh, fn);
}

int bcmsdh_start(void *sdh, int stage)
{
	bcmsdh_info_t *bcmsdh = (bcmsdh_info_t *) sdh;

	return sdioh_start(bcmsdh->sdioh, stage);
}

int bcmsdh_stop(void *sdh)
{
	bcmsdh_info_t *bcmsdh = (bcmsdh_info_t *) sdh;

	return sdioh_stop(bcmsdh->sdioh);
}

int bcmsdh_query_device(void *sdh)
{
	bcmsdh_info_t *bcmsdh = (bcmsdh_info_t *) sdh;
	bcmsdh->vendevid = (PCI_VENDOR_ID_BROADCOM << 16) | 0;
	return bcmsdh->vendevid;
}

uint bcmsdh_query_iofnum(void *sdh)
{
	bcmsdh_info_t *bcmsdh = (bcmsdh_info_t *) sdh;

	if (!bcmsdh)
		bcmsdh = l_bcmsdh;

	return sdioh_query_iofnum(bcmsdh->sdioh);
}

int bcmsdh_reset(bcmsdh_info_t *sdh)
{
	bcmsdh_info_t *bcmsdh = (bcmsdh_info_t *) sdh;

	return sdioh_sdio_reset(bcmsdh->sdioh);
}

void *bcmsdh_get_sdioh(bcmsdh_info_t *sdh)
{
	ASSERT(sdh);
	return sdh->sdioh;
}

/* Function to pass device-status bits to DHD. */
u32 bcmsdh_get_dstatus(void *sdh)
{
	return 0;
}

u32 bcmsdh_cur_sbwad(void *sdh)
{
	bcmsdh_info_t *bcmsdh = (bcmsdh_info_t *) sdh;

	if (!bcmsdh)
		bcmsdh = l_bcmsdh;

	return bcmsdh->sbwad;
}

void bcmsdh_chipinfo(void *sdh, u32 chip, u32 chiprev)
{
	return;
=======
	return brcmf_sdioh_request_buffer(card->sdioh, SDIOH_DATA_PIO,
		incr_fix, SDIOH_WRITE, fn, addr, width, nbytes, buf, pkt);
}

int brcmf_sdcard_rwdata(struct brcmf_sdio_card *card, uint rw, u32 addr,
			u8 *buf, uint nbytes)
{
	addr &= SBSDIO_SB_OFT_ADDR_MASK;
	addr |= SBSDIO_SB_ACCESS_2_4B_FLAG;

	return brcmf_sdioh_request_buffer(card->sdioh, SDIOH_DATA_PIO,
		SDIOH_DATA_INC, (rw ? SDIOH_WRITE : SDIOH_READ), SDIO_FUNC_1,
		addr, 4, nbytes, buf, NULL);
}

int brcmf_sdcard_abort(struct brcmf_sdio_card *card, uint fn)
{
	return brcmf_sdioh_abort(card->sdioh, fn);
}

int brcmf_sdcard_query_device(struct brcmf_sdio_card *card)
{
	card->vendevid = (PCI_VENDOR_ID_BROADCOM << 16) | 0;
	return card->vendevid;
}

u32 brcmf_sdcard_cur_sbwad(struct brcmf_sdio_card *card)
{
	if (!card)
		card = l_card;

	return card->sbwad;
}

int brcmf_sdio_probe(struct device *dev)
{
	struct sdio_hc *sdhc = NULL;
	u32 regs = 0;
	struct brcmf_sdio_card *card = NULL;
	int irq = 0;
	u32 vendevid;
	unsigned long irq_flags = 0;

	/* allocate SDIO Host Controller state info */
	sdhc = kzalloc(sizeof(struct sdio_hc), GFP_ATOMIC);
	if (!sdhc) {
		SDLX_MSG(("%s: out of memory\n", __func__));
		goto err;
	}
	sdhc->dev = (void *)dev;

	card = brcmf_sdcard_attach((void *)0, &regs, irq);
	if (!card) {
		SDLX_MSG(("%s: attach failed\n", __func__));
		goto err;
	}

	sdhc->card = card;
	sdhc->oob_irq = irq;
	sdhc->oob_flags = irq_flags;
	sdhc->oob_irq_registered = false;	/* to make sure.. */

	/* chain SDIO Host Controller info together */
	sdhc->next = sdhcinfo;
	sdhcinfo = sdhc;
	/* Read the vendor/device ID from the CIS */
	vendevid = brcmf_sdcard_query_device(card);

	/* try to attach to the target device */
	sdhc->ch = drvinfo.attach((vendevid >> 16), (vendevid & 0xFFFF),
				  0, 0, 0, 0, regs, card);
	if (!sdhc->ch) {
		SDLX_MSG(("%s: device attach failed\n", __func__));
		goto err;
	}

	return 0;

	/* error handling */
err:
	if (sdhc) {
		if (sdhc->card)
			brcmf_sdcard_detach(sdhc->card);
		kfree(sdhc);
	}

	return -ENODEV;
}

int brcmf_sdio_remove(struct device *dev)
{
	struct sdio_hc *sdhc, *prev;

	sdhc = sdhcinfo;
	drvinfo.detach(sdhc->ch);
	brcmf_sdcard_detach(sdhc->card);
	/* find the SDIO Host Controller state for this pdev
		 and take it out from the list */
	for (sdhc = sdhcinfo, prev = NULL; sdhc; sdhc = sdhc->next) {
		if (sdhc->dev == (void *)dev) {
			if (prev)
				prev->next = sdhc->next;
			else
				sdhcinfo = NULL;
			break;
		}
		prev = sdhc;
	}
	if (!sdhc) {
		SDLX_MSG(("%s: failed\n", __func__));
		return 0;
	}

	/* release SDIO Host Controller info */
	kfree(sdhc);
	return 0;
}

int brcmf_sdio_register(struct brcmf_sdioh_driver *driver)
{
	drvinfo = *driver;

	SDLX_MSG(("Linux Kernel SDIO/MMC Driver\n"));
	return brcmf_sdio_function_init();
}

void brcmf_sdio_unregister(void)
{
	brcmf_sdio_function_cleanup();
}

void brcmf_sdio_wdtmr_enable(bool enable)
{
	if (enable)
		brcmf_sdbrcm_wd_timer(sdhcinfo->ch, brcmf_watchdog_ms);
	else
		brcmf_sdbrcm_wd_timer(sdhcinfo->ch, 0);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
}
