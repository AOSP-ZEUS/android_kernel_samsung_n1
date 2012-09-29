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

#include <linux/types.h>
#include <linux/kernel.h>
<<<<<<< HEAD
#include <linux/printk.h>
#include <linux/pci_ids.h>
#include <linux/netdevice.h>
#include <bcmdefs.h>
#include <bcmsdh.h>

#ifdef BCMEMBEDIMAGE
#include BCMEMBEDIMAGE
#endif				/* BCMEMBEDIMAGE */

#include <bcmdefs.h>
#include <bcmutils.h>
#include <bcmdevs.h>

#include <hndsoc.h>
#ifdef DHD_DEBUG
#include <hndrte_armtrap.h>
#include <hndrte_cons.h>
#endif				/* DHD_DEBUG */
#include <sbchipc.h>
#include <sbhnddma.h>

#include <sdio.h>
#include <sbsdio.h>
#include <sbsdpcmdev.h>
#include <bcmsdpcm.h>

#include <proto/802.11.h>

#include <dngl_stats.h>
#include <dhd.h>
#include <dhd_bus.h>
#include <dhd_proto.h>
#include <dhd_dbg.h>
#include <dhdioctl.h>
#include <sdiovar.h>
#include <bcmchip.h>

#ifndef DHDSDIO_MEM_DUMP_FNAME
#define DHDSDIO_MEM_DUMP_FNAME         "mem_dump"
#endif
=======
#include <linux/kthread.h>
#include <linux/printk.h>
#include <linux/pci_ids.h>
#include <linux/netdevice.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/semaphore.h>
#include <linux/firmware.h>
#include <asm/unaligned.h>
#include <defs.h>
#include <brcmu_wifi.h>
#include <brcmu_utils.h>
#include <brcm_hw_ids.h>
#include <soc.h>
#include "sdio_host.h"

/* register access macros */
#ifndef __BIG_ENDIAN
#ifndef __mips__
#define R_REG(r, typ) \
	brcmf_sdcard_reg_read(NULL, (r), sizeof(typ))
#else				/* __mips__ */
#define R_REG(r, typ) \
	({ \
		__typeof(*(r)) __osl_v; \
		__asm__ __volatile__("sync"); \
		__osl_v = brcmf_sdcard_reg_read(NULL, (r),\
					  sizeof(typ)); \
		__asm__ __volatile__("sync"); \
		__osl_v; \
	})
#endif				/* __mips__ */

#else				/* __BIG_ENDIAN */
#define R_REG(r, typ) \
	brcmf_sdcard_reg_read(NULL, (r), sizeof(typ))
#endif				/* __BIG_ENDIAN */

#define OR_REG(r, v, typ) \
	brcmf_sdcard_reg_write(NULL, (r), sizeof(typ), R_REG(r, typ) | (v))

#ifdef BCMDBG

/* ARM trap handling */

/* Trap types defined by ARM (see arminc.h) */

#if defined(__ARM_ARCH_4T__)
#define	MAX_TRAP_TYPE	(TR_FIQ + 1)
#elif defined(__ARM_ARCH_7M__)
#define	MAX_TRAP_TYPE	(TR_ISR + ARMCM3_NUMINTS)
#endif				/* __ARM_ARCH_7M__ */

/* The trap structure is defined here as offsets for assembly */
#define	TR_TYPE		0x00
#define	TR_EPC		0x04
#define	TR_CPSR		0x08
#define	TR_SPSR		0x0c
#define	TR_REGS		0x10
#define	TR_REG(n)	(TR_REGS + (n) * 4)
#define	TR_SP		TR_REG(13)
#define	TR_LR		TR_REG(14)
#define	TR_PC		TR_REG(15)

#define	TRAP_T_SIZE	80

struct brcmf_trap {
	u32 type;
	u32 epc;
	u32 cpsr;
	u32 spsr;
	u32 r0;
	u32 r1;
	u32 r2;
	u32 r3;
	u32 r4;
	u32 r5;
	u32 r6;
	u32 r7;
	u32 r8;
	u32 r9;
	u32 r10;
	u32 r11;
	u32 r12;
	u32 r13;
	u32 r14;
	u32 pc;
};

#define CBUF_LEN	(128)

struct rte_log {
	u32 buf;		/* Can't be pointer on (64-bit) hosts */
	uint buf_size;
	uint idx;
	char *_buf_compat;	/* Redundant pointer for backward compat. */
};

struct rte_console {
	/* Virtual UART
	 * When there is no UART (e.g. Quickturn),
	 * the host should write a complete
	 * input line directly into cbuf and then write
	 * the length into vcons_in.
	 * This may also be used when there is a real UART
	 * (at risk of conflicting with
	 * the real UART).  vcons_out is currently unused.
	 */
	volatile uint vcons_in;
	volatile uint vcons_out;

	/* Output (logging) buffer
	 * Console output is written to a ring buffer log_buf at index log_idx.
	 * The host may read the output when it sees log_idx advance.
	 * Output will be lost if the output wraps around faster than the host
	 * polls.
	 */
	struct rte_log log;

	/* Console input line buffer
	 * Characters are read one at a time into cbuf
	 * until <CR> is received, then
	 * the buffer is processed as a command line.
	 * Also used for virtual UART.
	 */
	uint cbuf_idx;
	char cbuf[CBUF_LEN];
};

#endif				/* BCMDBG */
#include <chipcommon.h>

#include "dhd.h"
#include "dhd_bus.h"
#include "dhd_proto.h"
#include "dhd_dbg.h"
#include <bcmchip.h>
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

#define TXQLEN		2048	/* bulk tx queue length */
#define TXHI		(TXQLEN - 256)	/* turn on flow control above TXHI */
#define TXLOW		(TXHI - 256)	/* turn off flow control below TXLOW */
#define PRIOMASK	7

#define TXRETRIES	2	/* # of retries for tx frames */

<<<<<<< HEAD
#if defined(CONFIG_MACH_SANDGATE2G)
#define DHD_RXBOUND	250	/* Default for max rx frames in
				 one scheduling */
#else
#define DHD_RXBOUND	50	/* Default for max rx frames in
				 one scheduling */
#endif				/* defined(CONFIG_MACH_SANDGATE2G) */

#define DHD_TXBOUND	20	/* Default for max tx frames in
				 one scheduling */

#define DHD_TXMINMAX	1	/* Max tx frames if rx still pending */
=======
#define BRCMF_RXBOUND	50	/* Default for max rx frames in
				 one scheduling */

#define BRCMF_TXBOUND	20	/* Default for max tx frames in
				 one scheduling */

#define BRCMF_TXMINMAX	1	/* Max tx frames if rx still pending */
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

#define MEMBLOCK	2048	/* Block size used for downloading
				 of dongle image */
#define MAX_DATA_BUF	(32 * 1024)	/* Must be large enough to hold
				 biggest possible glom */

<<<<<<< HEAD
/* Packet alignment for most efficient SDIO (can change based on platform) */
#ifndef DHD_SDALIGN
#define DHD_SDALIGN	32
#endif
#if !ISPOWEROF2(DHD_SDALIGN)
#error DHD_SDALIGN is not a power of 2!
#endif

#ifndef DHD_FIRSTREAD
#define DHD_FIRSTREAD	32
#endif
#if !ISPOWEROF2(DHD_FIRSTREAD)
#error DHD_FIRSTREAD is not a power of 2!
#endif

/* Total length of frame header for dongle protocol */
#define SDPCM_HDRLEN	(SDPCM_FRAMETAG_LEN + SDPCM_SWHEADER_LEN)
#ifdef SDTEST
#define SDPCM_RESERVE	(SDPCM_HDRLEN + SDPCM_TEST_HDRLEN + DHD_SDALIGN)
#else
#define SDPCM_RESERVE	(SDPCM_HDRLEN + DHD_SDALIGN)
#endif

=======
#ifndef BRCMF_FIRSTREAD
#define BRCMF_FIRSTREAD	32
#endif

#if !ISPOWEROF2(BRCMF_FIRSTREAD)
#error BRCMF_FIRSTREAD is not a power of 2!
#endif

/* SBSDIO_DEVICE_CTL */
#define SBSDIO_DEVCTL_SETBUSY		0x01	/* 1: device will assert busy signal when
						 * receiving CMD53
						 */
#define SBSDIO_DEVCTL_SPI_INTR_SYNC	0x02	/* 1: assertion of sdio interrupt is
						 * synchronous to the sdio clock
						 */
#define SBSDIO_DEVCTL_CA_INT_ONLY	0x04	/* 1: mask all interrupts to host
						 * except the chipActive (rev 8)
						 */
#define SBSDIO_DEVCTL_PADS_ISO		0x08	/* 1: isolate internal sdio signals, put
						 * external pads in tri-state; requires
						 * sdio bus power cycle to clear (rev 9)
						 */
#define SBSDIO_DEVCTL_SB_RST_CTL	0x30	/* Force SD->SB reset mapping (rev 11) */
#define SBSDIO_DEVCTL_RST_CORECTL	0x00	/*   Determined by CoreControl bit */
#define SBSDIO_DEVCTL_RST_BPRESET	0x10	/*   Force backplane reset */
#define SBSDIO_DEVCTL_RST_NOBPRESET	0x20	/*   Force no backplane reset */

/* SBSDIO_FUNC1_CHIPCLKCSR */
#define SBSDIO_FORCE_ALP		0x01	/* Force ALP request to backplane */
#define SBSDIO_FORCE_HT			0x02	/* Force HT request to backplane */
#define SBSDIO_FORCE_ILP		0x04	/* Force ILP request to backplane */
#define SBSDIO_ALP_AVAIL_REQ		0x08	/* Make ALP ready (power up xtal) */
#define SBSDIO_HT_AVAIL_REQ		0x10	/* Make HT ready (power up PLL) */
#define SBSDIO_FORCE_HW_CLKREQ_OFF	0x20	/* Squelch clock requests from HW */
#define SBSDIO_ALP_AVAIL		0x40	/* Status: ALP is ready */
#define SBSDIO_HT_AVAIL			0x80	/* Status: HT is ready */

#define SBSDIO_AVBITS			(SBSDIO_HT_AVAIL | SBSDIO_ALP_AVAIL)
#define SBSDIO_ALPAV(regval)		((regval) & SBSDIO_AVBITS)
#define SBSDIO_HTAV(regval)		(((regval) & SBSDIO_AVBITS) == SBSDIO_AVBITS)
#define SBSDIO_ALPONLY(regval)		(SBSDIO_ALPAV(regval) && !SBSDIO_HTAV(regval))
#define SBSDIO_CLKAV(regval, alponly)	(SBSDIO_ALPAV(regval) && \
					(alponly ? 1 : SBSDIO_HTAV(regval)))
/* direct(mapped) cis space */
#define SBSDIO_CIS_BASE_COMMON		0x1000	/* MAPPED common CIS address */
#define SBSDIO_CIS_SIZE_LIMIT		0x200	/* maximum bytes in one CIS */
#define SBSDIO_CIS_OFT_ADDR_MASK	0x1FFFF	/* cis offset addr is < 17 bits */

#define SBSDIO_CIS_MANFID_TUPLE_LEN	6	/* manfid tuple length, include tuple,
						 * link bytes
						 */

/* intstatus */
#define I_SMB_SW0	(1 << 0)	/* To SB Mail S/W interrupt 0 */
#define I_SMB_SW1	(1 << 1)	/* To SB Mail S/W interrupt 1 */
#define I_SMB_SW2	(1 << 2)	/* To SB Mail S/W interrupt 2 */
#define I_SMB_SW3	(1 << 3)	/* To SB Mail S/W interrupt 3 */
#define I_SMB_SW_MASK	0x0000000f	/* To SB Mail S/W interrupts mask */
#define I_SMB_SW_SHIFT	0	/* To SB Mail S/W interrupts shift */
#define I_HMB_SW0	(1 << 4)	/* To Host Mail S/W interrupt 0 */
#define I_HMB_SW1	(1 << 5)	/* To Host Mail S/W interrupt 1 */
#define I_HMB_SW2	(1 << 6)	/* To Host Mail S/W interrupt 2 */
#define I_HMB_SW3	(1 << 7)	/* To Host Mail S/W interrupt 3 */
#define I_HMB_SW_MASK	0x000000f0	/* To Host Mail S/W interrupts mask */
#define I_HMB_SW_SHIFT	4	/* To Host Mail S/W interrupts shift */
#define I_WR_OOSYNC	(1 << 8)	/* Write Frame Out Of Sync */
#define I_RD_OOSYNC	(1 << 9)	/* Read Frame Out Of Sync */
#define	I_PC		(1 << 10)	/* descriptor error */
#define	I_PD		(1 << 11)	/* data error */
#define	I_DE		(1 << 12)	/* Descriptor protocol Error */
#define	I_RU		(1 << 13)	/* Receive descriptor Underflow */
#define	I_RO		(1 << 14)	/* Receive fifo Overflow */
#define	I_XU		(1 << 15)	/* Transmit fifo Underflow */
#define	I_RI		(1 << 16)	/* Receive Interrupt */
#define I_BUSPWR	(1 << 17)	/* SDIO Bus Power Change (rev 9) */
#define I_XMTDATA_AVAIL (1 << 23)	/* bits in fifo */
#define	I_XI		(1 << 24)	/* Transmit Interrupt */
#define I_RF_TERM	(1 << 25)	/* Read Frame Terminate */
#define I_WF_TERM	(1 << 26)	/* Write Frame Terminate */
#define I_PCMCIA_XU	(1 << 27)	/* PCMCIA Transmit FIFO Underflow */
#define I_SBINT		(1 << 28)	/* sbintstatus Interrupt */
#define I_CHIPACTIVE	(1 << 29)	/* chip from doze to active state */
#define I_SRESET	(1 << 30)	/* CCCR RES interrupt */
#define I_IOE2		(1U << 31)	/* CCCR IOE2 Bit Changed */
#define	I_ERRORS	(I_PC | I_PD | I_DE | I_RU | I_RO | I_XU)
#define I_DMA		(I_RI | I_XI | I_ERRORS)

/* corecontrol */
#define CC_CISRDY		(1 << 0)	/* CIS Ready */
#define CC_BPRESEN		(1 << 1)	/* CCCR RES signal */
#define CC_F2RDY		(1 << 2)	/* set CCCR IOR2 bit */
#define CC_CLRPADSISO		(1 << 3)	/* clear SDIO pads isolation */
#define CC_XMTDATAAVAIL_MODE	(1 << 4)
#define CC_XMTDATAAVAIL_CTRL	(1 << 5)

/* SDA_FRAMECTRL */
#define SFC_RF_TERM	(1 << 0)	/* Read Frame Terminate */
#define SFC_WF_TERM	(1 << 1)	/* Write Frame Terminate */
#define SFC_CRC4WOOS	(1 << 2)	/* CRC error for write out of sync */
#define SFC_ABORTALL	(1 << 3)	/* Abort all in-progress frames */

/* HW frame tag */
#define SDPCM_FRAMETAG_LEN	4	/* 2 bytes len, 2 bytes check val */

/* Total length of frame header for dongle protocol */
#define SDPCM_HDRLEN	(SDPCM_FRAMETAG_LEN + SDPCM_SWHEADER_LEN)
#ifdef SDTEST
#define SDPCM_RESERVE	(SDPCM_HDRLEN + SDPCM_TEST_HDRLEN + BRCMF_SDALIGN)
#else
#define SDPCM_RESERVE	(SDPCM_HDRLEN + BRCMF_SDALIGN)
#endif

/*
 * Software allocation of To SB Mailbox resources
 */

/* tosbmailbox bits corresponding to intstatus bits */
#define SMB_NAK		(1 << 0)	/* Frame NAK */
#define SMB_INT_ACK	(1 << 1)	/* Host Interrupt ACK */
#define SMB_USE_OOB	(1 << 2)	/* Use OOB Wakeup */
#define SMB_DEV_INT	(1 << 3)	/* Miscellaneous Interrupt */

/* tosbmailboxdata */
#define SMB_DATA_VERSION_SHIFT	16	/* host protocol version */

/*
 * Software allocation of To Host Mailbox resources
 */

/* intstatus bits */
#define I_HMB_FC_STATE	I_HMB_SW0	/* Flow Control State */
#define I_HMB_FC_CHANGE	I_HMB_SW1	/* Flow Control State Changed */
#define I_HMB_FRAME_IND	I_HMB_SW2	/* Frame Indication */
#define I_HMB_HOST_INT	I_HMB_SW3	/* Miscellaneous Interrupt */

/* tohostmailboxdata */
#define HMB_DATA_NAKHANDLED	1	/* retransmit NAK'd frame */
#define HMB_DATA_DEVREADY	2	/* talk to host after enable */
#define HMB_DATA_FC		4	/* per prio flowcontrol update flag */
#define HMB_DATA_FWREADY	8	/* fw ready for protocol activity */

#define HMB_DATA_FCDATA_MASK	0xff000000
#define HMB_DATA_FCDATA_SHIFT	24

#define HMB_DATA_VERSION_MASK	0x00ff0000
#define HMB_DATA_VERSION_SHIFT	16

/*
 * Software-defined protocol header
 */

/* Current protocol version */
#define SDPCM_PROT_VERSION	4

/* SW frame header */
#define SDPCM_PACKET_SEQUENCE(p)	(((u8 *)p)[0] & 0xff)

#define SDPCM_CHANNEL_MASK		0x00000f00
#define SDPCM_CHANNEL_SHIFT		8
#define SDPCM_PACKET_CHANNEL(p)		(((u8 *)p)[1] & 0x0f)

#define SDPCM_NEXTLEN_OFFSET		2

/* Data Offset from SOF (HW Tag, SW Tag, Pad) */
#define SDPCM_DOFFSET_OFFSET		3	/* Data Offset */
#define SDPCM_DOFFSET_VALUE(p)		(((u8 *)p)[SDPCM_DOFFSET_OFFSET] & 0xff)
#define SDPCM_DOFFSET_MASK		0xff000000
#define SDPCM_DOFFSET_SHIFT		24
#define SDPCM_FCMASK_OFFSET		4	/* Flow control */
#define SDPCM_FCMASK_VALUE(p)		(((u8 *)p)[SDPCM_FCMASK_OFFSET] & 0xff)
#define SDPCM_WINDOW_OFFSET		5	/* Credit based fc */
#define SDPCM_WINDOW_VALUE(p)		(((u8 *)p)[SDPCM_WINDOW_OFFSET] & 0xff)

#define SDPCM_SWHEADER_LEN	8	/* SW header is 64 bits */

/* logical channel numbers */
#define SDPCM_CONTROL_CHANNEL	0	/* Control channel Id */
#define SDPCM_EVENT_CHANNEL	1	/* Asyc Event Indication Channel Id */
#define SDPCM_DATA_CHANNEL	2	/* Data Xmit/Recv Channel Id */
#define SDPCM_GLOM_CHANNEL	3	/* For coalesced packets */
#define SDPCM_TEST_CHANNEL	15	/* Reserved for test/debug packets */

#define SDPCM_SEQUENCE_WRAP	256	/* wrap-around val for 8bit frame seq */

#define SDPCM_GLOMDESC(p)	(((u8 *)p)[1] & 0x80)

/* For TEST_CHANNEL packets, define another 4-byte header */
#define SDPCM_TEST_HDRLEN	4	/*
					 * Generally: Cmd(1), Ext(1), Len(2);
					 * Semantics of Ext byte depend on
					 * command. Len is current or requested
					 * frame length, not including test
					 * header; sent little-endian.
					 */
#define SDPCM_TEST_DISCARD	0x01	/* Receiver discards. Ext:pattern id. */
#define SDPCM_TEST_ECHOREQ	0x02	/* Echo request. Ext:pattern id. */
#define SDPCM_TEST_ECHORSP	0x03	/* Echo response. Ext:pattern id. */
#define SDPCM_TEST_BURST	0x04	/*
					 * Receiver to send a burst.
					 * Ext is a frame count
					 */
#define SDPCM_TEST_SEND		0x05	/*
					 * Receiver sets send mode.
					 * Ext is boolean on/off
					 */

/* Handy macro for filling in datagen packets with a pattern */
#define SDPCM_TEST_FILL(byteno, id)	((u8)(id + byteno))

/*
 * Shared structure between dongle and the host.
 * The structure contains pointers to trap or assert information.
 */
#define SDPCM_SHARED_VERSION       0x0002
#define SDPCM_SHARED_VERSION_MASK  0x00FF
#define SDPCM_SHARED_ASSERT_BUILT  0x0100
#define SDPCM_SHARED_ASSERT        0x0200
#define SDPCM_SHARED_TRAP          0x0400


>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
/* Space for header read, limit for data packets */
#ifndef MAX_HDR_READ
#define MAX_HDR_READ	32
#endif
#if !ISPOWEROF2(MAX_HDR_READ)
#error MAX_HDR_READ is not a power of 2!
#endif

#define MAX_RX_DATASZ	2048

/* Maximum milliseconds to wait for F2 to come up */
<<<<<<< HEAD
#define DHD_WAIT_F2RDY	3000
=======
#define BRCMF_WAIT_F2RDY	3000
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

/* Bump up limit on waiting for HT to account for first startup;
 * if the image is doing a CRC calculation before programming the PMU
 * for HT availability, it could take a couple hundred ms more, so
 * max out at a 1 second (1000000us).
 */
#if (PMU_MAX_TRANSITION_DLY <= 1000000)
#undef PMU_MAX_TRANSITION_DLY
#define PMU_MAX_TRANSITION_DLY 1000000
#endif

/* Value for ChipClockCSR during initial setup */
<<<<<<< HEAD
#define DHD_INIT_CLKCTL1	(SBSDIO_FORCE_HW_CLKREQ_OFF |	\
					SBSDIO_ALP_AVAIL_REQ)
#define DHD_INIT_CLKCTL2	(SBSDIO_FORCE_HW_CLKREQ_OFF | SBSDIO_FORCE_ALP)
=======
#define BRCMF_INIT_CLKCTL1	(SBSDIO_FORCE_HW_CLKREQ_OFF |	\
					SBSDIO_ALP_AVAIL_REQ)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

/* Flags for SDH calls */
#define F2SYNC	(SDIO_REQ_4BYTE | SDIO_REQ_FIXED)

<<<<<<< HEAD
=======
/* sbimstate */
#define	SBIM_IBE		0x20000	/* inbanderror */
#define	SBIM_TO			0x40000	/* timeout */
#define	SBIM_BY			0x01800000	/* busy (sonics >= 2.3) */
#define	SBIM_RJ			0x02000000	/* reject (sonics >= 2.3) */

/* sbtmstatelow */
#define	SBTML_RESET		0x0001	/* reset */
#define	SBTML_REJ_MASK		0x0006	/* reject field */
#define	SBTML_REJ		0x0002	/* reject */
#define	SBTML_TMPREJ		0x0004	/* temporary reject, for error recovery */

#define	SBTML_SICF_SHIFT	16	/* Shift to locate the SI control flags in sbtml */

/* sbtmstatehigh */
#define	SBTMH_SERR		0x0001	/* serror */
#define	SBTMH_INT		0x0002	/* interrupt */
#define	SBTMH_BUSY		0x0004	/* busy */
#define	SBTMH_TO		0x0020	/* timeout (sonics >= 2.3) */

#define	SBTMH_SISF_SHIFT	16	/* Shift to locate the SI status flags in sbtmh */

/* sbidlow */
#define	SBIDL_INIT		0x80	/* initiator */

/* sbidhigh */
#define	SBIDH_RC_MASK		0x000f	/* revision code */
#define	SBIDH_RCE_MASK		0x7000	/* revision code extension field */
#define	SBIDH_RCE_SHIFT		8
#define	SBCOREREV(sbidh) \
	((((sbidh) & SBIDH_RCE_MASK) >> SBIDH_RCE_SHIFT) | ((sbidh) & SBIDH_RC_MASK))
#define	SBIDH_CC_MASK		0x8ff0	/* core code */
#define	SBIDH_CC_SHIFT		4
#define	SBIDH_VC_MASK		0xffff0000	/* vendor code */
#define	SBIDH_VC_SHIFT		16

>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
/*
 * Conversion of 802.1D priority to precedence level
 */
#define PRIO2PREC(prio) \
	(((prio) == PRIO_8021D_NONE || (prio) == PRIO_8021D_BE) ? \
	((prio^2)) : (prio))

<<<<<<< HEAD
DHD_SPINWAIT_SLEEP_INIT(sdioh_spinwait_sleep);
extern int dhdcdc_set_ioctl(dhd_pub_t *dhd, int ifidx, uint cmd, void *buf,
			    uint len);

#ifdef DHD_DEBUG
/* Device console log buffer state */
typedef struct dhd_console {
	uint count;		/* Poll interval msec counter */
	uint log_addr;		/* Log struct address (fixed) */
	hndrte_log_t log;	/* Log struct (host copy) */
	uint bufsize;		/* Size of log buffer */
	u8 *buf;		/* Log buffer (host copy) */
	uint last;		/* Last buffer read index */
} dhd_console_t;
#endif				/* DHD_DEBUG */
=======
BRCMF_SPINWAIT_SLEEP_INIT(sdioh_spinwait_sleep);

/*
 * Core reg address translation.
 * Both macro's returns a 32 bits byte address on the backplane bus.
 */
#define CORE_CC_REG(base, field)	(base + offsetof(chipcregs_t, field))
#define CORE_BUS_REG(base, field) \
		(base + offsetof(struct sdpcmd_regs, field))
#define CORE_SB(base, field) \
		(base + SBCONFIGOFF + offsetof(struct sbconfig, field))

/* core registers */
struct sdpcmd_regs {
	u32 corecontrol;		/* 0x00, rev8 */
	u32 corestatus;			/* rev8 */
	u32 PAD[1];
	u32 biststatus;			/* rev8 */

	/* PCMCIA access */
	u16 pcmciamesportaladdr;	/* 0x010, rev8 */
	u16 PAD[1];
	u16 pcmciamesportalmask;	/* rev8 */
	u16 PAD[1];
	u16 pcmciawrframebc;		/* rev8 */
	u16 PAD[1];
	u16 pcmciaunderflowtimer;	/* rev8 */
	u16 PAD[1];

	/* interrupt */
	u32 intstatus;			/* 0x020, rev8 */
	u32 hostintmask;		/* rev8 */
	u32 intmask;			/* rev8 */
	u32 sbintstatus;		/* rev8 */
	u32 sbintmask;			/* rev8 */
	u32 funcintmask;		/* rev4 */
	u32 PAD[2];
	u32 tosbmailbox;		/* 0x040, rev8 */
	u32 tohostmailbox;		/* rev8 */
	u32 tosbmailboxdata;		/* rev8 */
	u32 tohostmailboxdata;		/* rev8 */

	/* synchronized access to registers in SDIO clock domain */
	u32 sdioaccess;			/* 0x050, rev8 */
	u32 PAD[3];

	/* PCMCIA frame control */
	u8 pcmciaframectrl;		/* 0x060, rev8 */
	u8 PAD[3];
	u8 pcmciawatermark;		/* rev8 */
	u8 PAD[155];

	/* interrupt batching control */
	u32 intrcvlazy;			/* 0x100, rev8 */
	u32 PAD[3];

	/* counters */
	u32 cmd52rd;			/* 0x110, rev8 */
	u32 cmd52wr;			/* rev8 */
	u32 cmd53rd;			/* rev8 */
	u32 cmd53wr;			/* rev8 */
	u32 abort;			/* rev8 */
	u32 datacrcerror;		/* rev8 */
	u32 rdoutofsync;		/* rev8 */
	u32 wroutofsync;		/* rev8 */
	u32 writebusy;			/* rev8 */
	u32 readwait;			/* rev8 */
	u32 readterm;			/* rev8 */
	u32 writeterm;			/* rev8 */
	u32 PAD[40];
	u32 clockctlstatus;		/* rev8 */
	u32 PAD[7];

	u32 PAD[128];			/* DMA engines */

	/* SDIO/PCMCIA CIS region */
	char cis[512];			/* 0x400-0x5ff, rev6 */

	/* PCMCIA function control registers */
	char pcmciafcr[256];		/* 0x600-6ff, rev6 */
	u16 PAD[55];

	/* PCMCIA backplane access */
	u16 backplanecsr;		/* 0x76E, rev6 */
	u16 backplaneaddr0;		/* rev6 */
	u16 backplaneaddr1;		/* rev6 */
	u16 backplaneaddr2;		/* rev6 */
	u16 backplaneaddr3;		/* rev6 */
	u16 backplanedata0;		/* rev6 */
	u16 backplanedata1;		/* rev6 */
	u16 backplanedata2;		/* rev6 */
	u16 backplanedata3;		/* rev6 */
	u16 PAD[31];

	/* sprom "size" & "blank" info */
	u16 spromstatus;		/* 0x7BE, rev2 */
	u32 PAD[464];

	u16 PAD[0x80];
};

#ifdef BCMDBG
/* Device console log buffer state */
struct brcmf_console {
	uint count;		/* Poll interval msec counter */
	uint log_addr;		/* Log struct address (fixed) */
	struct rte_log log;	/* Log struct (host copy) */
	uint bufsize;		/* Size of log buffer */
	u8 *buf;		/* Log buffer (host copy) */
	uint last;		/* Last buffer read index */
};
#endif				/* BCMDBG */

struct sdpcm_shared {
	u32 flags;
	u32 trap_addr;
	u32 assert_exp_addr;
	u32 assert_file_addr;
	u32 assert_line;
	u32 console_addr;	/* Address of struct rte_console */
	u32 msgtrace_addr;
	u8 tag[32];
};

>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

/* misc chip info needed by some of the routines */
struct chip_info {
	u32 chip;
	u32 chiprev;
	u32 cccorebase;
	u32 ccrev;
	u32 cccaps;
<<<<<<< HEAD
	u32 buscorebase;
=======
	u32 buscorebase; /* 32 bits backplane bus address */
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	u32 buscorerev;
	u32 buscoretype;
	u32 ramcorebase;
	u32 armcorebase;
	u32 pmurev;
	u32 ramsize;
};

/* Private data for SDIO bus interaction */
<<<<<<< HEAD
typedef struct dhd_bus {
	dhd_pub_t *dhd;

	bcmsdh_info_t *sdh;	/* Handle for BCMSDH calls */
	struct chip_info *ci;	/* Chip info struct */
	char *vars;		/* Variables (from CIS and/or other) */
	uint varsz;		/* Size of variables buffer */
	u32 sbaddr;		/* Current SB window pointer (-1, invalid) */

	sdpcmd_regs_t *regs;	/* Registers for SDIO core */
	uint sdpcmrev;		/* SDIO core revision */
	uint armrev;		/* CPU core revision */
	uint ramrev;		/* SOCRAM core revision */
=======
struct brcmf_bus {
	struct brcmf_pub *drvr;

	struct brcmf_sdio_card *card;	/* Handle for sdio card calls */
	struct chip_info *ci;	/* Chip info struct */
	char *vars;		/* Variables (from CIS and/or other) */
	uint varsz;		/* Size of variables buffer */

>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	u32 ramsize;		/* Size of RAM in SOCRAM (bytes) */
	u32 orig_ramsize;	/* Size of RAM in SOCRAM (bytes) */

	u32 bus;		/* gSPI or SDIO bus */
	u32 hostintmask;	/* Copy of Host Interrupt Mask */
	u32 intstatus;	/* Intstatus bits (events) pending */
	bool dpc_sched;		/* Indicates DPC schedule (intrpt rcvd) */
	bool fcstate;		/* State of dongle flow-control */

<<<<<<< HEAD
	u16 cl_devid;	/* cached devid for dhdsdio_probe_attach() */
	char *fw_path;		/* module_param: path to firmware image */
	char *nv_path;		/* module_param: path to nvram vars file */
	const char *nvram_params;	/* user specified nvram params. */
=======
	u16 cl_devid;	/* cached devid for brcmf_sdio_probe_attach() */
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	uint blocksize;		/* Block size of SDIO transfers */
	uint roundup;		/* Max roundup limit */

	struct pktq txq;	/* Queue length used for flow-control */
	u8 flowcontrol;	/* per prio flow control bitmask */
	u8 tx_seq;		/* Transmit sequence number (next) */
	u8 tx_max;		/* Maximum transmit sequence allowed */

<<<<<<< HEAD
	u8 hdrbuf[MAX_HDR_READ + DHD_SDALIGN];
=======
	u8 hdrbuf[MAX_HDR_READ + BRCMF_SDALIGN];
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	u8 *rxhdr;		/* Header of current rx frame (in hdrbuf) */
	u16 nextlen;		/* Next Read Len from last header */
	u8 rx_seq;		/* Receive sequence number (expected) */
	bool rxskip;		/* Skip receive (awaiting NAK ACK) */

	struct sk_buff *glomd;	/* Packet containing glomming descriptor */
	struct sk_buff *glom;	/* Packet chain for glommed superframe */
	uint glomerr;		/* Glom packet read errors */

	u8 *rxbuf;		/* Buffer for receiving control packets */
	uint rxblen;		/* Allocated length of rxbuf */
	u8 *rxctl;		/* Aligned pointer into rxbuf */
	u8 *databuf;		/* Buffer for receiving big glom packet */
	u8 *dataptr;		/* Aligned pointer into databuf */
	uint rxlen;		/* Length of valid data in buffer */

	u8 sdpcm_ver;	/* Bus protocol reported by dongle */

	bool intr;		/* Use interrupts */
	bool poll;		/* Use polling */
	bool ipend;		/* Device interrupt is pending */
	bool intdis;		/* Interrupts disabled by isr */
	uint intrcount;		/* Count of device interrupt callbacks */
	uint lastintrs;		/* Count as of last watchdog timer */
	uint spurious;		/* Count of spurious interrupts */
	uint pollrate;		/* Ticks between device polls */
	uint polltick;		/* Tick counter */
	uint pollcnt;		/* Count of active polls */

<<<<<<< HEAD
#ifdef DHD_DEBUG
	dhd_console_t console;	/* Console output polling support */
	uint console_addr;	/* Console address from shared struct */
#endif				/* DHD_DEBUG */

	uint regfails;		/* Count of R_REG/W_REG failures */
=======
#ifdef BCMDBG
	struct brcmf_console console;	/* Console output polling support */
	uint console_addr;	/* Console address from shared struct */
#endif				/* BCMDBG */

	uint regfails;		/* Count of R_REG failures */
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	uint clkstate;		/* State of sd and backplane clock(s) */
	bool activity;		/* Activity flag for clock down */
	s32 idletime;		/* Control for activity timeout */
	s32 idlecount;	/* Activity timeout counter */
	s32 idleclock;	/* How to set bus driver when idle */
<<<<<<< HEAD
	s32 sd_divisor;	/* Speed control to bus driver */
	s32 sd_mode;		/* Mode control to bus driver */
	s32 sd_rxchain;	/* If bcmsdh api accepts PKT chains */
	bool use_rxchain;	/* If dhd should use PKT chains */
	bool sleeping;		/* Is SDIO bus sleeping? */
	bool rxflow_mode;	/* Rx flow control mode */
	bool rxflow;		/* Is rx flow control on */
	uint prev_rxlim_hit;	/* Is prev rx limit exceeded
					 (per dpc schedule) */
=======
	s32 sd_rxchain;
	bool use_rxchain;	/* If brcmf should use PKT chains */
	bool sleeping;		/* Is SDIO bus sleeping? */
	bool rxflow_mode;	/* Rx flow control mode */
	bool rxflow;		/* Is rx flow control on */
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	bool alp_only;		/* Don't use HT clock (ALP only) */
/* Field to decide if rx of control frames happen in rxbuf or lb-pool */
	bool usebufpool;

#ifdef SDTEST
	/* external loopback */
	bool ext_loop;
	u8 loopid;

	/* pktgen configuration */
	uint pktgen_freq;	/* Ticks between bursts */
	uint pktgen_count;	/* Packets to send each burst */
	uint pktgen_print;	/* Bursts between count displays */
	uint pktgen_total;	/* Stop after this many */
	uint pktgen_minlen;	/* Minimum packet data len */
	uint pktgen_maxlen;	/* Maximum packet data len */
	uint pktgen_mode;	/* Configured mode: tx, rx, or echo */
	uint pktgen_stop;	/* Number of tx failures causing stop */

	/* active pktgen fields */
	uint pktgen_tick;	/* Tick counter for bursts */
	uint pktgen_ptick;	/* Burst counter for printing */
	uint pktgen_sent;	/* Number of test packets generated */
	uint pktgen_rcvd;	/* Number of test packets received */
	uint pktgen_fail;	/* Number of failed send attempts */
	u16 pktgen_len;	/* Length of next packet to send */
#endif				/* SDTEST */

	/* Some additional counters */
	uint tx_sderrs;		/* Count of tx attempts with sd errors */
	uint fcqueued;		/* Tx packets that got queued */
	uint rxrtx;		/* Count of rtx requests (NAK to dongle) */
	uint rx_toolong;	/* Receive frames too long to receive */
	uint rxc_errors;	/* SDIO errors when reading control frames */
	uint rx_hdrfail;	/* SDIO errors on header reads */
	uint rx_badhdr;		/* Bad received headers (roosync?) */
	uint rx_badseq;		/* Mismatched rx sequence number */
	uint fc_rcvd;		/* Number of flow-control events received */
	uint fc_xoff;		/* Number which turned on flow-control */
	uint fc_xon;		/* Number which turned off flow-control */
	uint rxglomfail;	/* Failed deglom attempts */
	uint rxglomframes;	/* Number of glom frames (superframes) */
	uint rxglompkts;	/* Number of packets from glom frames */
	uint f2rxhdrs;		/* Number of header reads */
	uint f2rxdata;		/* Number of frame data reads */
	uint f2txdata;		/* Number of f2 frame writes */
	uint f1regdata;		/* Number of f1 register accesses */

	u8 *ctrl_frame_buf;
	u32 ctrl_frame_len;
	bool ctrl_frame_stat;
<<<<<<< HEAD
} dhd_bus_t;
=======

	spinlock_t txqlock;
	wait_queue_head_t ctrl_wait;

	struct timer_list timer;
	struct completion watchdog_wait;
	struct task_struct *watchdog_tsk;
	bool wd_timer_valid;

	struct tasklet_struct tasklet;
	struct task_struct *dpc_tsk;
	struct completion dpc_wait;

	bool threads_only;
	struct semaphore sdsem;
	spinlock_t sdlock;

	const char *fw_name;
	const struct firmware *firmware;
	const char *nv_name;
	u32 fw_ptr;
};

struct sbconfig {
	u32 PAD[2];
	u32 sbipsflag;	/* initiator port ocp slave flag */
	u32 PAD[3];
	u32 sbtpsflag;	/* target port ocp slave flag */
	u32 PAD[11];
	u32 sbtmerrloga;	/* (sonics >= 2.3) */
	u32 PAD;
	u32 sbtmerrlog;	/* (sonics >= 2.3) */
	u32 PAD[3];
	u32 sbadmatch3;	/* address match3 */
	u32 PAD;
	u32 sbadmatch2;	/* address match2 */
	u32 PAD;
	u32 sbadmatch1;	/* address match1 */
	u32 PAD[7];
	u32 sbimstate;	/* initiator agent state */
	u32 sbintvec;	/* interrupt mask */
	u32 sbtmstatelow;	/* target state */
	u32 sbtmstatehigh;	/* target state */
	u32 sbbwa0;		/* bandwidth allocation table0 */
	u32 PAD;
	u32 sbimconfiglow;	/* initiator configuration */
	u32 sbimconfighigh;	/* initiator configuration */
	u32 sbadmatch0;	/* address match0 */
	u32 PAD;
	u32 sbtmconfiglow;	/* target configuration */
	u32 sbtmconfighigh;	/* target configuration */
	u32 sbbconfig;	/* broadcast configuration */
	u32 PAD;
	u32 sbbstate;	/* broadcast state */
	u32 PAD[3];
	u32 sbactcnfg;	/* activate configuration */
	u32 PAD[3];
	u32 sbflagst;	/* current sbflags */
	u32 PAD[3];
	u32 sbidlow;		/* identification */
	u32 sbidhigh;	/* identification */
};
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

/* clkstate */
#define CLK_NONE	0
#define CLK_SDONLY	1
#define CLK_PENDING	2	/* Not used yet */
#define CLK_AVAIL	3

<<<<<<< HEAD
#define DHD_NOPMU(dhd)	(false)

#ifdef DHD_DEBUG
static int qcount[NUMPRIO];
static int tx_packets[NUMPRIO];
#endif				/* DHD_DEBUG */

/* Deferred transmit */
const uint dhd_deferred_tx = 1;

extern uint dhd_watchdog_ms;
extern void dhd_os_wd_timer(void *bus, uint wdtick);

/* Tx/Rx bounds */
uint dhd_txbound;
uint dhd_rxbound;
uint dhd_txminmax;

/* override the RAM size if possible */
#define DONGLE_MIN_MEMSIZE (128 * 1024)
int dhd_dongle_memsize;

static bool dhd_alignctl;
=======
#define BRCMF_NOPMU(brcmf)	(false)

#ifdef BCMDBG
static int qcount[NUMPRIO];
static int tx_packets[NUMPRIO];
#endif				/* BCMDBG */

/* Deferred transmit */
uint brcmf_deferred_tx = 1;
module_param(brcmf_deferred_tx, uint, 0);

/* Watchdog thread priority, -1 to use kernel timer */
int brcmf_watchdog_prio = 97;
module_param(brcmf_watchdog_prio, int, 0);

/* Watchdog interval */
uint brcmf_watchdog_ms = 10;
module_param(brcmf_watchdog_ms, uint, 0);

/* DPC thread priority, -1 to use tasklet */
int brcmf_dpc_prio = 98;
module_param(brcmf_dpc_prio, int, 0);

#ifdef BCMDBG
/* Console poll interval */
uint brcmf_console_ms;
module_param(brcmf_console_ms, uint, 0);
#endif		/* BCMDBG */

/* Tx/Rx bounds */
uint brcmf_txbound;
uint brcmf_rxbound;
uint brcmf_txminmax;

/* override the RAM size if possible */
#define DONGLE_MIN_MEMSIZE (128 * 1024)
int brcmf_dongle_memsize;

static bool brcmf_alignctl;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

static bool sd1idle;

static bool retrydata;
#define RETRYCHAN(chan) (((chan) == SDPCM_EVENT_CHANNEL) || retrydata)

static const uint watermark = 8;
<<<<<<< HEAD
static const uint firstread = DHD_FIRSTREAD;

#define HDATLEN (firstread - (SDPCM_HDRLEN))
=======
static const uint firstread = BRCMF_FIRSTREAD;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

/* Retry count for register access failures */
static const uint retry_limit = 2;

/* Force even SD lengths (some host controllers mess up on odd bytes) */
static bool forcealign;

#define ALIGNMENT  4

<<<<<<< HEAD
#if defined(OOB_INTR_ONLY) && defined(HW_OOB)
extern void bcmsdh_enable_hw_oob_intr(void *sdh, bool enable);
#endif

#if defined(OOB_INTR_ONLY) && defined(SDIO_ISR_THREAD)
#error OOB_INTR_ONLY is NOT working with SDIO_ISR_THREAD
#endif	/* defined(OOB_INTR_ONLY) && defined(SDIO_ISR_THREAD) */
=======
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
#define PKTALIGN(_p, _len, _align)				\
	do {								\
		uint datalign;						\
		datalign = (unsigned long)((_p)->data);			\
		datalign = roundup(datalign, (_align)) - datalign;	\
<<<<<<< HEAD
		ASSERT(datalign < (_align));				\
		ASSERT((_p)->len >= ((_len) + datalign));		\
=======
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		if (datalign)						\
			skb_pull((_p), datalign);			\
		__skb_trim((_p), (_len));				\
	} while (0)

/* Limit on rounding up frames */
static const uint max_roundup = 512;

/* Try doing readahead */
<<<<<<< HEAD
static bool dhd_readahead;
=======
static bool brcmf_readahead;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

/* To check if there's window offered */
#define DATAOK(bus) \
	(((u8)(bus->tx_max - bus->tx_seq) != 0) && \
	(((u8)(bus->tx_max - bus->tx_seq) & 0x80) == 0))

<<<<<<< HEAD
/* Macros to get register read/write status */
/* NOTE: these assume a local dhdsdio_bus_t *bus! */
#define R_SDREG(regvar, regaddr, retryvar) \
do { \
	retryvar = 0; \
	do { \
		regvar = R_REG(regaddr); \
	} while (bcmsdh_regfail(bus->sdh) && (++retryvar <= retry_limit)); \
	if (retryvar) { \
		bus->regfails += (retryvar-1); \
		if (retryvar > retry_limit) { \
			DHD_ERROR(("%s: FAILED" #regvar "READ, LINE %d\n", \
			__func__, __LINE__)); \
			regvar = 0; \
		} \
	} \
} while (0)

#define W_SDREG(regval, regaddr, retryvar) \
do { \
	retryvar = 0; \
	do { \
		W_REG(regaddr, regval); \
	} while (bcmsdh_regfail(bus->sdh) && (++retryvar <= retry_limit)); \
	if (retryvar) { \
		bus->regfails += (retryvar-1); \
		if (retryvar > retry_limit) \
			DHD_ERROR(("%s: FAILED REGISTER WRITE, LINE %d\n", \
			__func__, __LINE__)); \
	} \
} while (0)

#define DHD_BUS			SDIO_BUS
=======
/*
 * Reads a register in the SDIO hardware block. This block occupies a series of
 * adresses on the 32 bit backplane bus.
 */
static void
r_sdreg32(struct brcmf_bus *bus, u32 *regvar, u32 reg_offset, u32 *retryvar)
{
	*retryvar = 0;
	do {
		*regvar = R_REG(bus->ci->buscorebase + reg_offset, u32);
	} while (brcmf_sdcard_regfail(bus->card) &&
		 (++(*retryvar) <= retry_limit));
	if (*retryvar) {
		bus->regfails += (*retryvar-1);
		if (*retryvar > retry_limit) {
			BRCMF_ERROR(("FAILED READ %Xh\n", reg_offset));
			*regvar = 0;
		}
	}
}

static void
w_sdreg32(struct brcmf_bus *bus, u32 regval, u32 reg_offset, u32 *retryvar)
{
	*retryvar = 0;
	do {
		brcmf_sdcard_reg_write(NULL, bus->ci->buscorebase + reg_offset,
				       sizeof(u32), regval);
	} while (brcmf_sdcard_regfail(bus->card) &&
		 (++(*retryvar) <= retry_limit));
	if (*retryvar) {
		bus->regfails += (*retryvar-1);
		if (*retryvar > retry_limit)
			BRCMF_ERROR(("FAILED REGISTER WRITE"
				     " %Xh\n", reg_offset));
	}
}

#define BRCMF_BUS			SDIO_BUS
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

#define PKT_AVAILABLE()		(intstatus & I_HMB_FRAME_IND)

#define HOSTINTMASK		(I_HMB_SW_MASK | I_CHIPACTIVE)

#ifdef SDTEST
<<<<<<< HEAD
static void dhdsdio_testrcv(dhd_bus_t *bus, void *pkt, uint seq);
static void dhdsdio_sdtest_set(dhd_bus_t *bus, bool start);
#endif

#ifdef DHD_DEBUG
static int dhdsdio_checkdied(dhd_bus_t *bus, u8 *data, uint size);
static int dhdsdio_mem_dump(dhd_bus_t *bus);
#endif				/* DHD_DEBUG  */
static int dhdsdio_download_state(dhd_bus_t *bus, bool enter);

static void dhdsdio_release(dhd_bus_t *bus);
static void dhdsdio_release_malloc(dhd_bus_t *bus);
static void dhdsdio_disconnect(void *ptr);
static bool dhdsdio_chipmatch(u16 chipid);
static bool dhdsdio_probe_attach(dhd_bus_t *bus, void *sdh,
				 void *regsva, u16 devid);
static bool dhdsdio_probe_malloc(dhd_bus_t *bus, void *sdh);
static bool dhdsdio_probe_init(dhd_bus_t *bus, void *sdh);
static void dhdsdio_release_dongle(dhd_bus_t *bus);

static uint process_nvram_vars(char *varbuf, uint len);

static void dhd_dongle_setmemsize(struct dhd_bus *bus, int mem_size);
static int dhd_bcmsdh_send_buf(dhd_bus_t *bus, u32 addr, uint fn,
			       uint flags, u8 *buf, uint nbytes,
			       struct sk_buff *pkt, bcmsdh_cmplt_fn_t complete,
			       void *handle);

static bool dhdsdio_download_firmware(struct dhd_bus *bus, void *sdh);
static int _dhdsdio_download_firmware(struct dhd_bus *bus);

static int dhdsdio_download_code_file(struct dhd_bus *bus, char *image_path);
static int dhdsdio_download_nvram(struct dhd_bus *bus);
#ifdef BCMEMBEDIMAGE
static int dhdsdio_download_code_array(struct dhd_bus *bus);
#endif
static void dhdsdio_chip_disablecore(bcmsdh_info_t *sdh, u32 corebase);
static int dhdsdio_chip_attach(struct dhd_bus *bus, void *regs);
static void dhdsdio_chip_resetcore(bcmsdh_info_t *sdh, u32 corebase);
static void dhdsdio_sdiod_drive_strength_init(struct dhd_bus *bus,
					u32 drivestrength);
static void dhdsdio_chip_detach(struct dhd_bus *bus);
=======
static void brcmf_sdbrcm_checkdied(struct brcmf_bus *bus, void *pkt, uint seq);
static void brcmf_sdbrcm_sdtest_set(struct brcmf_bus *bus, bool start);
#endif

#ifdef BCMDBG
static int brcmf_sdbrcm_bus_console_in(struct brcmf_pub *drvr,
				       unsigned char *msg, uint msglen);
static int brcmf_sdbrcm_checkdied(struct brcmf_bus *bus, u8 *data, uint size);
static int brcmf_sdbrcm_mem_dump(struct brcmf_bus *bus);
#endif				/* BCMDBG  */
static int brcmf_sdbrcm_download_state(struct brcmf_bus *bus, bool enter);

static void brcmf_sdbrcm_release(struct brcmf_bus *bus);
static void brcmf_sdbrcm_release_malloc(struct brcmf_bus *bus);
static void brcmf_sdbrcm_disconnect(void *ptr);
static bool brcmf_sdbrcm_chipmatch(u16 chipid);
static bool brcmf_sdbrcm_probe_attach(struct brcmf_bus *bus, void *card,
				      u32 regsva, u16 devid);
static bool brcmf_sdbrcm_probe_malloc(struct brcmf_bus *bus, void *card);
static bool brcmf_sdbrcm_probe_init(struct brcmf_bus *bus, void *card);
static void brcmf_sdbrcm_release_dongle(struct brcmf_bus *bus);

static uint brcmf_process_nvram_vars(char *varbuf, uint len);

static void brcmf_sdbrcm_setmemsize(struct brcmf_bus *bus, int mem_size);
static int brcmf_sdbrcm_send_buf(struct brcmf_bus *bus, u32 addr, uint fn,
			       uint flags, u8 *buf, uint nbytes,
			       struct sk_buff *pkt,
			       void (*complete)(void *handle, int status,
						      bool sync_waiting),
			       void *handle);

static bool brcmf_sdbrcm_download_firmware(struct brcmf_bus *bus, void *card);
static int  _brcmf_sdbrcm_download_firmware(struct brcmf_bus *bus);

static int brcmf_sdbrcm_download_code_file(struct brcmf_bus *bus);
static int brcmf_sdbrcm_download_nvram(struct brcmf_bus *bus);

static void
brcmf_sdbrcm_chip_disablecore(struct brcmf_sdio_card *card, u32 corebase);

static int brcmf_sdbrcm_chip_attach(struct brcmf_bus *bus, u32 regs);

static void
brcmf_sdbrcm_chip_resetcore(struct brcmf_sdio_card *card, u32 corebase);

static void brcmf_sdbrcm_sdiod_drive_strength_init(struct brcmf_bus *bus,
					u32 drivestrength);
static void brcmf_sdbrcm_chip_detach(struct brcmf_bus *bus);
static void brcmf_sdbrcm_wait_for_event(struct brcmf_bus *bus, bool *lockvar);
static void brcmf_sdbrcm_wait_event_wakeup(struct brcmf_bus *bus);
static void brcmf_sdbrcm_watchdog(unsigned long data);
static int brcmf_sdbrcm_watchdog_thread(void *data);
static int brcmf_sdbrcm_dpc_thread(void *data);
static void brcmf_sdbrcm_dpc_tasklet(unsigned long data);
static void brcmf_sdbrcm_sched_dpc(struct brcmf_bus *bus);
static void brcmf_sdbrcm_sdlock(struct brcmf_bus *bus);
static void brcmf_sdbrcm_sdunlock(struct brcmf_bus *bus);
static int brcmf_sdbrcm_get_image(char *buf, int len, struct brcmf_bus *bus);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

/* Packet free applicable unconditionally for sdio and sdspi.
 * Conditional if bufpool was present for gspi bus.
 */
<<<<<<< HEAD
static void dhdsdio_pktfree2(dhd_bus_t *bus, struct sk_buff *pkt)
{
	dhd_os_sdlock_rxq(bus->dhd);
	if ((bus->bus != SPI_BUS) || bus->usebufpool)
		bcm_pkt_buf_free_skb(pkt);
	dhd_os_sdunlock_rxq(bus->dhd);
}

static void dhd_dongle_setmemsize(struct dhd_bus *bus, int mem_size)
{
	s32 min_size = DONGLE_MIN_MEMSIZE;
	/* Restrict the memsize to user specified limit */
	DHD_ERROR(("user: Restrict the dongle ram size to %d, min %d\n",
		dhd_dongle_memsize, min_size));
	if ((dhd_dongle_memsize > min_size) &&
	    (dhd_dongle_memsize < (s32) bus->orig_ramsize))
		bus->ramsize = dhd_dongle_memsize;
}

static int dhdsdio_set_siaddr_window(dhd_bus_t *bus, u32 address)
{
	int err = 0;
	bcmsdh_cfg_write(bus->sdh, SDIO_FUNC_1, SBSDIO_FUNC1_SBADDRLOW,
			 (address >> 8) & SBSDIO_SBADDRLOW_MASK, &err);
	if (!err)
		bcmsdh_cfg_write(bus->sdh, SDIO_FUNC_1, SBSDIO_FUNC1_SBADDRMID,
				 (address >> 16) & SBSDIO_SBADDRMID_MASK, &err);
	if (!err)
		bcmsdh_cfg_write(bus->sdh, SDIO_FUNC_1, SBSDIO_FUNC1_SBADDRHIGH,
				 (address >> 24) & SBSDIO_SBADDRHIGH_MASK,
				 &err);
=======
static void brcmf_sdbrcm_pktfree2(struct brcmf_bus *bus, struct sk_buff *pkt)
{
	if ((bus->bus != SPI_BUS) || bus->usebufpool)
		brcmu_pkt_buf_free_skb(pkt);
}

static void brcmf_sdbrcm_setmemsize(struct brcmf_bus *bus, int mem_size)
{
	s32 min_size = DONGLE_MIN_MEMSIZE;
	/* Restrict the memsize to user specified limit */
	BRCMF_ERROR(("user: Restrict the dongle ram size to %d, min %d\n",
		     brcmf_dongle_memsize, min_size));
	if ((brcmf_dongle_memsize > min_size) &&
	    (brcmf_dongle_memsize < (s32) bus->orig_ramsize))
		bus->ramsize = brcmf_dongle_memsize;
}

static int brcmf_sdbrcm_set_siaddr_window(struct brcmf_bus *bus, u32 address)
{
	int err = 0;
	brcmf_sdcard_cfg_write(bus->card, SDIO_FUNC_1, SBSDIO_FUNC1_SBADDRLOW,
			 (address >> 8) & SBSDIO_SBADDRLOW_MASK, &err);
	if (!err)
		brcmf_sdcard_cfg_write(bus->card, SDIO_FUNC_1,
				 SBSDIO_FUNC1_SBADDRMID,
				 (address >> 16) & SBSDIO_SBADDRMID_MASK, &err);
	if (!err)
		brcmf_sdcard_cfg_write(bus->card, SDIO_FUNC_1,
				       SBSDIO_FUNC1_SBADDRHIGH,
				       (address >> 24) & SBSDIO_SBADDRHIGH_MASK,
				       &err);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	return err;
}

/* Turn backplane clock on or off */
<<<<<<< HEAD
static int dhdsdio_htclk(dhd_bus_t *bus, bool on, bool pendok)
{
	int err;
	u8 clkctl, clkreq, devctl;
	bcmsdh_info_t *sdh;

	DHD_TRACE(("%s: Enter\n", __func__));

#if defined(OOB_INTR_ONLY)
	pendok = false;
#endif
	clkctl = 0;
	sdh = bus->sdh;
=======
static int brcmf_sdbrcm_htclk(struct brcmf_bus *bus, bool on, bool pendok)
{
	int err;
	u8 clkctl, clkreq, devctl;
	struct brcmf_sdio_card *card;

	BRCMF_TRACE(("%s: Enter\n", __func__));

	clkctl = 0;
	card = bus->card;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	if (on) {
		/* Request HT Avail */
		clkreq =
		    bus->alp_only ? SBSDIO_ALP_AVAIL_REQ : SBSDIO_HT_AVAIL_REQ;

		if ((bus->ci->chip == BCM4329_CHIP_ID)
		    && (bus->ci->chiprev == 0))
			clkreq |= SBSDIO_FORCE_ALP;

<<<<<<< HEAD
		bcmsdh_cfg_write(sdh, SDIO_FUNC_1, SBSDIO_FUNC1_CHIPCLKCSR,
				 clkreq, &err);
		if (err) {
			DHD_ERROR(("%s: HT Avail request error: %d\n",
				   __func__, err));
=======
		brcmf_sdcard_cfg_write(card, SDIO_FUNC_1,
				       SBSDIO_FUNC1_CHIPCLKCSR, clkreq, &err);
		if (err) {
			BRCMF_ERROR(("%s: HT Avail request error: %d\n",
				     __func__, err));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			return -EBADE;
		}

		if (pendok && ((bus->ci->buscoretype == PCMCIA_CORE_ID)
			       && (bus->ci->buscorerev == 9))) {
			u32 dummy, retries;
<<<<<<< HEAD
			R_SDREG(dummy, &bus->regs->clockctlstatus, retries);
		}

		/* Check current status */
		clkctl =
		    bcmsdh_cfg_read(sdh, SDIO_FUNC_1, SBSDIO_FUNC1_CHIPCLKCSR,
				    &err);
		if (err) {
			DHD_ERROR(("%s: HT Avail read error: %d\n",
				   __func__, err));
=======
			r_sdreg32(bus, &dummy,
				  offsetof(struct sdpcmd_regs, clockctlstatus),
				  &retries);
		}

		/* Check current status */
		clkctl = brcmf_sdcard_cfg_read(card, SDIO_FUNC_1,
					       SBSDIO_FUNC1_CHIPCLKCSR, &err);
		if (err) {
			BRCMF_ERROR(("%s: HT Avail read error: %d\n",
				     __func__, err));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			return -EBADE;
		}

		/* Go to pending and await interrupt if appropriate */
		if (!SBSDIO_CLKAV(clkctl, bus->alp_only) && pendok) {
			/* Allow only clock-available interrupt */
<<<<<<< HEAD
			devctl =
			    bcmsdh_cfg_read(sdh, SDIO_FUNC_1, SBSDIO_DEVICE_CTL,
					    &err);
			if (err) {
				DHD_ERROR(("%s: Devctl error setting CA: %d\n",
					__func__, err));
=======
			devctl = brcmf_sdcard_cfg_read(card, SDIO_FUNC_1,
					SBSDIO_DEVICE_CTL, &err);
			if (err) {
				BRCMF_ERROR(("%s: Devctl error setting CA:"
					     " %d\n", __func__, err));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				return -EBADE;
			}

			devctl |= SBSDIO_DEVCTL_CA_INT_ONLY;
<<<<<<< HEAD
			bcmsdh_cfg_write(sdh, SDIO_FUNC_1, SBSDIO_DEVICE_CTL,
					 devctl, &err);
			DHD_INFO(("CLKCTL: set PENDING\n"));
=======
			brcmf_sdcard_cfg_write(card, SDIO_FUNC_1,
					       SBSDIO_DEVICE_CTL, devctl, &err);
			BRCMF_INFO(("CLKCTL: set PENDING\n"));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			bus->clkstate = CLK_PENDING;

			return 0;
		} else if (bus->clkstate == CLK_PENDING) {
			/* Cancel CA-only interrupt filter */
			devctl =
<<<<<<< HEAD
			    bcmsdh_cfg_read(sdh, SDIO_FUNC_1, SBSDIO_DEVICE_CTL,
					    &err);
			devctl &= ~SBSDIO_DEVCTL_CA_INT_ONLY;
			bcmsdh_cfg_write(sdh, SDIO_FUNC_1, SBSDIO_DEVICE_CTL,
					 devctl, &err);
=======
			    brcmf_sdcard_cfg_read(card, SDIO_FUNC_1,
						  SBSDIO_DEVICE_CTL, &err);
			devctl &= ~SBSDIO_DEVCTL_CA_INT_ONLY;
			brcmf_sdcard_cfg_write(card, SDIO_FUNC_1,
				SBSDIO_DEVICE_CTL, devctl, &err);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		}

		/* Otherwise, wait here (polling) for HT Avail */
		if (!SBSDIO_CLKAV(clkctl, bus->alp_only)) {
<<<<<<< HEAD
			SPINWAIT_SLEEP(sdioh_spinwait_sleep,
				       ((clkctl =
					 bcmsdh_cfg_read(sdh, SDIO_FUNC_1,
						 SBSDIO_FUNC1_CHIPCLKCSR,
							 &err)),
					!SBSDIO_CLKAV(clkctl, bus->alp_only)),
				       PMU_MAX_TRANSITION_DLY);
		}
		if (err) {
			DHD_ERROR(("%s: HT Avail request error: %d\n",
				   __func__, err));
			return -EBADE;
		}
		if (!SBSDIO_CLKAV(clkctl, bus->alp_only)) {
			DHD_ERROR(("%s: HT Avail timeout (%d): clkctl 0x%02x\n",
				   __func__, PMU_MAX_TRANSITION_DLY, clkctl));
=======
			BRCMF_SPINWAIT_SLEEP(sdioh_spinwait_sleep,
			       ((clkctl =
				 brcmf_sdcard_cfg_read(card, SDIO_FUNC_1,
					 SBSDIO_FUNC1_CHIPCLKCSR,
						 &err)),
				!SBSDIO_CLKAV(clkctl, bus->alp_only)),
			       PMU_MAX_TRANSITION_DLY);
		}
		if (err) {
			BRCMF_ERROR(("%s: HT Avail request error: %d\n",
				     __func__, err));
			return -EBADE;
		}
		if (!SBSDIO_CLKAV(clkctl, bus->alp_only)) {
			BRCMF_ERROR(("%s: HT Avail timeout (%d): "
				     "clkctl 0x%02x\n", __func__,
				     PMU_MAX_TRANSITION_DLY, clkctl));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			return -EBADE;
		}

		/* Mark clock available */
		bus->clkstate = CLK_AVAIL;
<<<<<<< HEAD
		DHD_INFO(("CLKCTL: turned ON\n"));

#if defined(DHD_DEBUG)
		if (bus->alp_only == true) {
#if !defined(BCMLXSDMMC)
			if (!SBSDIO_ALPONLY(clkctl)) {
				DHD_ERROR(("%s: HT Clock, when ALP Only\n",
					   __func__));
			}
#endif				/* !defined(BCMLXSDMMC) */
		} else {
			if (SBSDIO_ALPONLY(clkctl)) {
				DHD_ERROR(("%s: HT Clock should be on.\n",
					   __func__));
			}
		}
#endif				/* defined (DHD_DEBUG) */
=======
		BRCMF_INFO(("CLKCTL: turned ON\n"));

#if defined(BCMDBG)
		if (bus->alp_only != true) {
			if (SBSDIO_ALPONLY(clkctl)) {
				BRCMF_ERROR(("%s: HT Clock should be on.\n",
					     __func__));
			}
		}
#endif				/* defined (BCMDBG) */
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

		bus->activity = true;
	} else {
		clkreq = 0;

		if (bus->clkstate == CLK_PENDING) {
			/* Cancel CA-only interrupt filter */
<<<<<<< HEAD
			devctl =
			    bcmsdh_cfg_read(sdh, SDIO_FUNC_1, SBSDIO_DEVICE_CTL,
					    &err);
			devctl &= ~SBSDIO_DEVCTL_CA_INT_ONLY;
			bcmsdh_cfg_write(sdh, SDIO_FUNC_1, SBSDIO_DEVICE_CTL,
					 devctl, &err);
		}

		bus->clkstate = CLK_SDONLY;
		bcmsdh_cfg_write(sdh, SDIO_FUNC_1, SBSDIO_FUNC1_CHIPCLKCSR,
				 clkreq, &err);
		DHD_INFO(("CLKCTL: turned OFF\n"));
		if (err) {
			DHD_ERROR(("%s: Failed access turning clock off: %d\n",
				   __func__, err));
=======
			devctl = brcmf_sdcard_cfg_read(card, SDIO_FUNC_1,
					SBSDIO_DEVICE_CTL, &err);
			devctl &= ~SBSDIO_DEVCTL_CA_INT_ONLY;
			brcmf_sdcard_cfg_write(card, SDIO_FUNC_1,
				SBSDIO_DEVICE_CTL, devctl, &err);
		}

		bus->clkstate = CLK_SDONLY;
		brcmf_sdcard_cfg_write(card, SDIO_FUNC_1,
			SBSDIO_FUNC1_CHIPCLKCSR, clkreq, &err);
		BRCMF_INFO(("CLKCTL: turned OFF\n"));
		if (err) {
			BRCMF_ERROR(("%s: Failed access turning clock off:"
				     " %d\n", __func__, err));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			return -EBADE;
		}
	}
	return 0;
}

/* Change idle/active SD state */
<<<<<<< HEAD
static int dhdsdio_sdclk(dhd_bus_t *bus, bool on)
{
	int err;
	s32 iovalue;

	DHD_TRACE(("%s: Enter\n", __func__));

	if (on) {
		if (bus->idleclock == DHD_IDLE_STOP) {
			/* Turn on clock and restore mode */
			iovalue = 1;
			err = bcmsdh_iovar_op(bus->sdh, "sd_clock", NULL, 0,
					      &iovalue, sizeof(iovalue), true);
			if (err) {
				DHD_ERROR(("%s: error enabling sd_clock: %d\n",
					   __func__, err));
				return -EBADE;
			}

			iovalue = bus->sd_mode;
			err = bcmsdh_iovar_op(bus->sdh, "sd_mode", NULL, 0,
					      &iovalue, sizeof(iovalue), true);
			if (err) {
				DHD_ERROR(("%s: error changing sd_mode: %d\n",
					   __func__, err));
				return -EBADE;
			}
		} else if (bus->idleclock != DHD_IDLE_ACTIVE) {
			/* Restore clock speed */
			iovalue = bus->sd_divisor;
			err = bcmsdh_iovar_op(bus->sdh, "sd_divisor", NULL, 0,
					      &iovalue, sizeof(iovalue), true);
			if (err) {
				DHD_ERROR(("%s: error restoring sd_divisor: %d\n",
					__func__, err));
				return -EBADE;
			}
		}
		bus->clkstate = CLK_SDONLY;
	} else {
		/* Stop or slow the SD clock itself */
		if ((bus->sd_divisor == -1) || (bus->sd_mode == -1)) {
			DHD_TRACE(("%s: can't idle clock, divisor %d mode %d\n",
				   __func__, bus->sd_divisor, bus->sd_mode));
			return -EBADE;
		}
		if (bus->idleclock == DHD_IDLE_STOP) {
			if (sd1idle) {
				/* Change to SD1 mode and turn off clock */
				iovalue = 1;
				err =
				    bcmsdh_iovar_op(bus->sdh, "sd_mode", NULL,
						    0, &iovalue,
						    sizeof(iovalue), true);
				if (err) {
					DHD_ERROR(("%s: error changing sd_clock: %d\n",
						__func__, err));
					return -EBADE;
				}
			}

			iovalue = 0;
			err = bcmsdh_iovar_op(bus->sdh, "sd_clock", NULL, 0,
					      &iovalue, sizeof(iovalue), true);
			if (err) {
				DHD_ERROR(("%s: error disabling sd_clock: %d\n",
					   __func__, err));
				return -EBADE;
			}
		} else if (bus->idleclock != DHD_IDLE_ACTIVE) {
			/* Set divisor to idle value */
			iovalue = bus->idleclock;
			err = bcmsdh_iovar_op(bus->sdh, "sd_divisor", NULL, 0,
					      &iovalue, sizeof(iovalue), true);
			if (err) {
				DHD_ERROR(("%s: error changing sd_divisor: %d\n",
					__func__, err));
				return -EBADE;
			}
		}
		bus->clkstate = CLK_NONE;
	}
=======
static int brcmf_sdbrcm_sdclk(struct brcmf_bus *bus, bool on)
{
	BRCMF_TRACE(("%s: Enter\n", __func__));

	if (on)
		bus->clkstate = CLK_SDONLY;
	else
		bus->clkstate = CLK_NONE;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	return 0;
}

/* Transition SD and backplane clock readiness */
<<<<<<< HEAD
static int dhdsdio_clkctl(dhd_bus_t *bus, uint target, bool pendok)
{
#ifdef DHD_DEBUG
	uint oldstate = bus->clkstate;
#endif				/* DHD_DEBUG */

	DHD_TRACE(("%s: Enter\n", __func__));
=======
static int brcmf_sdbrcm_clkctl(struct brcmf_bus *bus, uint target, bool pendok)
{
#ifdef BCMDBG
	uint oldstate = bus->clkstate;
#endif				/* BCMDBG */

	BRCMF_TRACE(("%s: Enter\n", __func__));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	/* Early exit if we're already there */
	if (bus->clkstate == target) {
		if (target == CLK_AVAIL) {
<<<<<<< HEAD
			dhd_os_wd_timer(bus->dhd, dhd_watchdog_ms);
=======
			brcmf_sdbrcm_wd_timer(bus, brcmf_watchdog_ms);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			bus->activity = true;
		}
		return 0;
	}

	switch (target) {
	case CLK_AVAIL:
		/* Make sure SD clock is available */
		if (bus->clkstate == CLK_NONE)
<<<<<<< HEAD
			dhdsdio_sdclk(bus, true);
		/* Now request HT Avail on the backplane */
		dhdsdio_htclk(bus, true, pendok);
		dhd_os_wd_timer(bus->dhd, dhd_watchdog_ms);
=======
			brcmf_sdbrcm_sdclk(bus, true);
		/* Now request HT Avail on the backplane */
		brcmf_sdbrcm_htclk(bus, true, pendok);
		brcmf_sdbrcm_wd_timer(bus, brcmf_watchdog_ms);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		bus->activity = true;
		break;

	case CLK_SDONLY:
		/* Remove HT request, or bring up SD clock */
		if (bus->clkstate == CLK_NONE)
<<<<<<< HEAD
			dhdsdio_sdclk(bus, true);
		else if (bus->clkstate == CLK_AVAIL)
			dhdsdio_htclk(bus, false, false);
		else
			DHD_ERROR(("dhdsdio_clkctl: request for %d -> %d\n",
				   bus->clkstate, target));
		dhd_os_wd_timer(bus->dhd, dhd_watchdog_ms);
=======
			brcmf_sdbrcm_sdclk(bus, true);
		else if (bus->clkstate == CLK_AVAIL)
			brcmf_sdbrcm_htclk(bus, false, false);
		else
			BRCMF_ERROR(("brcmf_sdbrcm_clkctl: request for %d -> %d"
				     "\n", bus->clkstate, target));
		brcmf_sdbrcm_wd_timer(bus, brcmf_watchdog_ms);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		break;

	case CLK_NONE:
		/* Make sure to remove HT request */
		if (bus->clkstate == CLK_AVAIL)
<<<<<<< HEAD
			dhdsdio_htclk(bus, false, false);
		/* Now remove the SD clock */
		dhdsdio_sdclk(bus, false);
		dhd_os_wd_timer(bus->dhd, 0);
		break;
	}
#ifdef DHD_DEBUG
	DHD_INFO(("dhdsdio_clkctl: %d -> %d\n", oldstate, bus->clkstate));
#endif				/* DHD_DEBUG */
=======
			brcmf_sdbrcm_htclk(bus, false, false);
		/* Now remove the SD clock */
		brcmf_sdbrcm_sdclk(bus, false);
		brcmf_sdbrcm_wd_timer(bus, 0);
		break;
	}
#ifdef BCMDBG
	BRCMF_INFO(("brcmf_sdbrcm_clkctl: %d -> %d\n",
		    oldstate, bus->clkstate));
#endif				/* BCMDBG */
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	return 0;
}

<<<<<<< HEAD
int dhdsdio_bussleep(dhd_bus_t *bus, bool sleep)
{
	bcmsdh_info_t *sdh = bus->sdh;
	sdpcmd_regs_t *regs = bus->regs;
	uint retries = 0;

	DHD_INFO(("dhdsdio_bussleep: request %s (currently %s)\n",
		  (sleep ? "SLEEP" : "WAKE"),
		  (bus->sleeping ? "SLEEP" : "WAKE")));
=======
int brcmf_sdbrcm_bussleep(struct brcmf_bus *bus, bool sleep)
{
	struct brcmf_sdio_card *card = bus->card;
	uint retries = 0;

	BRCMF_INFO(("brcmf_sdbrcm_bussleep: request %s (currently %s)\n",
		    (sleep ? "SLEEP" : "WAKE"),
		    (bus->sleeping ? "SLEEP" : "WAKE")));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	/* Done if we're already in the requested state */
	if (sleep == bus->sleeping)
		return 0;

	/* Going to sleep: set the alarm and turn off the lights... */
	if (sleep) {
		/* Don't sleep if something is pending */
		if (bus->dpc_sched || bus->rxskip || pktq_len(&bus->txq))
			return -EBUSY;

		/* Disable SDIO interrupts (no longer interested) */
<<<<<<< HEAD
		bcmsdh_intr_disable(bus->sdh);

		/* Make sure the controller has the bus up */
		dhdsdio_clkctl(bus, CLK_AVAIL, false);

		/* Tell device to start using OOB wakeup */
		W_SDREG(SMB_USE_OOB, &regs->tosbmailbox, retries);
		if (retries > retry_limit)
			DHD_ERROR(("CANNOT SIGNAL CHIP, WILL NOT WAKE UP!!\n"));

		/* Turn off our contribution to the HT clock request */
		dhdsdio_clkctl(bus, CLK_SDONLY, false);

		bcmsdh_cfg_write(sdh, SDIO_FUNC_1, SBSDIO_FUNC1_CHIPCLKCSR,
				 SBSDIO_FORCE_HW_CLKREQ_OFF, NULL);
=======
		brcmf_sdcard_intr_disable(bus->card);

		/* Make sure the controller has the bus up */
		brcmf_sdbrcm_clkctl(bus, CLK_AVAIL, false);

		/* Tell device to start using OOB wakeup */
		w_sdreg32(bus, SMB_USE_OOB,
			  offsetof(struct sdpcmd_regs, tosbmailbox), &retries);
		if (retries > retry_limit)
			BRCMF_ERROR(("CANNOT SIGNAL CHIP, "
				     "WILL NOT WAKE UP!!\n"));

		/* Turn off our contribution to the HT clock request */
		brcmf_sdbrcm_clkctl(bus, CLK_SDONLY, false);

		brcmf_sdcard_cfg_write(card, SDIO_FUNC_1,
			SBSDIO_FUNC1_CHIPCLKCSR,
			SBSDIO_FORCE_HW_CLKREQ_OFF, NULL);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

		/* Isolate the bus */
		if (bus->ci->chip != BCM4329_CHIP_ID
		    && bus->ci->chip != BCM4319_CHIP_ID) {
<<<<<<< HEAD
			bcmsdh_cfg_write(sdh, SDIO_FUNC_1, SBSDIO_DEVICE_CTL,
					 SBSDIO_DEVCTL_PADS_ISO, NULL);
=======
			brcmf_sdcard_cfg_write(card, SDIO_FUNC_1,
				SBSDIO_DEVICE_CTL,
				SBSDIO_DEVCTL_PADS_ISO, NULL);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		}

		/* Change state */
		bus->sleeping = true;

	} else {
		/* Waking up: bus power up is ok, set local state */

<<<<<<< HEAD
		bcmsdh_cfg_write(sdh, SDIO_FUNC_1, SBSDIO_FUNC1_CHIPCLKCSR,
				 0, NULL);
=======
		brcmf_sdcard_cfg_write(card, SDIO_FUNC_1,
			SBSDIO_FUNC1_CHIPCLKCSR, 0, NULL);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

		/* Force pad isolation off if possible
			 (in case power never toggled) */
		if ((bus->ci->buscoretype == PCMCIA_CORE_ID)
		    && (bus->ci->buscorerev >= 10))
<<<<<<< HEAD
			bcmsdh_cfg_write(sdh, SDIO_FUNC_1, SBSDIO_DEVICE_CTL, 0,
					 NULL);

		/* Make sure the controller has the bus up */
		dhdsdio_clkctl(bus, CLK_AVAIL, false);

		/* Send misc interrupt to indicate OOB not needed */
		W_SDREG(0, &regs->tosbmailboxdata, retries);
		if (retries <= retry_limit)
			W_SDREG(SMB_DEV_INT, &regs->tosbmailbox, retries);

		if (retries > retry_limit)
			DHD_ERROR(("CANNOT SIGNAL CHIP TO CLEAR OOB!!\n"));

		/* Make sure we have SD bus access */
		dhdsdio_clkctl(bus, CLK_SDONLY, false);
=======
			brcmf_sdcard_cfg_write(card, SDIO_FUNC_1,
				SBSDIO_DEVICE_CTL, 0, NULL);

		/* Make sure the controller has the bus up */
		brcmf_sdbrcm_clkctl(bus, CLK_AVAIL, false);

		/* Send misc interrupt to indicate OOB not needed */
		w_sdreg32(bus, 0, offsetof(struct sdpcmd_regs, tosbmailboxdata),
			  &retries);
		if (retries <= retry_limit)
			w_sdreg32(bus, SMB_DEV_INT,
				  offsetof(struct sdpcmd_regs, tosbmailbox),
				  &retries);

		if (retries > retry_limit)
			BRCMF_ERROR(("CANNOT SIGNAL CHIP TO CLEAR OOB!!\n"));

		/* Make sure we have SD bus access */
		brcmf_sdbrcm_clkctl(bus, CLK_SDONLY, false);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

		/* Change state */
		bus->sleeping = false;

		/* Enable interrupts again */
<<<<<<< HEAD
		if (bus->intr && (bus->dhd->busstate == DHD_BUS_DATA)) {
			bus->intdis = false;
			bcmsdh_intr_enable(bus->sdh);
=======
		if (bus->intr && (bus->drvr->busstate == BRCMF_BUS_DATA)) {
			bus->intdis = false;
			brcmf_sdcard_intr_enable(bus->card);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		}
	}

	return 0;
}

<<<<<<< HEAD
#if defined(OOB_INTR_ONLY)
void dhd_enable_oob_intr(struct dhd_bus *bus, bool enable)
{
#if defined(HW_OOB)
	bcmsdh_enable_hw_oob_intr(bus->sdh, enable);
#else
	sdpcmd_regs_t *regs = bus->regs;
	uint retries = 0;

	dhdsdio_clkctl(bus, CLK_AVAIL, false);
	if (enable == true) {

		/* Tell device to start using OOB wakeup */
		W_SDREG(SMB_USE_OOB, &regs->tosbmailbox, retries);
		if (retries > retry_limit)
			DHD_ERROR(("CANNOT SIGNAL CHIP, WILL NOT WAKE UP!!\n"));

	} else {
		/* Send misc interrupt to indicate OOB not needed */
		W_SDREG(0, &regs->tosbmailboxdata, retries);
		if (retries <= retry_limit)
			W_SDREG(SMB_DEV_INT, &regs->tosbmailbox, retries);
	}

	/* Turn off our contribution to the HT clock request */
	dhdsdio_clkctl(bus, CLK_SDONLY, false);
#endif				/* !defined(HW_OOB) */
}
#endif				/* defined(OOB_INTR_ONLY) */

#define BUS_WAKE(bus) \
	do { \
		if ((bus)->sleeping) \
			dhdsdio_bussleep((bus), false); \
=======
#define BUS_WAKE(bus) \
	do { \
		if ((bus)->sleeping) \
			brcmf_sdbrcm_bussleep((bus), false); \
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	} while (0);

/* Writes a HW/SW header into the packet and sends it. */
/* Assumes: (a) header space already there, (b) caller holds lock */
<<<<<<< HEAD
static int dhdsdio_txpkt(dhd_bus_t *bus, struct sk_buff *pkt, uint chan,
=======
static int brcmf_sdbrcm_txpkt(struct brcmf_bus *bus, struct sk_buff *pkt, uint chan,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			 bool free_pkt)
{
	int ret;
	u8 *frame;
	u16 len, pad = 0;
	u32 swheader;
	uint retries = 0;
<<<<<<< HEAD
	bcmsdh_info_t *sdh;
	struct sk_buff *new;
	int i;

	DHD_TRACE(("%s: Enter\n", __func__));

	sdh = bus->sdh;

	if (bus->dhd->dongle_reset) {
=======
	struct brcmf_sdio_card *card;
	struct sk_buff *new;
	int i;

	BRCMF_TRACE(("%s: Enter\n", __func__));

	card = bus->card;

	if (bus->drvr->dongle_reset) {
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		ret = -EPERM;
		goto done;
	}

	frame = (u8 *) (pkt->data);

	/* Add alignment padding, allocate new packet if needed */
<<<<<<< HEAD
	pad = ((unsigned long)frame % DHD_SDALIGN);
	if (pad) {
		if (skb_headroom(pkt) < pad) {
			DHD_INFO(("%s: insufficient headroom %d for %d pad\n",
				  __func__, skb_headroom(pkt), pad));
			bus->dhd->tx_realloc++;
			new = bcm_pkt_buf_get_skb(pkt->len + DHD_SDALIGN);
			if (!new) {
				DHD_ERROR(("%s: couldn't allocate new %d-byte "
					"packet\n",
					__func__, pkt->len + DHD_SDALIGN));
=======
	pad = ((unsigned long)frame % BRCMF_SDALIGN);
	if (pad) {
		if (skb_headroom(pkt) < pad) {
			BRCMF_INFO(("%s: insufficient headroom %d for %d pad\n",
				    __func__, skb_headroom(pkt), pad));
			bus->drvr->tx_realloc++;
			new = brcmu_pkt_buf_get_skb(pkt->len + BRCMF_SDALIGN);
			if (!new) {
				BRCMF_ERROR(("%s: couldn't allocate new "
					     "%d-byte packet\n", __func__,
					     pkt->len + BRCMF_SDALIGN));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				ret = -ENOMEM;
				goto done;
			}

<<<<<<< HEAD
			PKTALIGN(new, pkt->len, DHD_SDALIGN);
			memcpy(new->data, pkt->data, pkt->len);
			if (free_pkt)
				bcm_pkt_buf_free_skb(pkt);
=======
			PKTALIGN(new, pkt->len, BRCMF_SDALIGN);
			memcpy(new->data, pkt->data, pkt->len);
			if (free_pkt)
				brcmu_pkt_buf_free_skb(pkt);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			/* free the pkt if canned one is not used */
			free_pkt = true;
			pkt = new;
			frame = (u8 *) (pkt->data);
<<<<<<< HEAD
			ASSERT(((unsigned long)frame % DHD_SDALIGN) == 0);
=======
			/* precondition: (frame % BRCMF_SDALIGN) == 0) */
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			pad = 0;
		} else {
			skb_push(pkt, pad);
			frame = (u8 *) (pkt->data);
<<<<<<< HEAD

			ASSERT((pad + SDPCM_HDRLEN) <= (int)(pkt->len));
			memset(frame, 0, pad + SDPCM_HDRLEN);
		}
	}
	ASSERT(pad < DHD_SDALIGN);
=======
			/* precondition: pad + SDPCM_HDRLEN <= pkt->len */
			memset(frame, 0, pad + SDPCM_HDRLEN);
		}
	}
	/* precondition: pad < BRCMF_SDALIGN */
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	/* Hardware tag: 2 byte len followed by 2 byte ~len check (all LE) */
	len = (u16) (pkt->len);
	*(u16 *) frame = cpu_to_le16(len);
	*(((u16 *) frame) + 1) = cpu_to_le16(~len);

	/* Software tag: channel, sequence number, data offset */
	swheader =
	    ((chan << SDPCM_CHANNEL_SHIFT) & SDPCM_CHANNEL_MASK) | bus->tx_seq |
	    (((pad +
	       SDPCM_HDRLEN) << SDPCM_DOFFSET_SHIFT) & SDPCM_DOFFSET_MASK);

	put_unaligned_le32(swheader, frame + SDPCM_FRAMETAG_LEN);
	put_unaligned_le32(0, frame + SDPCM_FRAMETAG_LEN + sizeof(swheader));

<<<<<<< HEAD
#ifdef DHD_DEBUG
	tx_packets[pkt->priority]++;
	if (DHD_BYTES_ON() &&
	    (((DHD_CTL_ON() && (chan == SDPCM_CONTROL_CHANNEL)) ||
	      (DHD_DATA_ON() && (chan != SDPCM_CONTROL_CHANNEL))))) {
		printk(KERN_DEBUG "Tx Frame:\n");
		print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, frame, len);
	} else if (DHD_HDRS_ON()) {
=======
#ifdef BCMDBG
	tx_packets[pkt->priority]++;
	if (BRCMF_BYTES_ON() &&
	    (((BRCMF_CTL_ON() && (chan == SDPCM_CONTROL_CHANNEL)) ||
	      (BRCMF_DATA_ON() && (chan != SDPCM_CONTROL_CHANNEL))))) {
		printk(KERN_DEBUG "Tx Frame:\n");
		print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, frame, len);
	} else if (BRCMF_HDRS_ON()) {
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		printk(KERN_DEBUG "TxHdr:\n");
		print_hex_dump_bytes("", DUMP_PREFIX_OFFSET,
				     frame, min_t(u16, len, 16));
	}
#endif

	/* Raise len to next SDIO block to eliminate tail command */
	if (bus->roundup && bus->blocksize && (len > bus->blocksize)) {
		u16 pad = bus->blocksize - (len % bus->blocksize);
		if ((pad <= bus->roundup) && (pad < bus->blocksize))
<<<<<<< HEAD
#ifdef NOTUSED
			if (pad <= skb_tailroom(pkt))
#endif				/* NOTUSED */
				len += pad;
	} else if (len % DHD_SDALIGN) {
		len += DHD_SDALIGN - (len % DHD_SDALIGN);
=======
				len += pad;
	} else if (len % BRCMF_SDALIGN) {
		len += BRCMF_SDALIGN - (len % BRCMF_SDALIGN);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	}

	/* Some controllers have trouble with odd bytes -- round to even */
	if (forcealign && (len & (ALIGNMENT - 1))) {
<<<<<<< HEAD
#ifdef NOTUSED
		if (skb_tailroom(pkt))
#endif
			len = roundup(len, ALIGNMENT);
#ifdef NOTUSED
		else
			DHD_ERROR(("%s: sending unrounded %d-byte packet\n",
				   __func__, len));
#endif
	}

	do {
		ret =
		    dhd_bcmsdh_send_buf(bus, bcmsdh_cur_sbwad(sdh), SDIO_FUNC_2,
					F2SYNC, frame, len, pkt, NULL, NULL);
		bus->f2txdata++;
		ASSERT(ret != -BCME_PENDING);
=======
			len = roundup(len, ALIGNMENT);
	}

	do {
		ret = brcmf_sdbrcm_send_buf(bus, brcmf_sdcard_cur_sbwad(card),
			SDIO_FUNC_2, F2SYNC, frame, len, pkt, NULL, NULL);
		bus->f2txdata++;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

		if (ret < 0) {
			/* On failure, abort the command
			 and terminate the frame */
<<<<<<< HEAD
			DHD_INFO(("%s: sdio error %d, abort command and "
				"terminate frame.\n", __func__, ret));
			bus->tx_sderrs++;

			bcmsdh_abort(sdh, SDIO_FUNC_2);
			bcmsdh_cfg_write(sdh, SDIO_FUNC_1,
=======
			BRCMF_INFO(("%s: sdio error %d, abort command and "
				    "terminate frame.\n", __func__, ret));
			bus->tx_sderrs++;

			brcmf_sdcard_abort(card, SDIO_FUNC_2);
			brcmf_sdcard_cfg_write(card, SDIO_FUNC_1,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
					 SBSDIO_FUNC1_FRAMECTRL, SFC_WF_TERM,
					 NULL);
			bus->f1regdata++;

			for (i = 0; i < 3; i++) {
				u8 hi, lo;
<<<<<<< HEAD
				hi = bcmsdh_cfg_read(sdh, SDIO_FUNC_1,
						     SBSDIO_FUNC1_WFRAMEBCHI,
						     NULL);
				lo = bcmsdh_cfg_read(sdh, SDIO_FUNC_1,
=======
				hi = brcmf_sdcard_cfg_read(card, SDIO_FUNC_1,
						     SBSDIO_FUNC1_WFRAMEBCHI,
						     NULL);
				lo = brcmf_sdcard_cfg_read(card, SDIO_FUNC_1,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
						     SBSDIO_FUNC1_WFRAMEBCLO,
						     NULL);
				bus->f1regdata += 2;
				if ((hi == 0) && (lo == 0))
					break;
			}

		}
		if (ret == 0)
			bus->tx_seq = (bus->tx_seq + 1) % SDPCM_SEQUENCE_WRAP;

	} while ((ret < 0) && retrydata && retries++ < TXRETRIES);

done:
	/* restore pkt buffer pointer before calling tx complete routine */
	skb_pull(pkt, SDPCM_HDRLEN + pad);
<<<<<<< HEAD
	dhd_os_sdunlock(bus->dhd);
	dhd_txcomplete(bus->dhd, pkt, ret != 0);
	dhd_os_sdlock(bus->dhd);

	if (free_pkt)
		bcm_pkt_buf_free_skb(pkt);
=======
	brcmf_sdbrcm_sdunlock(bus);
	brcmf_txcomplete(bus->drvr, pkt, ret != 0);
	brcmf_sdbrcm_sdlock(bus);

	if (free_pkt)
		brcmu_pkt_buf_free_skb(pkt);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	return ret;
}

<<<<<<< HEAD
int dhd_bus_txdata(struct dhd_bus *bus, struct sk_buff *pkt)
=======
int brcmf_sdbrcm_bus_txdata(struct brcmf_bus *bus, struct sk_buff *pkt)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
{
	int ret = -EBADE;
	uint datalen, prec;

<<<<<<< HEAD
	DHD_TRACE(("%s: Enter\n", __func__));
=======
	BRCMF_TRACE(("%s: Enter\n", __func__));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	datalen = pkt->len;

#ifdef SDTEST
	/* Push the test header if doing loopback */
	if (bus->ext_loop) {
		u8 *data;
		skb_push(pkt, SDPCM_TEST_HDRLEN);
		data = pkt->data;
		*data++ = SDPCM_TEST_ECHOREQ;
		*data++ = (u8) bus->loopid++;
		*data++ = (datalen >> 0);
		*data++ = (datalen >> 8);
		datalen += SDPCM_TEST_HDRLEN;
	}
#endif				/* SDTEST */

	/* Add space for the header */
	skb_push(pkt, SDPCM_HDRLEN);
<<<<<<< HEAD
	ASSERT(IS_ALIGNED((unsigned long)(pkt->data), 2));
=======
	/* precondition: IS_ALIGNED((unsigned long)(pkt->data), 2) */
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	prec = PRIO2PREC((pkt->priority & PRIOMASK));

	/* Check for existing queue, current flow-control,
			 pending event, or pending clock */
<<<<<<< HEAD
	if (dhd_deferred_tx || bus->fcstate || pktq_len(&bus->txq)
	    || bus->dpc_sched || (!DATAOK(bus))
	    || (bus->flowcontrol & NBITVAL(prec))
	    || (bus->clkstate != CLK_AVAIL)) {
		DHD_TRACE(("%s: deferring pktq len %d\n", __func__,
			   pktq_len(&bus->txq)));
		bus->fcqueued++;

		/* Priority based enq */
		dhd_os_sdlock_txq(bus->dhd);
		if (dhd_prec_enq(bus->dhd, &bus->txq, pkt, prec) == false) {
			skb_pull(pkt, SDPCM_HDRLEN);
			dhd_txcomplete(bus->dhd, pkt, false);
			bcm_pkt_buf_free_skb(pkt);
			DHD_ERROR(("%s: out of bus->txq !!!\n", __func__));
=======
	if (brcmf_deferred_tx || bus->fcstate || pktq_len(&bus->txq)
	    || bus->dpc_sched || (!DATAOK(bus))
	    || (bus->flowcontrol & NBITVAL(prec))
	    || (bus->clkstate != CLK_AVAIL)) {
		BRCMF_TRACE(("%s: deferring pktq len %d\n", __func__,
			     pktq_len(&bus->txq)));
		bus->fcqueued++;

		/* Priority based enq */
		spin_lock_bh(&bus->txqlock);
		if (brcmf_c_prec_enq(bus->drvr, &bus->txq, pkt, prec) == false) {
			skb_pull(pkt, SDPCM_HDRLEN);
			brcmf_txcomplete(bus->drvr, pkt, false);
			brcmu_pkt_buf_free_skb(pkt);
			BRCMF_ERROR(("%s: out of bus->txq !!!\n", __func__));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			ret = -ENOSR;
		} else {
			ret = 0;
		}
<<<<<<< HEAD
		dhd_os_sdunlock_txq(bus->dhd);

		if (pktq_len(&bus->txq) >= TXHI)
			dhd_txflowcontrol(bus->dhd, 0, ON);

#ifdef DHD_DEBUG
=======
		spin_unlock_bh(&bus->txqlock);

		if (pktq_len(&bus->txq) >= TXHI)
			brcmf_txflowcontrol(bus->drvr, 0, ON);

#ifdef BCMDBG
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		if (pktq_plen(&bus->txq, prec) > qcount[prec])
			qcount[prec] = pktq_plen(&bus->txq, prec);
#endif
		/* Schedule DPC if needed to send queued packet(s) */
<<<<<<< HEAD
		if (dhd_deferred_tx && !bus->dpc_sched) {
			bus->dpc_sched = true;
			dhd_sched_dpc(bus->dhd);
		}
	} else {
		/* Lock: we're about to use shared data/code (and SDIO) */
		dhd_os_sdlock(bus->dhd);
=======
		if (brcmf_deferred_tx && !bus->dpc_sched) {
			bus->dpc_sched = true;
			brcmf_sdbrcm_sched_dpc(bus);
		}
	} else {
		/* Lock: we're about to use shared data/code (and SDIO) */
		brcmf_sdbrcm_sdlock(bus);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

		/* Otherwise, send it now */
		BUS_WAKE(bus);
		/* Make sure back plane ht clk is on, no pending allowed */
<<<<<<< HEAD
		dhdsdio_clkctl(bus, CLK_AVAIL, true);

#ifndef SDTEST
		DHD_TRACE(("%s: calling txpkt\n", __func__));
		ret = dhdsdio_txpkt(bus, pkt, SDPCM_DATA_CHANNEL, true);
#else
		ret = dhdsdio_txpkt(bus, pkt,
=======
		brcmf_sdbrcm_clkctl(bus, CLK_AVAIL, true);

#ifndef SDTEST
		BRCMF_TRACE(("%s: calling txpkt\n", __func__));
		ret = brcmf_sdbrcm_txpkt(bus, pkt, SDPCM_DATA_CHANNEL, true);
#else
		ret = brcmf_sdbrcm_txpkt(bus, pkt,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				    (bus->ext_loop ? SDPCM_TEST_CHANNEL :
				     SDPCM_DATA_CHANNEL), true);
#endif
		if (ret)
<<<<<<< HEAD
			bus->dhd->tx_errors++;
		else
			bus->dhd->dstats.tx_bytes += datalen;

		if ((bus->idletime == DHD_IDLE_IMMEDIATE) && !bus->dpc_sched) {
			bus->activity = false;
			dhdsdio_clkctl(bus, CLK_NONE, true);
		}

		dhd_os_sdunlock(bus->dhd);
=======
			bus->drvr->tx_errors++;
		else
			bus->drvr->dstats.tx_bytes += datalen;

		if (bus->idletime == BRCMF_IDLE_IMMEDIATE &&
		    !bus->dpc_sched) {
			bus->activity = false;
			brcmf_sdbrcm_clkctl(bus, CLK_NONE, true);
		}

		brcmf_sdbrcm_sdunlock(bus);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	}

	return ret;
}

<<<<<<< HEAD
static uint dhdsdio_sendfromq(dhd_bus_t *bus, uint maxframes)
=======
static uint brcmf_sdbrcm_sendfromq(struct brcmf_bus *bus, uint maxframes)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
{
	struct sk_buff *pkt;
	u32 intstatus = 0;
	uint retries = 0;
	int ret = 0, prec_out;
	uint cnt = 0;
	uint datalen;
	u8 tx_prec_map;

<<<<<<< HEAD
	dhd_pub_t *dhd = bus->dhd;
	sdpcmd_regs_t *regs = bus->regs;

	DHD_TRACE(("%s: Enter\n", __func__));
=======
	struct brcmf_pub *drvr = bus->drvr;

	BRCMF_TRACE(("%s: Enter\n", __func__));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	tx_prec_map = ~bus->flowcontrol;

	/* Send frames until the limit or some other event */
	for (cnt = 0; (cnt < maxframes) && DATAOK(bus); cnt++) {
<<<<<<< HEAD
		dhd_os_sdlock_txq(bus->dhd);
		pkt = bcm_pktq_mdeq(&bus->txq, tx_prec_map, &prec_out);
		if (pkt == NULL) {
			dhd_os_sdunlock_txq(bus->dhd);
			break;
		}
		dhd_os_sdunlock_txq(bus->dhd);
		datalen = pkt->len - SDPCM_HDRLEN;

#ifndef SDTEST
		ret = dhdsdio_txpkt(bus, pkt, SDPCM_DATA_CHANNEL, true);
#else
		ret = dhdsdio_txpkt(bus, pkt,
=======
		spin_lock_bh(&bus->txqlock);
		pkt = brcmu_pktq_mdeq(&bus->txq, tx_prec_map, &prec_out);
		if (pkt == NULL) {
			spin_unlock_bh(&bus->txqlock);
			break;
		}
		spin_unlock_bh(&bus->txqlock);
		datalen = pkt->len - SDPCM_HDRLEN;

#ifndef SDTEST
		ret = brcmf_sdbrcm_txpkt(bus, pkt, SDPCM_DATA_CHANNEL, true);
#else
		ret = brcmf_sdbrcm_txpkt(bus, pkt,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				    (bus->ext_loop ? SDPCM_TEST_CHANNEL :
				     SDPCM_DATA_CHANNEL), true);
#endif
		if (ret)
<<<<<<< HEAD
			bus->dhd->tx_errors++;
		else
			bus->dhd->dstats.tx_bytes += datalen;
=======
			bus->drvr->tx_errors++;
		else
			bus->drvr->dstats.tx_bytes += datalen;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

		/* In poll mode, need to check for other events */
		if (!bus->intr && cnt) {
			/* Check device status, signal pending interrupt */
<<<<<<< HEAD
			R_SDREG(intstatus, &regs->intstatus, retries);
			bus->f2txdata++;
			if (bcmsdh_regfail(bus->sdh))
=======
			r_sdreg32(bus, &intstatus,
				  offsetof(struct sdpcmd_regs, intstatus),
				  &retries);
			bus->f2txdata++;
			if (brcmf_sdcard_regfail(bus->card))
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				break;
			if (intstatus & bus->hostintmask)
				bus->ipend = true;
		}
	}

	/* Deflow-control stack if needed */
<<<<<<< HEAD
	if (dhd->up && (dhd->busstate == DHD_BUS_DATA) &&
	    dhd->txoff && (pktq_len(&bus->txq) < TXLOW))
		dhd_txflowcontrol(dhd, 0, OFF);
=======
	if (drvr->up && (drvr->busstate == BRCMF_BUS_DATA) &&
	    drvr->txoff && (pktq_len(&bus->txq) < TXLOW))
		brcmf_txflowcontrol(drvr, 0, OFF);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	return cnt;
}

<<<<<<< HEAD
int dhd_bus_txctl(struct dhd_bus *bus, unsigned char *msg, uint msglen)
=======
int
brcmf_sdbrcm_bus_txctl(struct brcmf_bus *bus, unsigned char *msg, uint msglen)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
{
	u8 *frame;
	u16 len;
	u32 swheader;
	uint retries = 0;
<<<<<<< HEAD
	bcmsdh_info_t *sdh = bus->sdh;
=======
	struct brcmf_sdio_card *card = bus->card;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	u8 doff = 0;
	int ret = -1;
	int i;

<<<<<<< HEAD
	DHD_TRACE(("%s: Enter\n", __func__));

	if (bus->dhd->dongle_reset)
=======
	BRCMF_TRACE(("%s: Enter\n", __func__));

	if (bus->drvr->dongle_reset)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		return -EIO;

	/* Back the pointer to make a room for bus header */
	frame = msg - SDPCM_HDRLEN;
	len = (msglen += SDPCM_HDRLEN);

	/* Add alignment padding (optional for ctl frames) */
<<<<<<< HEAD
	if (dhd_alignctl) {
		doff = ((unsigned long)frame % DHD_SDALIGN);
=======
	if (brcmf_alignctl) {
		doff = ((unsigned long)frame % BRCMF_SDALIGN);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		if (doff) {
			frame -= doff;
			len += doff;
			msglen += doff;
			memset(frame, 0, doff + SDPCM_HDRLEN);
		}
<<<<<<< HEAD
		ASSERT(doff < DHD_SDALIGN);
=======
		/* precondition: doff < BRCMF_SDALIGN */
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	}
	doff += SDPCM_HDRLEN;

	/* Round send length to next SDIO block */
	if (bus->roundup && bus->blocksize && (len > bus->blocksize)) {
		u16 pad = bus->blocksize - (len % bus->blocksize);
		if ((pad <= bus->roundup) && (pad < bus->blocksize))
			len += pad;
<<<<<<< HEAD
	} else if (len % DHD_SDALIGN) {
		len += DHD_SDALIGN - (len % DHD_SDALIGN);
=======
	} else if (len % BRCMF_SDALIGN) {
		len += BRCMF_SDALIGN - (len % BRCMF_SDALIGN);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	}

	/* Satisfy length-alignment requirements */
	if (forcealign && (len & (ALIGNMENT - 1)))
		len = roundup(len, ALIGNMENT);

<<<<<<< HEAD
	ASSERT(IS_ALIGNED((unsigned long)frame, 2));

	/* Need to lock here to protect txseq and SDIO tx calls */
	dhd_os_sdlock(bus->dhd);
=======
	/* precondition: IS_ALIGNED((unsigned long)frame, 2) */

	/* Need to lock here to protect txseq and SDIO tx calls */
	brcmf_sdbrcm_sdlock(bus);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	BUS_WAKE(bus);

	/* Make sure backplane clock is on */
<<<<<<< HEAD
	dhdsdio_clkctl(bus, CLK_AVAIL, false);
=======
	brcmf_sdbrcm_clkctl(bus, CLK_AVAIL, false);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	/* Hardware tag: 2 byte len followed by 2 byte ~len check (all LE) */
	*(u16 *) frame = cpu_to_le16((u16) msglen);
	*(((u16 *) frame) + 1) = cpu_to_le16(~msglen);

	/* Software tag: channel, sequence number, data offset */
	swheader =
	    ((SDPCM_CONTROL_CHANNEL << SDPCM_CHANNEL_SHIFT) &
	     SDPCM_CHANNEL_MASK)
	    | bus->tx_seq | ((doff << SDPCM_DOFFSET_SHIFT) &
			     SDPCM_DOFFSET_MASK);
	put_unaligned_le32(swheader, frame + SDPCM_FRAMETAG_LEN);
	put_unaligned_le32(0, frame + SDPCM_FRAMETAG_LEN + sizeof(swheader));

	if (!DATAOK(bus)) {
<<<<<<< HEAD
		DHD_INFO(("%s: No bus credit bus->tx_max %d, bus->tx_seq %d\n",
			  __func__, bus->tx_max, bus->tx_seq));
=======
		BRCMF_INFO(("%s: No bus credit bus->tx_max %d,"
			    " bus->tx_seq %d\n", __func__,
			    bus->tx_max, bus->tx_seq));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		bus->ctrl_frame_stat = true;
		/* Send from dpc */
		bus->ctrl_frame_buf = frame;
		bus->ctrl_frame_len = len;

<<<<<<< HEAD
		dhd_wait_for_event(bus->dhd, &bus->ctrl_frame_stat);

		if (bus->ctrl_frame_stat == false) {
			DHD_INFO(("%s: ctrl_frame_stat == false\n", __func__));
			ret = 0;
		} else {
			DHD_INFO(("%s: ctrl_frame_stat == true\n", __func__));
=======
		brcmf_sdbrcm_wait_for_event(bus, &bus->ctrl_frame_stat);

		if (bus->ctrl_frame_stat == false) {
			BRCMF_INFO(("%s: ctrl_frame_stat == false\n",
				    __func__));
			ret = 0;
		} else {
			BRCMF_INFO(("%s: ctrl_frame_stat == true\n", __func__));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			ret = -1;
		}
	}

	if (ret == -1) {
<<<<<<< HEAD
#ifdef DHD_DEBUG
		if (DHD_BYTES_ON() && DHD_CTL_ON()) {
			printk(KERN_DEBUG "Tx Frame:\n");
			print_hex_dump_bytes("", DUMP_PREFIX_OFFSET,
					     frame, len);
		} else if (DHD_HDRS_ON()) {
=======
#ifdef BCMDBG
		if (BRCMF_BYTES_ON() && BRCMF_CTL_ON()) {
			printk(KERN_DEBUG "Tx Frame:\n");
			print_hex_dump_bytes("", DUMP_PREFIX_OFFSET,
					     frame, len);
		} else if (BRCMF_HDRS_ON()) {
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			printk(KERN_DEBUG "TxHdr:\n");
			print_hex_dump_bytes("", DUMP_PREFIX_OFFSET,
					     frame, min_t(u16, len, 16));
		}
#endif

		do {
			bus->ctrl_frame_stat = false;
<<<<<<< HEAD
			ret =
			    dhd_bcmsdh_send_buf(bus, bcmsdh_cur_sbwad(sdh),
						SDIO_FUNC_2, F2SYNC, frame, len,
						NULL, NULL, NULL);

			ASSERT(ret != -BCME_PENDING);
=======
			ret = brcmf_sdbrcm_send_buf(bus,
				brcmf_sdcard_cur_sbwad(card), SDIO_FUNC_2,
				F2SYNC, frame, len, NULL, NULL, NULL);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

			if (ret < 0) {
				/* On failure, abort the command and
				 terminate the frame */
<<<<<<< HEAD
				DHD_INFO(("%s: sdio error %d, abort command and terminate frame.\n",
					__func__, ret));
				bus->tx_sderrs++;

				bcmsdh_abort(sdh, SDIO_FUNC_2);

				bcmsdh_cfg_write(sdh, SDIO_FUNC_1,
=======
				BRCMF_INFO(("%s: sdio error %d, abort command "
					    "and terminate frame.\n",
					    __func__, ret));
				bus->tx_sderrs++;

				brcmf_sdcard_abort(card, SDIO_FUNC_2);

				brcmf_sdcard_cfg_write(card, SDIO_FUNC_1,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
						 SBSDIO_FUNC1_FRAMECTRL,
						 SFC_WF_TERM, NULL);
				bus->f1regdata++;

				for (i = 0; i < 3; i++) {
					u8 hi, lo;
<<<<<<< HEAD
					hi = bcmsdh_cfg_read(sdh, SDIO_FUNC_1,
					     SBSDIO_FUNC1_WFRAMEBCHI,
					     NULL);
					lo = bcmsdh_cfg_read(sdh, SDIO_FUNC_1,
					     SBSDIO_FUNC1_WFRAMEBCLO,
							     NULL);
=======
					hi = brcmf_sdcard_cfg_read(card,
					     SDIO_FUNC_1,
					     SBSDIO_FUNC1_WFRAMEBCHI,
					     NULL);
					lo = brcmf_sdcard_cfg_read(card,
					     SDIO_FUNC_1,
					     SBSDIO_FUNC1_WFRAMEBCLO,
					     NULL);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
					bus->f1regdata += 2;
					if ((hi == 0) && (lo == 0))
						break;
				}

			}
			if (ret == 0) {
				bus->tx_seq =
				    (bus->tx_seq + 1) % SDPCM_SEQUENCE_WRAP;
			}
		} while ((ret < 0) && retries++ < TXRETRIES);
	}

<<<<<<< HEAD
	if ((bus->idletime == DHD_IDLE_IMMEDIATE) && !bus->dpc_sched) {
		bus->activity = false;
		dhdsdio_clkctl(bus, CLK_NONE, true);
	}

	dhd_os_sdunlock(bus->dhd);

	if (ret)
		bus->dhd->tx_ctlerrs++;
	else
		bus->dhd->tx_ctlpkts++;
=======
	if ((bus->idletime == BRCMF_IDLE_IMMEDIATE) && !bus->dpc_sched) {
		bus->activity = false;
		brcmf_sdbrcm_clkctl(bus, CLK_NONE, true);
	}

	brcmf_sdbrcm_sdunlock(bus);

	if (ret)
		bus->drvr->tx_ctlerrs++;
	else
		bus->drvr->tx_ctlpkts++;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	return ret ? -EIO : 0;
}

<<<<<<< HEAD
int dhd_bus_rxctl(struct dhd_bus *bus, unsigned char *msg, uint msglen)
=======
int brcmf_sdbrcm_bus_rxctl(struct brcmf_bus *bus, unsigned char *msg, uint msglen)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
{
	int timeleft;
	uint rxlen = 0;
	bool pending;

<<<<<<< HEAD
	DHD_TRACE(("%s: Enter\n", __func__));

	if (bus->dhd->dongle_reset)
		return -EIO;

	/* Wait until control frame is available */
	timeleft = dhd_os_ioctl_resp_wait(bus->dhd, &bus->rxlen, &pending);

	dhd_os_sdlock(bus->dhd);
	rxlen = bus->rxlen;
	memcpy(msg, bus->rxctl, min(msglen, rxlen));
	bus->rxlen = 0;
	dhd_os_sdunlock(bus->dhd);

	if (rxlen) {
		DHD_CTL(("%s: resumed on rxctl frame, got %d expected %d\n",
			 __func__, rxlen, msglen));
	} else if (timeleft == 0) {
		DHD_ERROR(("%s: resumed on timeout\n", __func__));
#ifdef DHD_DEBUG
		dhd_os_sdlock(bus->dhd);
		dhdsdio_checkdied(bus, NULL, 0);
		dhd_os_sdunlock(bus->dhd);
#endif				/* DHD_DEBUG */
	} else if (pending == true) {
		DHD_CTL(("%s: cancelled\n", __func__));
		return -ERESTARTSYS;
	} else {
		DHD_CTL(("%s: resumed for unknown reason?\n", __func__));
#ifdef DHD_DEBUG
		dhd_os_sdlock(bus->dhd);
		dhdsdio_checkdied(bus, NULL, 0);
		dhd_os_sdunlock(bus->dhd);
#endif				/* DHD_DEBUG */
	}

	if (rxlen)
		bus->dhd->rx_ctlpkts++;
	else
		bus->dhd->rx_ctlerrs++;
=======
	BRCMF_TRACE(("%s: Enter\n", __func__));

	if (bus->drvr->dongle_reset)
		return -EIO;

	/* Wait until control frame is available */
	timeleft = brcmf_os_ioctl_resp_wait(bus->drvr, &bus->rxlen, &pending);

	brcmf_sdbrcm_sdlock(bus);
	rxlen = bus->rxlen;
	memcpy(msg, bus->rxctl, min(msglen, rxlen));
	bus->rxlen = 0;
	brcmf_sdbrcm_sdunlock(bus);

	if (rxlen) {
		BRCMF_CTL(("%s: resumed on rxctl frame, got %d expected %d\n",
			   __func__, rxlen, msglen));
	} else if (timeleft == 0) {
		BRCMF_ERROR(("%s: resumed on timeout\n", __func__));
#ifdef BCMDBG
		brcmf_sdbrcm_sdlock(bus);
		brcmf_sdbrcm_checkdied(bus, NULL, 0);
		brcmf_sdbrcm_sdunlock(bus);
#endif				/* BCMDBG */
	} else if (pending == true) {
		BRCMF_CTL(("%s: cancelled\n", __func__));
		return -ERESTARTSYS;
	} else {
		BRCMF_CTL(("%s: resumed for unknown reason?\n", __func__));
#ifdef BCMDBG
		brcmf_sdbrcm_sdlock(bus);
		brcmf_sdbrcm_checkdied(bus, NULL, 0);
		brcmf_sdbrcm_sdunlock(bus);
#endif				/* BCMDBG */
	}

	if (rxlen)
		bus->drvr->rx_ctlpkts++;
	else
		bus->drvr->rx_ctlerrs++;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	return rxlen ? (int)rxlen : -ETIMEDOUT;
}

/* IOVar table */
enum {
	IOV_INTR = 1,
	IOV_POLLRATE,
	IOV_SDREG,
	IOV_SBREG,
	IOV_SDCIS,
	IOV_MEMBYTES,
	IOV_MEMSIZE,
<<<<<<< HEAD
#ifdef DHD_DEBUG
	IOV_CHECKDIED,
=======
#ifdef BCMDBG
	IOV_CHECKDIED,
	IOV_CONS,
	IOV_DCONSOLE_POLL,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
#endif
	IOV_DOWNLOAD,
	IOV_FORCEEVEN,
	IOV_SDIOD_DRIVE,
	IOV_READAHEAD,
	IOV_SDRXCHAIN,
	IOV_ALIGNCTL,
	IOV_SDALIGN,
	IOV_DEVRESET,
	IOV_CPU,
#ifdef SDTEST
	IOV_PKTGEN,
	IOV_EXTLOOP,
#endif				/* SDTEST */
	IOV_SPROM,
	IOV_TXBOUND,
	IOV_RXBOUND,
	IOV_TXMINMAX,
	IOV_IDLETIME,
	IOV_IDLECLOCK,
	IOV_SD1IDLE,
	IOV_SLEEP,
<<<<<<< HEAD
	IOV_VARS
};

const bcm_iovar_t dhdsdio_iovars[] = {
=======
	IOV_WDTICK,
	IOV_VARS
};

const struct brcmu_iovar brcmf_sdio_iovars[] = {
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	{"intr", IOV_INTR, 0, IOVT_BOOL, 0},
	{"sleep", IOV_SLEEP, 0, IOVT_BOOL, 0},
	{"pollrate", IOV_POLLRATE, 0, IOVT_UINT32, 0},
	{"idletime", IOV_IDLETIME, 0, IOVT_INT32, 0},
	{"idleclock", IOV_IDLECLOCK, 0, IOVT_INT32, 0},
	{"sd1idle", IOV_SD1IDLE, 0, IOVT_BOOL, 0},
	{"membytes", IOV_MEMBYTES, 0, IOVT_BUFFER, 2 * sizeof(int)},
	{"memsize", IOV_MEMSIZE, 0, IOVT_UINT32, 0},
	{"download", IOV_DOWNLOAD, 0, IOVT_BOOL, 0},
	{"vars", IOV_VARS, 0, IOVT_BUFFER, 0},
	{"sdiod_drive", IOV_SDIOD_DRIVE, 0, IOVT_UINT32, 0},
	{"readahead", IOV_READAHEAD, 0, IOVT_BOOL, 0},
	{"sdrxchain", IOV_SDRXCHAIN, 0, IOVT_BOOL, 0},
	{"alignctl", IOV_ALIGNCTL, 0, IOVT_BOOL, 0},
	{"sdalign", IOV_SDALIGN, 0, IOVT_BOOL, 0},
	{"devreset", IOV_DEVRESET, 0, IOVT_BOOL, 0},
<<<<<<< HEAD
#ifdef DHD_DEBUG
	{"sdreg", IOV_SDREG, 0, IOVT_BUFFER, sizeof(sdreg_t)}
	,
	{"sbreg", IOV_SBREG, 0, IOVT_BUFFER, sizeof(sdreg_t)}
	,
	{"sd_cis", IOV_SDCIS, 0, IOVT_BUFFER, DHD_IOCTL_MAXLEN}
=======
	{"wdtick", IOV_WDTICK, 0, IOVT_UINT32, 0},
#ifdef BCMDBG
	{"cons", IOV_CONS, 0, IOVT_BUFFER, 0}
	,
	{"dconpoll", IOV_DCONSOLE_POLL, 0, IOVT_UINT32, 0}
	,
	{"sdreg", IOV_SDREG, 0, IOVT_BUFFER, sizeof(struct brcmf_sdreg)}
	,
	{"sbreg", IOV_SBREG, 0, IOVT_BUFFER, sizeof(struct brcmf_sdreg)}
	,
	{"sd_cis", IOV_SDCIS, 0, IOVT_BUFFER, BRCMF_IOCTL_MAXLEN}
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	,
	{"forcealign", IOV_FORCEEVEN, 0, IOVT_BOOL, 0}
	,
	{"txbound", IOV_TXBOUND, 0, IOVT_UINT32, 0}
	,
	{"rxbound", IOV_RXBOUND, 0, IOVT_UINT32, 0}
	,
	{"txminmax", IOV_TXMINMAX, 0, IOVT_UINT32, 0}
	,
	{"cpu", IOV_CPU, 0, IOVT_BOOL, 0}
	,
<<<<<<< HEAD
#ifdef DHD_DEBUG
	{"checkdied", IOV_CHECKDIED, 0, IOVT_BUFFER, 0}
	,
#endif				/* DHD_DEBUG  */
#endif				/* DHD_DEBUG */
#ifdef SDTEST
	{"extloop", IOV_EXTLOOP, 0, IOVT_BOOL, 0}
	,
	{"pktgen", IOV_PKTGEN, 0, IOVT_BUFFER, sizeof(dhd_pktgen_t)}
=======
	{"checkdied", IOV_CHECKDIED, 0, IOVT_BUFFER, 0}
	,
#endif				/* BCMDBG */
#ifdef SDTEST
	{"extloop", IOV_EXTLOOP, 0, IOVT_BOOL, 0}
	,
	{"pktgen", IOV_PKTGEN, 0, IOVT_BUFFER, sizeof(struct brcmf_pktgen)}
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	,
#endif				/* SDTEST */

	{NULL, 0, 0, 0, 0}
};

static void
<<<<<<< HEAD
dhd_dump_pct(struct bcmstrbuf *strbuf, char *desc, uint num, uint div)
=======
brcmf_dump_pct(struct brcmu_strbuf *strbuf, char *desc, uint num, uint div)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
{
	uint q1, q2;

	if (!div) {
<<<<<<< HEAD
		bcm_bprintf(strbuf, "%s N/A", desc);
	} else {
		q1 = num / div;
		q2 = (100 * (num - (q1 * div))) / div;
		bcm_bprintf(strbuf, "%s %d.%02d", desc, q1, q2);
	}
}

void dhd_bus_dump(dhd_pub_t *dhdp, struct bcmstrbuf *strbuf)
{
	dhd_bus_t *bus = dhdp->bus;

	bcm_bprintf(strbuf, "Bus SDIO structure:\n");
	bcm_bprintf(strbuf,
		    "hostintmask 0x%08x intstatus 0x%08x sdpcm_ver %d\n",
		    bus->hostintmask, bus->intstatus, bus->sdpcm_ver);
	bcm_bprintf(strbuf,
		    "fcstate %d qlen %d tx_seq %d, max %d, rxskip %d rxlen %d rx_seq %d\n",
		    bus->fcstate, pktq_len(&bus->txq), bus->tx_seq, bus->tx_max,
		    bus->rxskip, bus->rxlen, bus->rx_seq);
	bcm_bprintf(strbuf, "intr %d intrcount %d lastintrs %d spurious %d\n",
		    bus->intr, bus->intrcount, bus->lastintrs, bus->spurious);
	bcm_bprintf(strbuf, "pollrate %d pollcnt %d regfails %d\n",
		    bus->pollrate, bus->pollcnt, bus->regfails);

	bcm_bprintf(strbuf, "\nAdditional counters:\n");
	bcm_bprintf(strbuf,
		    "tx_sderrs %d fcqueued %d rxrtx %d rx_toolong %d rxc_errors %d\n",
		    bus->tx_sderrs, bus->fcqueued, bus->rxrtx, bus->rx_toolong,
		    bus->rxc_errors);
	bcm_bprintf(strbuf, "rx_hdrfail %d badhdr %d badseq %d\n",
		    bus->rx_hdrfail, bus->rx_badhdr, bus->rx_badseq);
	bcm_bprintf(strbuf, "fc_rcvd %d, fc_xoff %d, fc_xon %d\n", bus->fc_rcvd,
		    bus->fc_xoff, bus->fc_xon);
	bcm_bprintf(strbuf, "rxglomfail %d, rxglomframes %d, rxglompkts %d\n",
		    bus->rxglomfail, bus->rxglomframes, bus->rxglompkts);
	bcm_bprintf(strbuf, "f2rx (hdrs/data) %d (%d/%d), f2tx %d f1regs %d\n",
		    (bus->f2rxhdrs + bus->f2rxdata), bus->f2rxhdrs,
		    bus->f2rxdata, bus->f2txdata, bus->f1regdata);
	{
		dhd_dump_pct(strbuf, "\nRx: pkts/f2rd", bus->dhd->rx_packets,
			     (bus->f2rxhdrs + bus->f2rxdata));
		dhd_dump_pct(strbuf, ", pkts/f1sd", bus->dhd->rx_packets,
			     bus->f1regdata);
		dhd_dump_pct(strbuf, ", pkts/sd", bus->dhd->rx_packets,
			     (bus->f2rxhdrs + bus->f2rxdata + bus->f1regdata));
		dhd_dump_pct(strbuf, ", pkts/int", bus->dhd->rx_packets,
			     bus->intrcount);
		bcm_bprintf(strbuf, "\n");

		dhd_dump_pct(strbuf, "Rx: glom pct", (100 * bus->rxglompkts),
			     bus->dhd->rx_packets);
		dhd_dump_pct(strbuf, ", pkts/glom", bus->rxglompkts,
			     bus->rxglomframes);
		bcm_bprintf(strbuf, "\n");

		dhd_dump_pct(strbuf, "Tx: pkts/f2wr", bus->dhd->tx_packets,
			     bus->f2txdata);
		dhd_dump_pct(strbuf, ", pkts/f1sd", bus->dhd->tx_packets,
			     bus->f1regdata);
		dhd_dump_pct(strbuf, ", pkts/sd", bus->dhd->tx_packets,
			     (bus->f2txdata + bus->f1regdata));
		dhd_dump_pct(strbuf, ", pkts/int", bus->dhd->tx_packets,
			     bus->intrcount);
		bcm_bprintf(strbuf, "\n");

		dhd_dump_pct(strbuf, "Total: pkts/f2rw",
			     (bus->dhd->tx_packets + bus->dhd->rx_packets),
			     (bus->f2txdata + bus->f2rxhdrs + bus->f2rxdata));
		dhd_dump_pct(strbuf, ", pkts/f1sd",
			     (bus->dhd->tx_packets + bus->dhd->rx_packets),
			     bus->f1regdata);
		dhd_dump_pct(strbuf, ", pkts/sd",
			     (bus->dhd->tx_packets + bus->dhd->rx_packets),
			     (bus->f2txdata + bus->f2rxhdrs + bus->f2rxdata +
			      bus->f1regdata));
		dhd_dump_pct(strbuf, ", pkts/int",
			     (bus->dhd->tx_packets + bus->dhd->rx_packets),
			     bus->intrcount);
		bcm_bprintf(strbuf, "\n\n");
=======
		brcmu_bprintf(strbuf, "%s N/A", desc);
	} else {
		q1 = num / div;
		q2 = (100 * (num - (q1 * div))) / div;
		brcmu_bprintf(strbuf, "%s %d.%02d", desc, q1, q2);
	}
}

void brcmf_sdbrcm_bus_dump(struct brcmf_pub *drvr, struct brcmu_strbuf *strbuf)
{
	struct brcmf_bus *bus = drvr->bus;

	brcmu_bprintf(strbuf, "Bus SDIO structure:\n");
	brcmu_bprintf(strbuf,
		    "hostintmask 0x%08x intstatus 0x%08x sdpcm_ver %d\n",
		    bus->hostintmask, bus->intstatus, bus->sdpcm_ver);
	brcmu_bprintf(strbuf,
		    "fcstate %d qlen %d tx_seq %d, max %d, rxskip %d rxlen %d rx_seq %d\n",
		    bus->fcstate, pktq_len(&bus->txq), bus->tx_seq, bus->tx_max,
		    bus->rxskip, bus->rxlen, bus->rx_seq);
	brcmu_bprintf(strbuf, "intr %d intrcount %d lastintrs %d spurious %d\n",
		    bus->intr, bus->intrcount, bus->lastintrs, bus->spurious);
	brcmu_bprintf(strbuf, "pollrate %d pollcnt %d regfails %d\n",
		    bus->pollrate, bus->pollcnt, bus->regfails);

	brcmu_bprintf(strbuf, "\nAdditional counters:\n");
	brcmu_bprintf(strbuf,
		    "tx_sderrs %d fcqueued %d rxrtx %d rx_toolong %d rxc_errors %d\n",
		    bus->tx_sderrs, bus->fcqueued, bus->rxrtx, bus->rx_toolong,
		    bus->rxc_errors);
	brcmu_bprintf(strbuf, "rx_hdrfail %d badhdr %d badseq %d\n",
		    bus->rx_hdrfail, bus->rx_badhdr, bus->rx_badseq);
	brcmu_bprintf(strbuf, "fc_rcvd %d, fc_xoff %d, fc_xon %d\n",
		      bus->fc_rcvd, bus->fc_xoff, bus->fc_xon);
	brcmu_bprintf(strbuf, "rxglomfail %d, rxglomframes %d, rxglompkts %d\n",
		    bus->rxglomfail, bus->rxglomframes, bus->rxglompkts);
	brcmu_bprintf(strbuf, "f2rx (hdrs/data) %d (%d/%d), f2tx %d f1regs"
		      " %d\n",
		      (bus->f2rxhdrs + bus->f2rxdata), bus->f2rxhdrs,
		      bus->f2rxdata, bus->f2txdata, bus->f1regdata);
	{
		brcmf_dump_pct(strbuf, "\nRx: pkts/f2rd", bus->drvr->rx_packets,
			     (bus->f2rxhdrs + bus->f2rxdata));
		brcmf_dump_pct(strbuf, ", pkts/f1sd", bus->drvr->rx_packets,
			     bus->f1regdata);
		brcmf_dump_pct(strbuf, ", pkts/sd", bus->drvr->rx_packets,
			     (bus->f2rxhdrs + bus->f2rxdata + bus->f1regdata));
		brcmf_dump_pct(strbuf, ", pkts/int", bus->drvr->rx_packets,
			     bus->intrcount);
		brcmu_bprintf(strbuf, "\n");

		brcmf_dump_pct(strbuf, "Rx: glom pct", (100 * bus->rxglompkts),
			     bus->drvr->rx_packets);
		brcmf_dump_pct(strbuf, ", pkts/glom", bus->rxglompkts,
			     bus->rxglomframes);
		brcmu_bprintf(strbuf, "\n");

		brcmf_dump_pct(strbuf, "Tx: pkts/f2wr", bus->drvr->tx_packets,
			     bus->f2txdata);
		brcmf_dump_pct(strbuf, ", pkts/f1sd", bus->drvr->tx_packets,
			     bus->f1regdata);
		brcmf_dump_pct(strbuf, ", pkts/sd", bus->drvr->tx_packets,
			     (bus->f2txdata + bus->f1regdata));
		brcmf_dump_pct(strbuf, ", pkts/int", bus->drvr->tx_packets,
			     bus->intrcount);
		brcmu_bprintf(strbuf, "\n");

		brcmf_dump_pct(strbuf, "Total: pkts/f2rw",
			     (bus->drvr->tx_packets + bus->drvr->rx_packets),
			     (bus->f2txdata + bus->f2rxhdrs + bus->f2rxdata));
		brcmf_dump_pct(strbuf, ", pkts/f1sd",
			     (bus->drvr->tx_packets + bus->drvr->rx_packets),
			     bus->f1regdata);
		brcmf_dump_pct(strbuf, ", pkts/sd",
			     (bus->drvr->tx_packets + bus->drvr->rx_packets),
			     (bus->f2txdata + bus->f2rxhdrs + bus->f2rxdata +
			      bus->f1regdata));
		brcmf_dump_pct(strbuf, ", pkts/int",
			     (bus->drvr->tx_packets + bus->drvr->rx_packets),
			     bus->intrcount);
		brcmu_bprintf(strbuf, "\n\n");
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	}

#ifdef SDTEST
	if (bus->pktgen_count) {
<<<<<<< HEAD
		bcm_bprintf(strbuf, "pktgen config and count:\n");
		bcm_bprintf(strbuf,
=======
		brcmu_bprintf(strbuf, "pktgen config and count:\n");
		brcmu_bprintf(strbuf,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			    "freq %d count %d print %d total %d min %d len %d\n",
			    bus->pktgen_freq, bus->pktgen_count,
			    bus->pktgen_print, bus->pktgen_total,
			    bus->pktgen_minlen, bus->pktgen_maxlen);
<<<<<<< HEAD
		bcm_bprintf(strbuf, "send attempts %d rcvd %d fail %d\n",
=======
		brcmu_bprintf(strbuf, "send attempts %d rcvd %d fail %d\n",
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			    bus->pktgen_sent, bus->pktgen_rcvd,
			    bus->pktgen_fail);
	}
#endif				/* SDTEST */
<<<<<<< HEAD
#ifdef DHD_DEBUG
	bcm_bprintf(strbuf, "dpc_sched %d host interrupt%spending\n",
		    bus->dpc_sched,
		    (bcmsdh_intr_pending(bus->sdh) ? " " : " not "));
	bcm_bprintf(strbuf, "blocksize %d roundup %d\n", bus->blocksize,
		    bus->roundup);
#endif				/* DHD_DEBUG */
	bcm_bprintf(strbuf,
=======
#ifdef BCMDBG
	brcmu_bprintf(strbuf, "dpc_sched %d host interrupt%spending\n",
		      bus->dpc_sched, " not ");
	brcmu_bprintf(strbuf, "blocksize %d roundup %d\n", bus->blocksize,
		    bus->roundup);
#endif				/* BCMDBG */
	brcmu_bprintf(strbuf,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		    "clkstate %d activity %d idletime %d idlecount %d sleeping %d\n",
		    bus->clkstate, bus->activity, bus->idletime, bus->idlecount,
		    bus->sleeping);
}

<<<<<<< HEAD
void dhd_bus_clearcounts(dhd_pub_t *dhdp)
{
	dhd_bus_t *bus = (dhd_bus_t *) dhdp->bus;
=======
void brcmf_bus_clearcounts(struct brcmf_pub *drvr)
{
	struct brcmf_bus *bus = (struct brcmf_bus *) drvr->bus;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	bus->intrcount = bus->lastintrs = bus->spurious = bus->regfails = 0;
	bus->rxrtx = bus->rx_toolong = bus->rxc_errors = 0;
	bus->rx_hdrfail = bus->rx_badhdr = bus->rx_badseq = 0;
	bus->tx_sderrs = bus->fc_rcvd = bus->fc_xoff = bus->fc_xon = 0;
	bus->rxglomfail = bus->rxglomframes = bus->rxglompkts = 0;
	bus->f2rxhdrs = bus->f2rxdata = bus->f2txdata = bus->f1regdata = 0;
}

#ifdef SDTEST
<<<<<<< HEAD
static int dhdsdio_pktgen_get(dhd_bus_t *bus, u8 *arg)
{
	dhd_pktgen_t pktgen;

	pktgen.version = DHD_PKTGEN_VERSION;
=======
static int brcmf_sdbrcm_pktgen_get(struct brcmf_bus *bus, u8 *arg)
{
	struct brcmf_pktgen pktgen;

	pktgen.version = BRCMF_PKTGEN_VERSION;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	pktgen.freq = bus->pktgen_freq;
	pktgen.count = bus->pktgen_count;
	pktgen.print = bus->pktgen_print;
	pktgen.total = bus->pktgen_total;
	pktgen.minlen = bus->pktgen_minlen;
	pktgen.maxlen = bus->pktgen_maxlen;
	pktgen.numsent = bus->pktgen_sent;
	pktgen.numrcvd = bus->pktgen_rcvd;
	pktgen.numfail = bus->pktgen_fail;
	pktgen.mode = bus->pktgen_mode;
	pktgen.stop = bus->pktgen_stop;

	memcpy(arg, &pktgen, sizeof(pktgen));

	return 0;
}

<<<<<<< HEAD
static int dhdsdio_pktgen_set(dhd_bus_t *bus, u8 *arg)
{
	dhd_pktgen_t pktgen;
	uint oldcnt, oldmode;

	memcpy(&pktgen, arg, sizeof(pktgen));
	if (pktgen.version != DHD_PKTGEN_VERSION)
=======
static int brcmf_sdbrcm_pktgen_set(struct brcmf_bus *bus, u8 *arg)
{
	struct brcmf_pktgen pktgen;
	uint oldcnt, oldmode;

	memcpy(&pktgen, arg, sizeof(pktgen));
	if (pktgen.version != BRCMF_PKTGEN_VERSION)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		return -EINVAL;

	oldcnt = bus->pktgen_count;
	oldmode = bus->pktgen_mode;

	bus->pktgen_freq = pktgen.freq;
	bus->pktgen_count = pktgen.count;
	bus->pktgen_print = pktgen.print;
	bus->pktgen_total = pktgen.total;
	bus->pktgen_minlen = pktgen.minlen;
	bus->pktgen_maxlen = pktgen.maxlen;
	bus->pktgen_mode = pktgen.mode;
	bus->pktgen_stop = pktgen.stop;

	bus->pktgen_tick = bus->pktgen_ptick = 0;
	bus->pktgen_len = max(bus->pktgen_len, bus->pktgen_minlen);
	bus->pktgen_len = min(bus->pktgen_len, bus->pktgen_maxlen);

	/* Clear counts for a new pktgen (mode change, or was stopped) */
	if (bus->pktgen_count && (!oldcnt || oldmode != bus->pktgen_mode))
		bus->pktgen_sent = bus->pktgen_rcvd = bus->pktgen_fail = 0;

	return 0;
}
#endif				/* SDTEST */

static int
<<<<<<< HEAD
dhdsdio_membytes(dhd_bus_t *bus, bool write, u32 address, u8 *data,
=======
brcmf_sdbrcm_membytes(struct brcmf_bus *bus, bool write, u32 address, u8 *data,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		 uint size)
{
	int bcmerror = 0;
	u32 sdaddr;
	uint dsize;

	/* Determine initial transfer parameters */
	sdaddr = address & SBSDIO_SB_OFT_ADDR_MASK;
	if ((sdaddr + size) & SBSDIO_SBWINDOW_MASK)
		dsize = (SBSDIO_SB_OFT_ADDR_LIMIT - sdaddr);
	else
		dsize = size;

	/* Set the backplane window to include the start address */
<<<<<<< HEAD
	bcmerror = dhdsdio_set_siaddr_window(bus, address);
	if (bcmerror) {
		DHD_ERROR(("%s: window change failed\n", __func__));
=======
	bcmerror = brcmf_sdbrcm_set_siaddr_window(bus, address);
	if (bcmerror) {
		BRCMF_ERROR(("%s: window change failed\n", __func__));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		goto xfer_done;
	}

	/* Do the transfer(s) */
	while (size) {
<<<<<<< HEAD
		DHD_INFO(("%s: %s %d bytes at offset 0x%08x in window 0x%08x\n",
			  __func__, (write ? "write" : "read"), dsize,
			  sdaddr, (address & SBSDIO_SBWINDOW_MASK)));
		bcmerror =
		     bcmsdh_rwdata(bus->sdh, write, sdaddr, data, dsize);
		if (bcmerror) {
			DHD_ERROR(("%s: membytes transfer failed\n", __func__));
=======
		BRCMF_INFO(("%s: %s %d bytes at offset 0x%08x in window"
			    " 0x%08x\n", __func__, (write ? "write" : "read"),
			    dsize, sdaddr, (address & SBSDIO_SBWINDOW_MASK)));
		bcmerror =
		     brcmf_sdcard_rwdata(bus->card, write, sdaddr, data, dsize);
		if (bcmerror) {
			BRCMF_ERROR(("%s: membytes transfer failed\n",
				     __func__));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			break;
		}

		/* Adjust for next transfer (if any) */
		size -= dsize;
		if (size) {
			data += dsize;
			address += dsize;
<<<<<<< HEAD
			bcmerror = dhdsdio_set_siaddr_window(bus, address);
			if (bcmerror) {
				DHD_ERROR(("%s: window change failed\n",
					   __func__));
=======
			bcmerror = brcmf_sdbrcm_set_siaddr_window(bus, address);
			if (bcmerror) {
				BRCMF_ERROR(("%s: window change failed\n",
					     __func__));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				break;
			}
			sdaddr = 0;
			dsize = min_t(uint, SBSDIO_SB_OFT_ADDR_LIMIT, size);
		}
	}

xfer_done:
	/* Return the window to backplane enumeration space for core access */
<<<<<<< HEAD
	if (dhdsdio_set_siaddr_window(bus, bcmsdh_cur_sbwad(bus->sdh))) {
		DHD_ERROR(("%s: FAILED to set window back to 0x%x\n",
			   __func__, bcmsdh_cur_sbwad(bus->sdh)));
=======
	if (brcmf_sdbrcm_set_siaddr_window(bus,
					   brcmf_sdcard_cur_sbwad(bus->card))) {
		BRCMF_ERROR(("%s: FAILED to set window back to 0x%x\n",
			     __func__, brcmf_sdcard_cur_sbwad(bus->card)));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	}

	return bcmerror;
}

<<<<<<< HEAD
#ifdef DHD_DEBUG
static int dhdsdio_readshared(dhd_bus_t *bus, sdpcm_shared_t *sh)
=======
#ifdef BCMDBG
static int brcmf_sdbrcm_readshared(struct brcmf_bus *bus, struct sdpcm_shared *sh)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
{
	u32 addr;
	int rv;

	/* Read last word in memory to determine address of
			 sdpcm_shared structure */
<<<<<<< HEAD
	rv = dhdsdio_membytes(bus, false, bus->ramsize - 4, (u8 *)&addr, 4);
=======
	rv = brcmf_sdbrcm_membytes(bus, false, bus->ramsize - 4, (u8 *)&addr,
				   4);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	if (rv < 0)
		return rv;

	addr = le32_to_cpu(addr);

<<<<<<< HEAD
	DHD_INFO(("sdpcm_shared address 0x%08X\n", addr));
=======
	BRCMF_INFO(("sdpcm_shared address 0x%08X\n", addr));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	/*
	 * Check if addr is valid.
	 * NVRAM length at the end of memory should have been overwritten.
	 */
	if (addr == 0 || ((~addr >> 16) & 0xffff) == (addr & 0xffff)) {
<<<<<<< HEAD
		DHD_ERROR(("%s: address (0x%08x) of sdpcm_shared invalid\n",
			   __func__, addr));
		return -EBADE;
	}

	/* Read hndrte_shared structure */
	rv = dhdsdio_membytes(bus, false, addr, (u8 *) sh,
			      sizeof(sdpcm_shared_t));
=======
		BRCMF_ERROR(("%s: address (0x%08x) of sdpcm_shared invalid\n",
			     __func__, addr));
		return -EBADE;
	}

	/* Read rte_shared structure */
	rv = brcmf_sdbrcm_membytes(bus, false, addr, (u8 *) sh,
			      sizeof(struct sdpcm_shared));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	if (rv < 0)
		return rv;

	/* Endianness */
	sh->flags = le32_to_cpu(sh->flags);
	sh->trap_addr = le32_to_cpu(sh->trap_addr);
	sh->assert_exp_addr = le32_to_cpu(sh->assert_exp_addr);
	sh->assert_file_addr = le32_to_cpu(sh->assert_file_addr);
	sh->assert_line = le32_to_cpu(sh->assert_line);
	sh->console_addr = le32_to_cpu(sh->console_addr);
	sh->msgtrace_addr = le32_to_cpu(sh->msgtrace_addr);

	if ((sh->flags & SDPCM_SHARED_VERSION_MASK) != SDPCM_SHARED_VERSION) {
<<<<<<< HEAD
		DHD_ERROR(("%s: sdpcm_shared version %d in dhd "
			   "is different than sdpcm_shared version %d in dongle\n",
			   __func__, SDPCM_SHARED_VERSION,
			   sh->flags & SDPCM_SHARED_VERSION_MASK));
=======
		BRCMF_ERROR(("%s: sdpcm_shared version %d in brcmf "
			     "is different than sdpcm_shared version %d in dongle\n",
			     __func__, SDPCM_SHARED_VERSION,
			     sh->flags & SDPCM_SHARED_VERSION_MASK));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		return -EBADE;
	}

	return 0;
}

<<<<<<< HEAD
static int dhdsdio_checkdied(dhd_bus_t *bus, u8 *data, uint size)
=======
static int brcmf_sdbrcm_checkdied(struct brcmf_bus *bus, u8 *data, uint size)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
{
	int bcmerror = 0;
	uint msize = 512;
	char *mbuffer = NULL;
	uint maxstrlen = 256;
	char *str = NULL;
<<<<<<< HEAD
	trap_t tr;
	sdpcm_shared_t sdpcm_shared;
	struct bcmstrbuf strbuf;

	DHD_TRACE(("%s: Enter\n", __func__));
=======
	struct brcmf_trap tr;
	struct sdpcm_shared sdpcm_shared;
	struct brcmu_strbuf strbuf;

	BRCMF_TRACE(("%s: Enter\n", __func__));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	if (data == NULL) {
		/*
		 * Called after a rx ctrl timeout. "data" is NULL.
		 * allocate memory to trace the trap or assert.
		 */
		size = msize;
		mbuffer = data = kmalloc(msize, GFP_ATOMIC);
		if (mbuffer == NULL) {
<<<<<<< HEAD
			DHD_ERROR(("%s: kmalloc(%d) failed\n", __func__,
				   msize));
=======
			BRCMF_ERROR(("%s: kmalloc(%d) failed\n", __func__,
				     msize));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			bcmerror = -ENOMEM;
			goto done;
		}
	}

	str = kmalloc(maxstrlen, GFP_ATOMIC);
	if (str == NULL) {
<<<<<<< HEAD
		DHD_ERROR(("%s: kmalloc(%d) failed\n", __func__, maxstrlen));
=======
		BRCMF_ERROR(("%s: kmalloc(%d) failed\n", __func__, maxstrlen));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		bcmerror = -ENOMEM;
		goto done;
	}

<<<<<<< HEAD
	bcmerror = dhdsdio_readshared(bus, &sdpcm_shared);
	if (bcmerror < 0)
		goto done;

	bcm_binit(&strbuf, data, size);

	bcm_bprintf(&strbuf,
=======
	bcmerror = brcmf_sdbrcm_readshared(bus, &sdpcm_shared);
	if (bcmerror < 0)
		goto done;

	brcmu_binit(&strbuf, data, size);

	brcmu_bprintf(&strbuf,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		    "msgtrace address : 0x%08X\nconsole address  : 0x%08X\n",
		    sdpcm_shared.msgtrace_addr, sdpcm_shared.console_addr);

	if ((sdpcm_shared.flags & SDPCM_SHARED_ASSERT_BUILT) == 0) {
		/* NOTE: Misspelled assert is intentional - DO NOT FIX.
		 * (Avoids conflict with real asserts for programmatic
		 * parsing of output.)
		 */
<<<<<<< HEAD
		bcm_bprintf(&strbuf, "Assrt not built in dongle\n");
=======
		brcmu_bprintf(&strbuf, "Assrt not built in dongle\n");
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	}

	if ((sdpcm_shared.flags & (SDPCM_SHARED_ASSERT | SDPCM_SHARED_TRAP)) ==
	    0) {
		/* NOTE: Misspelled assert is intentional - DO NOT FIX.
		 * (Avoids conflict with real asserts for programmatic
		 * parsing of output.)
		 */
<<<<<<< HEAD
		bcm_bprintf(&strbuf, "No trap%s in dongle",
=======
		brcmu_bprintf(&strbuf, "No trap%s in dongle",
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			    (sdpcm_shared.flags & SDPCM_SHARED_ASSERT_BUILT)
			    ? "/assrt" : "");
	} else {
		if (sdpcm_shared.flags & SDPCM_SHARED_ASSERT) {
			/* Download assert */
<<<<<<< HEAD
			bcm_bprintf(&strbuf, "Dongle assert");
			if (sdpcm_shared.assert_exp_addr != 0) {
				str[0] = '\0';
				bcmerror = dhdsdio_membytes(bus, false,
=======
			brcmu_bprintf(&strbuf, "Dongle assert");
			if (sdpcm_shared.assert_exp_addr != 0) {
				str[0] = '\0';
				bcmerror = brcmf_sdbrcm_membytes(bus, false,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
						sdpcm_shared.assert_exp_addr,
						(u8 *) str, maxstrlen);
				if (bcmerror < 0)
					goto done;

				str[maxstrlen - 1] = '\0';
<<<<<<< HEAD
				bcm_bprintf(&strbuf, " expr \"%s\"", str);
=======
				brcmu_bprintf(&strbuf, " expr \"%s\"", str);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			}

			if (sdpcm_shared.assert_file_addr != 0) {
				str[0] = '\0';
<<<<<<< HEAD
				bcmerror = dhdsdio_membytes(bus, false,
=======
				bcmerror = brcmf_sdbrcm_membytes(bus, false,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
						sdpcm_shared.assert_file_addr,
						(u8 *) str, maxstrlen);
				if (bcmerror < 0)
					goto done;

				str[maxstrlen - 1] = '\0';
<<<<<<< HEAD
				bcm_bprintf(&strbuf, " file \"%s\"", str);
			}

			bcm_bprintf(&strbuf, " line %d ",
=======
				brcmu_bprintf(&strbuf, " file \"%s\"", str);
			}

			brcmu_bprintf(&strbuf, " line %d ",
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				    sdpcm_shared.assert_line);
		}

		if (sdpcm_shared.flags & SDPCM_SHARED_TRAP) {
<<<<<<< HEAD
			bcmerror = dhdsdio_membytes(bus, false,
					sdpcm_shared.trap_addr, (u8 *)&tr,
					sizeof(trap_t));
			if (bcmerror < 0)
				goto done;

			bcm_bprintf(&strbuf,
=======
			bcmerror = brcmf_sdbrcm_membytes(bus, false,
					sdpcm_shared.trap_addr, (u8 *)&tr,
					sizeof(struct brcmf_trap));
			if (bcmerror < 0)
				goto done;

			brcmu_bprintf(&strbuf,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				    "Dongle trap type 0x%x @ epc 0x%x, cpsr 0x%x, spsr 0x%x, sp 0x%x,"
				    "lp 0x%x, rpc 0x%x Trap offset 0x%x, "
				    "r0 0x%x, r1 0x%x, r2 0x%x, r3 0x%x, r4 0x%x, r5 0x%x, r6 0x%x, r7 0x%x\n",
				    tr.type, tr.epc, tr.cpsr, tr.spsr, tr.r13,
				    tr.r14, tr.pc, sdpcm_shared.trap_addr,
				    tr.r0, tr.r1, tr.r2, tr.r3, tr.r4, tr.r5,
				    tr.r6, tr.r7);
		}
	}

	if (sdpcm_shared.flags & (SDPCM_SHARED_ASSERT | SDPCM_SHARED_TRAP))
<<<<<<< HEAD
		DHD_ERROR(("%s: %s\n", __func__, strbuf.origbuf));

#ifdef DHD_DEBUG
	if (sdpcm_shared.flags & SDPCM_SHARED_TRAP) {
		/* Mem dump to a file on device */
		dhdsdio_mem_dump(bus);
	}
#endif				/* DHD_DEBUG */
=======
		BRCMF_ERROR(("%s: %s\n", __func__, strbuf.origbuf));

#ifdef BCMDBG
	if (sdpcm_shared.flags & SDPCM_SHARED_TRAP) {
		/* Mem dump to a file on device */
		brcmf_sdbrcm_mem_dump(bus);
	}
#endif				/* BCMDBG */
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

done:
	kfree(mbuffer);
	kfree(str);

	return bcmerror;
}

<<<<<<< HEAD
static int dhdsdio_mem_dump(dhd_bus_t *bus)
=======
static int brcmf_sdbrcm_mem_dump(struct brcmf_bus *bus)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
{
	int ret = 0;
	int size;		/* Full mem size */
	int start = 0;		/* Start address */
	int read_size = 0;	/* Read size of each iteration */
	u8 *buf = NULL, *databuf = NULL;

	/* Get full mem size */
	size = bus->ramsize;
	buf = kmalloc(size, GFP_ATOMIC);
	if (!buf) {
<<<<<<< HEAD
		DHD_ERROR(("%s: Out of memory (%d bytes)\n", __func__, size));
=======
		BRCMF_ERROR(("%s: Out of memory (%d bytes)\n", __func__, size));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		return -1;
	}

	/* Read mem content */
	printk(KERN_DEBUG "Dump dongle memory");
	databuf = buf;
	while (size) {
		read_size = min(MEMBLOCK, size);
<<<<<<< HEAD
		ret = dhdsdio_membytes(bus, false, start, databuf, read_size);
		if (ret) {
			DHD_ERROR(("%s: Error membytes %d\n", __func__, ret));
=======
		ret = brcmf_sdbrcm_membytes(bus, false, start, databuf,
					  read_size);
		if (ret) {
			BRCMF_ERROR(("%s: Error membytes %d\n", __func__, ret));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			kfree(buf);
			return -1;
		}
		printk(".");

		/* Decrement size and increment start address */
		size -= read_size;
		start += read_size;
		databuf += read_size;
	}
	printk(KERN_DEBUG "Done\n");

	/* free buf before return !!! */
<<<<<<< HEAD
	if (write_to_file(bus->dhd, buf, bus->ramsize)) {
		DHD_ERROR(("%s: Error writing to files\n", __func__));
		return -1;
	}

	/* buf free handled in write_to_file, not here */
=======
	if (brcmf_write_to_file(bus->drvr, buf, bus->ramsize)) {
		BRCMF_ERROR(("%s: Error writing to files\n", __func__));
		return -1;
	}

	/* buf free handled in brcmf_write_to_file, not here */
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	return 0;
}

#define CONSOLE_LINE_MAX	192

<<<<<<< HEAD
static int dhdsdio_readconsole(dhd_bus_t *bus)
{
	dhd_console_t *c = &bus->console;
=======
static int brcmf_sdbrcm_readconsole(struct brcmf_bus *bus)
{
	struct brcmf_console *c = &bus->console;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	u8 line[CONSOLE_LINE_MAX], ch;
	u32 n, idx, addr;
	int rv;

	/* Don't do anything until FWREADY updates console address */
	if (bus->console_addr == 0)
		return 0;

	/* Read console log struct */
<<<<<<< HEAD
	addr = bus->console_addr + offsetof(hndrte_cons_t, log);
	rv = dhdsdio_membytes(bus, false, addr, (u8 *)&c->log,
=======
	addr = bus->console_addr + offsetof(struct rte_console, log);
	rv = brcmf_sdbrcm_membytes(bus, false, addr, (u8 *)&c->log,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				sizeof(c->log));
	if (rv < 0)
		return rv;

	/* Allocate console buffer (one time only) */
	if (c->buf == NULL) {
		c->bufsize = le32_to_cpu(c->log.buf_size);
		c->buf = kmalloc(c->bufsize, GFP_ATOMIC);
		if (c->buf == NULL)
			return -ENOMEM;
	}

	idx = le32_to_cpu(c->log.idx);

	/* Protect against corrupt value */
	if (idx > c->bufsize)
		return -EBADE;

	/* Skip reading the console buffer if the index pointer
	 has not moved */
	if (idx == c->last)
		return 0;

	/* Read the console buffer */
	addr = le32_to_cpu(c->log.buf);
<<<<<<< HEAD
	rv = dhdsdio_membytes(bus, false, addr, c->buf, c->bufsize);
=======
	rv = brcmf_sdbrcm_membytes(bus, false, addr, c->buf, c->bufsize);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	if (rv < 0)
		return rv;

	while (c->last != idx) {
		for (n = 0; n < CONSOLE_LINE_MAX - 2; n++) {
			if (c->last == idx) {
				/* This would output a partial line.
				 * Instead, back up
				 * the buffer pointer and output this
				 * line next time around.
				 */
				if (c->last >= n)
					c->last -= n;
				else
					c->last = c->bufsize - n;
				goto break2;
			}
			ch = c->buf[c->last];
			c->last = (c->last + 1) % c->bufsize;
			if (ch == '\n')
				break;
			line[n] = ch;
		}

		if (n > 0) {
			if (line[n - 1] == '\r')
				n--;
			line[n] = 0;
			printk(KERN_DEBUG "CONSOLE: %s\n", line);
		}
	}
break2:

	return 0;
}
<<<<<<< HEAD
#endif				/* DHD_DEBUG */

int dhdsdio_downloadvars(dhd_bus_t *bus, void *arg, int len)
{
	int bcmerror = 0;

	DHD_TRACE(("%s: Enter\n", __func__));

	/* Basic sanity checks */
	if (bus->dhd->up) {
=======
#endif				/* BCMDBG */

int brcmf_sdbrcm_downloadvars(struct brcmf_bus *bus, void *arg, int len)
{
	int bcmerror = 0;

	BRCMF_TRACE(("%s: Enter\n", __func__));

	/* Basic sanity checks */
	if (bus->drvr->up) {
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		bcmerror = -EISCONN;
		goto err;
	}
	if (!len) {
		bcmerror = -EOVERFLOW;
		goto err;
	}

	/* Free the old ones and replace with passed variables */
	kfree(bus->vars);

	bus->vars = kmalloc(len, GFP_ATOMIC);
	bus->varsz = bus->vars ? len : 0;
	if (bus->vars == NULL) {
		bcmerror = -ENOMEM;
		goto err;
	}

	/* Copy the passed variables, which should include the
		 terminating double-null */
	memcpy(bus->vars, arg, bus->varsz);
err:
	return bcmerror;
}

static int
<<<<<<< HEAD
dhdsdio_doiovar(dhd_bus_t *bus, const bcm_iovar_t *vi, u32 actionid,
=======
brcmf_sdbrcm_doiovar(struct brcmf_bus *bus, const struct brcmu_iovar *vi, u32 actionid,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		const char *name, void *params, int plen, void *arg, int len,
		int val_size)
{
	int bcmerror = 0;
	s32 int_val = 0;
	bool bool_val = 0;

<<<<<<< HEAD
	DHD_TRACE(("%s: Enter, action %d name %s params %p plen %d arg %p "
		"len %d val_size %d\n",
		__func__, actionid, name, params, plen, arg, len, val_size));

	bcmerror = bcm_iovar_lencheck(vi, arg, len, IOV_ISSET(actionid));
=======
	BRCMF_TRACE(("%s: Enter, action %d name %s params %p plen %d arg %p "
		     "len %d val_size %d\n", __func__, actionid, name, params,
		     plen, arg, len, val_size));

	bcmerror = brcmu_iovar_lencheck(vi, arg, len, IOV_ISSET(actionid));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	if (bcmerror != 0)
		goto exit;

	if (plen >= (int)sizeof(int_val))
		memcpy(&int_val, params, sizeof(int_val));

	bool_val = (int_val != 0) ? true : false;

	/* Some ioctls use the bus */
<<<<<<< HEAD
	dhd_os_sdlock(bus->dhd);

	/* Check if dongle is in reset. If so, only allow DEVRESET iovars */
	if (bus->dhd->dongle_reset && !(actionid == IOV_SVAL(IOV_DEVRESET) ||
=======
	brcmf_sdbrcm_sdlock(bus);

	/* Check if dongle is in reset. If so, only allow DEVRESET iovars */
	if (bus->drvr->dongle_reset && !(actionid == IOV_SVAL(IOV_DEVRESET) ||
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
					actionid == IOV_GVAL(IOV_DEVRESET))) {
		bcmerror = -EPERM;
		goto exit;
	}

	/* Handle sleep stuff before any clock mucking */
	if (vi->varid == IOV_SLEEP) {
		if (IOV_ISSET(actionid)) {
<<<<<<< HEAD
			bcmerror = dhdsdio_bussleep(bus, bool_val);
=======
			bcmerror = brcmf_sdbrcm_bussleep(bus, bool_val);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		} else {
			int_val = (s32) bus->sleeping;
			memcpy(arg, &int_val, val_size);
		}
		goto exit;
	}

	/* Request clock to allow SDIO accesses */
<<<<<<< HEAD
	if (!bus->dhd->dongle_reset) {
		BUS_WAKE(bus);
		dhdsdio_clkctl(bus, CLK_AVAIL, false);
=======
	if (!bus->drvr->dongle_reset) {
		BUS_WAKE(bus);
		brcmf_sdbrcm_clkctl(bus, CLK_AVAIL, false);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	}

	switch (actionid) {
	case IOV_GVAL(IOV_INTR):
		int_val = (s32) bus->intr;
		memcpy(arg, &int_val, val_size);
		break;

	case IOV_SVAL(IOV_INTR):
		bus->intr = bool_val;
		bus->intdis = false;
<<<<<<< HEAD
		if (bus->dhd->up) {
			if (bus->intr) {
				DHD_INTR(("%s: enable SDIO device interrupts\n",
					  __func__));
				bcmsdh_intr_enable(bus->sdh);
			} else {
				DHD_INTR(("%s: disable SDIO interrupts\n",
					  __func__));
				bcmsdh_intr_disable(bus->sdh);
=======
		if (bus->drvr->up) {
			BRCMF_INTR(("%s: %s SDIO interrupts\n", __func__,
				    bus->intr ? "enable" : "disable"));
			if (bus->intr) {
				brcmf_sdcard_intr_enable(bus->card);
			} else {
				brcmf_sdcard_intr_disable(bus->card);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			}
		}
		break;

	case IOV_GVAL(IOV_POLLRATE):
		int_val = (s32) bus->pollrate;
		memcpy(arg, &int_val, val_size);
		break;

	case IOV_SVAL(IOV_POLLRATE):
		bus->pollrate = (uint) int_val;
		bus->poll = (bus->pollrate != 0);
		break;

	case IOV_GVAL(IOV_IDLETIME):
		int_val = bus->idletime;
		memcpy(arg, &int_val, val_size);
		break;

	case IOV_SVAL(IOV_IDLETIME):
<<<<<<< HEAD
		if ((int_val < 0) && (int_val != DHD_IDLE_IMMEDIATE))
=======
		if ((int_val < 0) && (int_val != BRCMF_IDLE_IMMEDIATE))
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			bcmerror = -EINVAL;
		else
			bus->idletime = int_val;
		break;

	case IOV_GVAL(IOV_IDLECLOCK):
		int_val = (s32) bus->idleclock;
		memcpy(arg, &int_val, val_size);
		break;

	case IOV_SVAL(IOV_IDLECLOCK):
		bus->idleclock = int_val;
		break;

	case IOV_GVAL(IOV_SD1IDLE):
		int_val = (s32) sd1idle;
		memcpy(arg, &int_val, val_size);
		break;

	case IOV_SVAL(IOV_SD1IDLE):
		sd1idle = bool_val;
		break;

	case IOV_SVAL(IOV_MEMBYTES):
	case IOV_GVAL(IOV_MEMBYTES):
		{
			u32 address;
			uint size, dsize;
			u8 *data;

			bool set = (actionid == IOV_SVAL(IOV_MEMBYTES));

<<<<<<< HEAD
			ASSERT(plen >= 2 * sizeof(int));

=======
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			address = (u32) int_val;
			memcpy(&int_val, (char *)params + sizeof(int_val),
			       sizeof(int_val));
			size = (uint) int_val;

			/* Do some validation */
			dsize = set ? plen - (2 * sizeof(int)) : len;
			if (dsize < size) {
<<<<<<< HEAD
				DHD_ERROR(("%s: error on %s membytes, addr "
				"0x%08x size %d dsize %d\n",
				__func__, (set ? "set" : "get"),
				address, size, dsize));
=======
				BRCMF_ERROR(("%s: error on %s membytes, addr "
					     "0x%08x size %d dsize %d\n",
					     __func__, (set ? "set" : "get"),
					     address, size, dsize));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				bcmerror = -EINVAL;
				break;
			}

<<<<<<< HEAD
			DHD_INFO(("%s: Request to %s %d bytes at address "
			"0x%08x\n",
			__func__, (set ? "write" : "read"), size, address));
=======
			BRCMF_INFO(("%s: Request to %s %d bytes at address "
				    "0x%08x\n", __func__,
				    (set ? "write" : "read"), size, address));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

			/* If we know about SOCRAM, check for a fit */
			if ((bus->orig_ramsize) &&
			    ((address > bus->orig_ramsize)
			     || (address + size > bus->orig_ramsize))) {
<<<<<<< HEAD
				DHD_ERROR(("%s: ramsize 0x%08x doesn't have %d "
				"bytes at 0x%08x\n",
				__func__, bus->orig_ramsize, size, address));
=======
				BRCMF_ERROR(("%s: ramsize 0x%08x doesn't have"
					     " %d bytes at 0x%08x\n", __func__,
					     bus->orig_ramsize, size, address));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				bcmerror = -EINVAL;
				break;
			}

			/* Generate the actual data pointer */
			data =
			    set ? (u8 *) params +
			    2 * sizeof(int) : (u8 *) arg;

			/* Call to do the transfer */
<<<<<<< HEAD
			bcmerror =
			    dhdsdio_membytes(bus, set, address, data, size);
=======
			bcmerror = brcmf_sdbrcm_membytes(bus, set, address,
							 data, size);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

			break;
		}

	case IOV_GVAL(IOV_MEMSIZE):
		int_val = (s32) bus->ramsize;
		memcpy(arg, &int_val, val_size);
		break;

	case IOV_GVAL(IOV_SDIOD_DRIVE):
<<<<<<< HEAD
		int_val = (s32) dhd_sdiod_drive_strength;
=======
		int_val = (s32) brcmf_sdiod_drive_strength;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		memcpy(arg, &int_val, val_size);
		break;

	case IOV_SVAL(IOV_SDIOD_DRIVE):
<<<<<<< HEAD
		dhd_sdiod_drive_strength = int_val;
		dhdsdio_sdiod_drive_strength_init(bus,
					     dhd_sdiod_drive_strength);
		break;

	case IOV_SVAL(IOV_DOWNLOAD):
		bcmerror = dhdsdio_download_state(bus, bool_val);
		break;

	case IOV_SVAL(IOV_VARS):
		bcmerror = dhdsdio_downloadvars(bus, arg, len);
		break;

	case IOV_GVAL(IOV_READAHEAD):
		int_val = (s32) dhd_readahead;
=======
		brcmf_sdiod_drive_strength = int_val;
		brcmf_sdbrcm_sdiod_drive_strength_init(bus,
					     brcmf_sdiod_drive_strength);
		break;

	case IOV_SVAL(IOV_DOWNLOAD):
		bcmerror = brcmf_sdbrcm_download_state(bus, bool_val);
		break;

	case IOV_SVAL(IOV_VARS):
		bcmerror = brcmf_sdbrcm_downloadvars(bus, arg, len);
		break;

	case IOV_GVAL(IOV_READAHEAD):
		int_val = (s32) brcmf_readahead;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		memcpy(arg, &int_val, val_size);
		break;

	case IOV_SVAL(IOV_READAHEAD):
<<<<<<< HEAD
		if (bool_val && !dhd_readahead)
			bus->nextlen = 0;
		dhd_readahead = bool_val;
=======
		if (bool_val && !brcmf_readahead)
			bus->nextlen = 0;
		brcmf_readahead = bool_val;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		break;

	case IOV_GVAL(IOV_SDRXCHAIN):
		int_val = (s32) bus->use_rxchain;
		memcpy(arg, &int_val, val_size);
		break;

	case IOV_SVAL(IOV_SDRXCHAIN):
		if (bool_val && !bus->sd_rxchain)
			bcmerror = -ENOTSUPP;
		else
			bus->use_rxchain = bool_val;
		break;
	case IOV_GVAL(IOV_ALIGNCTL):
<<<<<<< HEAD
		int_val = (s32) dhd_alignctl;
=======
		int_val = (s32) brcmf_alignctl;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		memcpy(arg, &int_val, val_size);
		break;

	case IOV_SVAL(IOV_ALIGNCTL):
<<<<<<< HEAD
		dhd_alignctl = bool_val;
		break;

	case IOV_GVAL(IOV_SDALIGN):
		int_val = DHD_SDALIGN;
		memcpy(arg, &int_val, val_size);
		break;

#ifdef DHD_DEBUG
=======
		brcmf_alignctl = bool_val;
		break;

	case IOV_GVAL(IOV_SDALIGN):
		int_val = BRCMF_SDALIGN;
		memcpy(arg, &int_val, val_size);
		break;

#ifdef BCMDBG
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	case IOV_GVAL(IOV_VARS):
		if (bus->varsz < (uint) len)
			memcpy(arg, bus->vars, bus->varsz);
		else
			bcmerror = -EOVERFLOW;
		break;
<<<<<<< HEAD
#endif				/* DHD_DEBUG */

#ifdef DHD_DEBUG
	case IOV_GVAL(IOV_SDREG):
		{
			sdreg_t *sd_ptr;
			u32 addr, size;

			sd_ptr = (sdreg_t *) params;

			addr = (unsigned long)bus->regs + sd_ptr->offset;
			size = sd_ptr->func;
			int_val = (s32) bcmsdh_reg_read(bus->sdh, addr, size);
			if (bcmsdh_regfail(bus->sdh))
=======
#endif				/* BCMDBG */

#ifdef BCMDBG
	case IOV_GVAL(IOV_DCONSOLE_POLL):
		int_val = (s32) brcmf_console_ms;
		memcpy(arg, &int_val, val_size);
		break;

	case IOV_SVAL(IOV_DCONSOLE_POLL):
		brcmf_console_ms = (uint) int_val;
		break;

	case IOV_SVAL(IOV_CONS):
		if (len > 0)
			bcmerror = brcmf_sdbrcm_bus_console_in(bus->drvr,
							       arg, len - 1);
		break;

	case IOV_GVAL(IOV_SDREG):
		{
			struct brcmf_sdreg *sd_ptr;
			u32 addr, size;

			sd_ptr = (struct brcmf_sdreg *) params;

			addr = bus->ci->buscorebase + sd_ptr->offset;
			size = sd_ptr->func;
			int_val = (s32) brcmf_sdcard_reg_read(bus->card, addr,
							      size);
			if (brcmf_sdcard_regfail(bus->card))
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				bcmerror = -EIO;
			memcpy(arg, &int_val, sizeof(s32));
			break;
		}

	case IOV_SVAL(IOV_SDREG):
		{
<<<<<<< HEAD
			sdreg_t *sd_ptr;
			u32 addr, size;

			sd_ptr = (sdreg_t *) params;

			addr = (unsigned long)bus->regs + sd_ptr->offset;
			size = sd_ptr->func;
			bcmsdh_reg_write(bus->sdh, addr, size, sd_ptr->value);
			if (bcmsdh_regfail(bus->sdh))
=======
			struct brcmf_sdreg *sd_ptr;
			u32 addr, size;

			sd_ptr = (struct brcmf_sdreg *) params;

			addr = bus->ci->buscorebase + sd_ptr->offset;
			size = sd_ptr->func;
			brcmf_sdcard_reg_write(bus->card, addr, size,
					       sd_ptr->value);
			if (brcmf_sdcard_regfail(bus->card))
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				bcmerror = -EIO;
			break;
		}

		/* Same as above, but offset is not backplane
		 (not SDIO core) */
	case IOV_GVAL(IOV_SBREG):
		{
<<<<<<< HEAD
			sdreg_t sdreg;
=======
			struct brcmf_sdreg sdreg;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			u32 addr, size;

			memcpy(&sdreg, params, sizeof(sdreg));

			addr = SI_ENUM_BASE + sdreg.offset;
			size = sdreg.func;
<<<<<<< HEAD
			int_val = (s32) bcmsdh_reg_read(bus->sdh, addr, size);
			if (bcmsdh_regfail(bus->sdh))
=======
			int_val = (s32) brcmf_sdcard_reg_read(bus->card, addr,
							      size);
			if (brcmf_sdcard_regfail(bus->card))
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				bcmerror = -EIO;
			memcpy(arg, &int_val, sizeof(s32));
			break;
		}

	case IOV_SVAL(IOV_SBREG):
		{
<<<<<<< HEAD
			sdreg_t sdreg;
=======
			struct brcmf_sdreg sdreg;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			u32 addr, size;

			memcpy(&sdreg, params, sizeof(sdreg));

			addr = SI_ENUM_BASE + sdreg.offset;
			size = sdreg.func;
<<<<<<< HEAD
			bcmsdh_reg_write(bus->sdh, addr, size, sdreg.value);
			if (bcmsdh_regfail(bus->sdh))
=======
			brcmf_sdcard_reg_write(bus->card, addr, size,
					       sdreg.value);
			if (brcmf_sdcard_regfail(bus->card))
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				bcmerror = -EIO;
			break;
		}

	case IOV_GVAL(IOV_SDCIS):
		{
			*(char *)arg = 0;

			strcat(arg, "\nFunc 0\n");
<<<<<<< HEAD
			bcmsdh_cis_read(bus->sdh, 0x10,
					(u8 *) arg + strlen(arg),
					SBSDIO_CIS_SIZE_LIMIT);
			strcat(arg, "\nFunc 1\n");
			bcmsdh_cis_read(bus->sdh, 0x11,
					(u8 *) arg + strlen(arg),
					SBSDIO_CIS_SIZE_LIMIT);
			strcat(arg, "\nFunc 2\n");
			bcmsdh_cis_read(bus->sdh, 0x12,
=======
			brcmf_sdcard_cis_read(bus->card, 0x10,
					(u8 *) arg + strlen(arg),
					SBSDIO_CIS_SIZE_LIMIT);
			strcat(arg, "\nFunc 1\n");
			brcmf_sdcard_cis_read(bus->card, 0x11,
					(u8 *) arg + strlen(arg),
					SBSDIO_CIS_SIZE_LIMIT);
			strcat(arg, "\nFunc 2\n");
			brcmf_sdcard_cis_read(bus->card, 0x12,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
					(u8 *) arg + strlen(arg),
					SBSDIO_CIS_SIZE_LIMIT);
			break;
		}

	case IOV_GVAL(IOV_FORCEEVEN):
		int_val = (s32) forcealign;
		memcpy(arg, &int_val, val_size);
		break;

	case IOV_SVAL(IOV_FORCEEVEN):
		forcealign = bool_val;
		break;

	case IOV_GVAL(IOV_TXBOUND):
<<<<<<< HEAD
		int_val = (s32) dhd_txbound;
=======
		int_val = (s32) brcmf_txbound;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		memcpy(arg, &int_val, val_size);
		break;

	case IOV_SVAL(IOV_TXBOUND):
<<<<<<< HEAD
		dhd_txbound = (uint) int_val;
		break;

	case IOV_GVAL(IOV_RXBOUND):
		int_val = (s32) dhd_rxbound;
=======
		brcmf_txbound = (uint) int_val;
		break;

	case IOV_GVAL(IOV_RXBOUND):
		int_val = (s32) brcmf_rxbound;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		memcpy(arg, &int_val, val_size);
		break;

	case IOV_SVAL(IOV_RXBOUND):
<<<<<<< HEAD
		dhd_rxbound = (uint) int_val;
		break;

	case IOV_GVAL(IOV_TXMINMAX):
		int_val = (s32) dhd_txminmax;
=======
		brcmf_rxbound = (uint) int_val;
		break;

	case IOV_GVAL(IOV_TXMINMAX):
		int_val = (s32) brcmf_txminmax;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		memcpy(arg, &int_val, val_size);
		break;

	case IOV_SVAL(IOV_TXMINMAX):
<<<<<<< HEAD
		dhd_txminmax = (uint) int_val;
		break;
#endif				/* DHD_DEBUG */
=======
		brcmf_txminmax = (uint) int_val;
		break;
#endif				/* BCMDBG */
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

#ifdef SDTEST
	case IOV_GVAL(IOV_EXTLOOP):
		int_val = (s32) bus->ext_loop;
		memcpy(arg, &int_val, val_size);
		break;

	case IOV_SVAL(IOV_EXTLOOP):
		bus->ext_loop = bool_val;
		break;

	case IOV_GVAL(IOV_PKTGEN):
<<<<<<< HEAD
		bcmerror = dhdsdio_pktgen_get(bus, arg);
		break;

	case IOV_SVAL(IOV_PKTGEN):
		bcmerror = dhdsdio_pktgen_set(bus, arg);
=======
		bcmerror = brcmf_sdbrcm_pktgen_get(bus, arg);
		break;

	case IOV_SVAL(IOV_PKTGEN):
		bcmerror = brcmf_sdbrcm_pktgen_set(bus, arg);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		break;
#endif				/* SDTEST */

	case IOV_SVAL(IOV_DEVRESET):
<<<<<<< HEAD
		DHD_TRACE(("%s: Called set IOV_DEVRESET=%d dongle_reset=%d "
			"busstate=%d\n",
			__func__, bool_val, bus->dhd->dongle_reset,
			bus->dhd->busstate));

		dhd_bus_devreset(bus->dhd, (u8) bool_val);
=======
		BRCMF_TRACE(("%s: Called set IOV_DEVRESET=%d dongle_reset=%d "
			     "busstate=%d\n",
			     __func__, bool_val, bus->drvr->dongle_reset,
			     bus->drvr->busstate));

		brcmf_bus_devreset(bus->drvr, (u8) bool_val);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

		break;

	case IOV_GVAL(IOV_DEVRESET):
<<<<<<< HEAD
		DHD_TRACE(("%s: Called get IOV_DEVRESET\n", __func__));

		/* Get its status */
		int_val = (bool) bus->dhd->dongle_reset;
		memcpy(arg, &int_val, val_size);

=======
		BRCMF_TRACE(("%s: Called get IOV_DEVRESET\n", __func__));

		/* Get its status */
		int_val = (bool) bus->drvr->dongle_reset;
		memcpy(arg, &int_val, val_size);

		break;

	case IOV_GVAL(IOV_WDTICK):
		int_val = (s32) brcmf_watchdog_ms;
		memcpy(arg, &int_val, val_size);
		break;

	case IOV_SVAL(IOV_WDTICK):
		if (!bus->drvr->up) {
			bcmerror = -ENOLINK;
			break;
		}
		brcmf_sdbrcm_wd_timer(bus, (uint) int_val);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		break;

	default:
		bcmerror = -ENOTSUPP;
		break;
	}

exit:
<<<<<<< HEAD
	if ((bus->idletime == DHD_IDLE_IMMEDIATE) && !bus->dpc_sched) {
		bus->activity = false;
		dhdsdio_clkctl(bus, CLK_NONE, true);
	}

	dhd_os_sdunlock(bus->dhd);

	if (actionid == IOV_SVAL(IOV_DEVRESET) && bool_val == false)
		dhd_preinit_ioctls((dhd_pub_t *) bus->dhd);
=======
	if ((bus->idletime == BRCMF_IDLE_IMMEDIATE) && !bus->dpc_sched) {
		bus->activity = false;
		brcmf_sdbrcm_clkctl(bus, CLK_NONE, true);
	}

	brcmf_sdbrcm_sdunlock(bus);

	if (actionid == IOV_SVAL(IOV_DEVRESET) && bool_val == false)
		brcmf_c_preinit_ioctls(bus->drvr);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	return bcmerror;
}

<<<<<<< HEAD
static int dhdsdio_write_vars(dhd_bus_t *bus)
=======
static int brcmf_sdbrcm_write_vars(struct brcmf_bus *bus)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
{
	int bcmerror = 0;
	u32 varsize;
	u32 varaddr;
	u8 *vbuffer;
	u32 varsizew;
<<<<<<< HEAD
#ifdef DHD_DEBUG
	char *nvram_ularray;
#endif				/* DHD_DEBUG */
=======
#ifdef BCMDBG
	char *nvram_ularray;
#endif				/* BCMDBG */
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	/* Even if there are no vars are to be written, we still
		 need to set the ramsize. */
	varsize = bus->varsz ? roundup(bus->varsz, 4) : 0;
	varaddr = (bus->ramsize - 4) - varsize;

	if (bus->vars) {
		vbuffer = kzalloc(varsize, GFP_ATOMIC);
		if (!vbuffer)
			return -ENOMEM;

		memcpy(vbuffer, bus->vars, bus->varsz);

		/* Write the vars list */
		bcmerror =
<<<<<<< HEAD
		    dhdsdio_membytes(bus, true, varaddr, vbuffer, varsize);
#ifdef DHD_DEBUG
		/* Verify NVRAM bytes */
		DHD_INFO(("Compare NVRAM dl & ul; varsize=%d\n", varsize));
=======
		    brcmf_sdbrcm_membytes(bus, true, varaddr, vbuffer, varsize);
#ifdef BCMDBG
		/* Verify NVRAM bytes */
		BRCMF_INFO(("Compare NVRAM dl & ul; varsize=%d\n", varsize));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		nvram_ularray = kmalloc(varsize, GFP_ATOMIC);
		if (!nvram_ularray)
			return -ENOMEM;

		/* Upload image to verify downloaded contents. */
		memset(nvram_ularray, 0xaa, varsize);

		/* Read the vars list to temp buffer for comparison */
		bcmerror =
<<<<<<< HEAD
		    dhdsdio_membytes(bus, false, varaddr, nvram_ularray,
				     varsize);
		if (bcmerror) {
			DHD_ERROR(("%s: error %d on reading %d nvram bytes at "
			"0x%08x\n", __func__, bcmerror, varsize, varaddr));
		}
		/* Compare the org NVRAM with the one read from RAM */
		if (memcmp(vbuffer, nvram_ularray, varsize)) {
			DHD_ERROR(("%s: Downloaded NVRAM image is corrupted.\n",
				   __func__));
		} else
			DHD_ERROR(("%s: Download/Upload/Compare of NVRAM ok.\n",
				__func__));

		kfree(nvram_ularray);
#endif				/* DHD_DEBUG */
=======
		    brcmf_sdbrcm_membytes(bus, false, varaddr, nvram_ularray,
				     varsize);
		if (bcmerror) {
			BRCMF_ERROR(("%s: error %d on reading %d nvram bytes"
				     " at 0x%08x\n", __func__, bcmerror,
				     varsize, varaddr));
		}
		/* Compare the org NVRAM with the one read from RAM */
		if (memcmp(vbuffer, nvram_ularray, varsize)) {
			BRCMF_ERROR(("%s: Downloaded NVRAM image is "
				     "corrupted.\n", __func__));
		} else
			BRCMF_ERROR(("%s: Download/Upload/Compare of"
				     " NVRAM ok.\n", __func__));

		kfree(nvram_ularray);
#endif				/* BCMDBG */
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

		kfree(vbuffer);
	}

	/* adjust to the user specified RAM */
<<<<<<< HEAD
	DHD_INFO(("Physical memory size: %d, usable memory size: %d\n",
		  bus->orig_ramsize, bus->ramsize));
	DHD_INFO(("Vars are at %d, orig varsize is %d\n", varaddr, varsize));
=======
	BRCMF_INFO(("Physical memory size: %d, usable memory size: %d\n",
		    bus->orig_ramsize, bus->ramsize));
	BRCMF_INFO(("Vars are at %d, orig varsize is %d\n", varaddr, varsize));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	varsize = ((bus->orig_ramsize - 4) - varaddr);

	/*
	 * Determine the length token:
	 * Varsize, converted to words, in lower 16-bits, checksum
	 * in upper 16-bits.
	 */
	if (bcmerror) {
		varsizew = 0;
	} else {
		varsizew = varsize / 4;
		varsizew = (~varsizew << 16) | (varsizew & 0x0000FFFF);
		varsizew = cpu_to_le32(varsizew);
	}

<<<<<<< HEAD
	DHD_INFO(("New varsize is %d, length token=0x%08x\n", varsize,
		  varsizew));

	/* Write the length token to the last word */
	bcmerror = dhdsdio_membytes(bus, true, (bus->orig_ramsize - 4),
=======
	BRCMF_INFO(("New varsize is %d, length token=0x%08x\n", varsize,
		    varsizew));

	/* Write the length token to the last word */
	bcmerror = brcmf_sdbrcm_membytes(bus, true, (bus->orig_ramsize - 4),
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				    (u8 *)&varsizew, 4);

	return bcmerror;
}

<<<<<<< HEAD
static int dhdsdio_download_state(dhd_bus_t *bus, bool enter)
=======
static int brcmf_sdbrcm_download_state(struct brcmf_bus *bus, bool enter)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
{
	uint retries;
	u32 regdata;
	int bcmerror = 0;

	/* To enter download state, disable ARM and reset SOCRAM.
	 * To exit download state, simply reset ARM (default is RAM boot).
	 */
	if (enter) {
		bus->alp_only = true;

<<<<<<< HEAD
		dhdsdio_chip_disablecore(bus->sdh, bus->ci->armcorebase);

		dhdsdio_chip_resetcore(bus->sdh, bus->ci->ramcorebase);
=======
		brcmf_sdbrcm_chip_disablecore(bus->card, bus->ci->armcorebase);

		brcmf_sdbrcm_chip_resetcore(bus->card, bus->ci->ramcorebase);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

		/* Clear the top bit of memory */
		if (bus->ramsize) {
			u32 zeros = 0;
<<<<<<< HEAD
			dhdsdio_membytes(bus, true, bus->ramsize - 4,
					 (u8 *)&zeros, 4);
		}
	} else {
		regdata = bcmsdh_reg_read(bus->sdh,
=======
			brcmf_sdbrcm_membytes(bus, true, bus->ramsize - 4,
					 (u8 *)&zeros, 4);
		}
	} else {
		regdata = brcmf_sdcard_reg_read(bus->card,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			CORE_SB(bus->ci->ramcorebase, sbtmstatelow), 4);
		regdata &= (SBTML_RESET | SBTML_REJ_MASK |
			(SICF_CLOCK_EN << SBTML_SICF_SHIFT));
		if ((SICF_CLOCK_EN << SBTML_SICF_SHIFT) != regdata) {
<<<<<<< HEAD
			DHD_ERROR(("%s: SOCRAM core is down after reset?\n",
				   __func__));
=======
			BRCMF_ERROR(("%s: SOCRAM core is down after reset?\n",
				     __func__));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			bcmerror = -EBADE;
			goto fail;
		}

<<<<<<< HEAD
		bcmerror = dhdsdio_write_vars(bus);
		if (bcmerror) {
			DHD_ERROR(("%s: no vars written to RAM\n", __func__));
			bcmerror = 0;
		}

		W_SDREG(0xFFFFFFFF, &bus->regs->intstatus, retries);

		dhdsdio_chip_resetcore(bus->sdh, bus->ci->armcorebase);
=======
		bcmerror = brcmf_sdbrcm_write_vars(bus);
		if (bcmerror) {
			BRCMF_ERROR(("%s: no vars written to RAM\n", __func__));
			bcmerror = 0;
		}

		w_sdreg32(bus, 0xFFFFFFFF,
			  offsetof(struct sdpcmd_regs, intstatus), &retries);

		brcmf_sdbrcm_chip_resetcore(bus->card, bus->ci->armcorebase);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

		/* Allow HT Clock now that the ARM is running. */
		bus->alp_only = false;

<<<<<<< HEAD
		bus->dhd->busstate = DHD_BUS_LOAD;
=======
		bus->drvr->busstate = BRCMF_BUS_LOAD;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	}
fail:
	return bcmerror;
}

int
<<<<<<< HEAD
dhd_bus_iovar_op(dhd_pub_t *dhdp, const char *name,
		 void *params, int plen, void *arg, int len, bool set)
{
	dhd_bus_t *bus = dhdp->bus;
	const bcm_iovar_t *vi = NULL;
=======
brcmf_sdbrcm_bus_iovar_op(struct brcmf_pub *drvr, const char *name,
			  void *params, int plen, void *arg, int len, bool set)
{
	struct brcmf_bus *bus = drvr->bus;
	const struct brcmu_iovar *vi = NULL;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	int bcmerror = 0;
	int val_size;
	u32 actionid;

<<<<<<< HEAD
	DHD_TRACE(("%s: Enter\n", __func__));

	ASSERT(name);
	ASSERT(len >= 0);

	/* Get MUST have return space */
	ASSERT(set || (arg && len));

	/* Set does NOT take qualifiers */
	ASSERT(!set || (!params && !plen));

	/* Look up var locally; if not found pass to host driver */
	vi = bcm_iovar_lookup(dhdsdio_iovars, name);
	if (vi == NULL) {
		dhd_os_sdlock(bus->dhd);
=======
	BRCMF_TRACE(("%s: Enter\n", __func__));

	if (name == NULL || len <= 0)
		return -EINVAL;

	/* Set does not take qualifiers */
	if (set && (params || plen))
		return -EINVAL;

	/* Get must have return space;*/
	if (!set && !(arg && len))
		return -EINVAL;

	/* Look up var locally; if not found pass to host driver */
	vi = brcmu_iovar_lookup(brcmf_sdio_iovars, name);
	if (vi == NULL) {
		brcmf_sdbrcm_sdlock(bus);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

		BUS_WAKE(bus);

		/* Turn on clock in case SD command needs backplane */
<<<<<<< HEAD
		dhdsdio_clkctl(bus, CLK_AVAIL, false);

		bcmerror =
		    bcmsdh_iovar_op(bus->sdh, name, params, plen, arg, len,
				    set);

		/* Check for bus configuration changes of interest */

		/* If it was divisor change, read the new one */
		if (set && strcmp(name, "sd_divisor") == 0) {
			if (bcmsdh_iovar_op(bus->sdh, "sd_divisor", NULL, 0,
					    &bus->sd_divisor, sizeof(s32),
					    false) != 0) {
				bus->sd_divisor = -1;
				DHD_ERROR(("%s: fail on %s get\n", __func__,
					   name));
			} else {
				DHD_INFO(("%s: noted %s update, value now %d\n",
					  __func__, name, bus->sd_divisor));
			}
		}
		/* If it was a mode change, read the new one */
		if (set && strcmp(name, "sd_mode") == 0) {
			if (bcmsdh_iovar_op(bus->sdh, "sd_mode", NULL, 0,
					    &bus->sd_mode, sizeof(s32),
					    false) != 0) {
				bus->sd_mode = -1;
				DHD_ERROR(("%s: fail on %s get\n", __func__,
					   name));
			} else {
				DHD_INFO(("%s: noted %s update, value now %d\n",
					  __func__, name, bus->sd_mode));
			}
		}
		/* Similar check for blocksize change */
		if (set && strcmp(name, "sd_blocksize") == 0) {
			s32 fnum = 2;
			if (bcmsdh_iovar_op
			    (bus->sdh, "sd_blocksize", &fnum, sizeof(s32),
			     &bus->blocksize, sizeof(s32),
			     false) != 0) {
				bus->blocksize = 0;
				DHD_ERROR(("%s: fail on %s get\n", __func__,
					   "sd_blocksize"));
			} else {
				DHD_INFO(("%s: noted %s update, value now %d\n",
					  __func__, "sd_blocksize",
					  bus->blocksize));
=======
		brcmf_sdbrcm_clkctl(bus, CLK_AVAIL, false);

		bcmerror = brcmf_sdcard_iovar_op(bus->card, name, params, plen,
						 arg, len, set);

		/* Similar check for blocksize change */
		if (set && strcmp(name, "sd_blocksize") == 0) {
			s32 fnum = 2;
			if (brcmf_sdcard_iovar_op
			    (bus->card, "sd_blocksize", &fnum, sizeof(s32),
			     &bus->blocksize, sizeof(s32),
			     false) != 0) {
				bus->blocksize = 0;
				BRCMF_ERROR(("%s: fail on %s get\n", __func__,
					     "sd_blocksize"));
			} else {
				BRCMF_INFO(("%s: noted sd_blocksize update,"
					    " value now %d\n", __func__,
					    bus->blocksize));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			}
		}
		bus->roundup = min(max_roundup, bus->blocksize);

<<<<<<< HEAD
		if ((bus->idletime == DHD_IDLE_IMMEDIATE) && !bus->dpc_sched) {
			bus->activity = false;
			dhdsdio_clkctl(bus, CLK_NONE, true);
		}

		dhd_os_sdunlock(bus->dhd);
		goto exit;
	}

	DHD_CTL(("%s: %s %s, len %d plen %d\n", __func__,
		 name, (set ? "set" : "get"), len, plen));
=======
		if (bus->idletime == BRCMF_IDLE_IMMEDIATE &&
		    !bus->dpc_sched) {
			bus->activity = false;
			brcmf_sdbrcm_clkctl(bus, CLK_NONE, true);
		}

		brcmf_sdbrcm_sdunlock(bus);
		goto exit;
	}

	BRCMF_CTL(("%s: %s %s, len %d plen %d\n", __func__,
		   name, (set ? "set" : "get"), len, plen));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	/* set up 'params' pointer in case this is a set command so that
	 * the convenience int and bool code can be common to set and get
	 */
	if (params == NULL) {
		params = arg;
		plen = len;
	}

	if (vi->type == IOVT_VOID)
		val_size = 0;
	else if (vi->type == IOVT_BUFFER)
		val_size = len;
	else
		/* all other types are integer sized */
		val_size = sizeof(int);

	actionid = set ? IOV_SVAL(vi->varid) : IOV_GVAL(vi->varid);
<<<<<<< HEAD
	bcmerror =
	    dhdsdio_doiovar(bus, vi, actionid, name, params, plen, arg, len,
			    val_size);
=======
	bcmerror = brcmf_sdbrcm_doiovar(bus, vi, actionid, name, params, plen,
					arg, len, val_size);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

exit:
	return bcmerror;
}

<<<<<<< HEAD
void dhd_bus_stop(struct dhd_bus *bus, bool enforce_mutex)
=======
void brcmf_sdbrcm_bus_stop(struct brcmf_bus *bus, bool enforce_mutex)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
{
	u32 local_hostintmask;
	u8 saveclk;
	uint retries;
	int err;

<<<<<<< HEAD
	DHD_TRACE(("%s: Enter\n", __func__));

	if (enforce_mutex)
		dhd_os_sdlock(bus->dhd);
=======
	BRCMF_TRACE(("%s: Enter\n", __func__));

	if (enforce_mutex)
		brcmf_sdbrcm_sdlock(bus);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	BUS_WAKE(bus);

	/* Enable clock for device interrupts */
<<<<<<< HEAD
	dhdsdio_clkctl(bus, CLK_AVAIL, false);

	/* Disable and clear interrupts at the chip level also */
	W_SDREG(0, &bus->regs->hostintmask, retries);
=======
	brcmf_sdbrcm_clkctl(bus, CLK_AVAIL, false);

	if (bus->watchdog_tsk) {
		send_sig(SIGTERM, bus->watchdog_tsk, 1);
		kthread_stop(bus->watchdog_tsk);
		bus->watchdog_tsk = NULL;
	}

	if (bus->dpc_tsk) {
		send_sig(SIGTERM, bus->dpc_tsk, 1);
		kthread_stop(bus->dpc_tsk);
		bus->dpc_tsk = NULL;
	} else
		tasklet_kill(&bus->tasklet);

	/* Disable and clear interrupts at the chip level also */
	w_sdreg32(bus, 0, offsetof(struct sdpcmd_regs, hostintmask), &retries);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	local_hostintmask = bus->hostintmask;
	bus->hostintmask = 0;

	/* Change our idea of bus state */
<<<<<<< HEAD
	bus->dhd->busstate = DHD_BUS_DOWN;

	/* Force clocks on backplane to be sure F2 interrupt propagates */
	saveclk =
	    bcmsdh_cfg_read(bus->sdh, SDIO_FUNC_1, SBSDIO_FUNC1_CHIPCLKCSR,
			    &err);
	if (!err) {
		bcmsdh_cfg_write(bus->sdh, SDIO_FUNC_1, SBSDIO_FUNC1_CHIPCLKCSR,
				 (saveclk | SBSDIO_FORCE_HT), &err);
	}
	if (err) {
		DHD_ERROR(("%s: Failed to force clock for F2: err %d\n",
			   __func__, err));
	}

	/* Turn off the bus (F2), free any pending packets */
	DHD_INTR(("%s: disable SDIO interrupts\n", __func__));
	bcmsdh_intr_disable(bus->sdh);
	bcmsdh_cfg_write(bus->sdh, SDIO_FUNC_0, SDIOD_CCCR_IOEN,
			 SDIO_FUNC_ENABLE_1, NULL);

	/* Clear any pending interrupts now that F2 is disabled */
	W_SDREG(local_hostintmask, &bus->regs->intstatus, retries);

	/* Turn off the backplane clock (only) */
	dhdsdio_clkctl(bus, CLK_SDONLY, false);

	/* Clear the data packet queues */
	bcm_pktq_flush(&bus->txq, true, NULL, NULL);

	/* Clear any held glomming stuff */
	if (bus->glomd)
		bcm_pkt_buf_free_skb(bus->glomd);

	if (bus->glom)
		bcm_pkt_buf_free_skb(bus->glom);
=======
	bus->drvr->busstate = BRCMF_BUS_DOWN;

	/* Force clocks on backplane to be sure F2 interrupt propagates */
	saveclk = brcmf_sdcard_cfg_read(bus->card, SDIO_FUNC_1,
					SBSDIO_FUNC1_CHIPCLKCSR, &err);
	if (!err) {
		brcmf_sdcard_cfg_write(bus->card, SDIO_FUNC_1,
				       SBSDIO_FUNC1_CHIPCLKCSR,
				       (saveclk | SBSDIO_FORCE_HT), &err);
	}
	if (err) {
		BRCMF_ERROR(("%s: Failed to force clock for F2: err %d\n",
			     __func__, err));
	}

	/* Turn off the bus (F2), free any pending packets */
	BRCMF_INTR(("%s: disable SDIO interrupts\n", __func__));
	brcmf_sdcard_intr_disable(bus->card);
	brcmf_sdcard_cfg_write(bus->card, SDIO_FUNC_0, SDIO_CCCR_IOEx,
			 SDIO_FUNC_ENABLE_1, NULL);

	/* Clear any pending interrupts now that F2 is disabled */
	w_sdreg32(bus, local_hostintmask,
		  offsetof(struct sdpcmd_regs, intstatus), &retries);

	/* Turn off the backplane clock (only) */
	brcmf_sdbrcm_clkctl(bus, CLK_SDONLY, false);

	/* Clear the data packet queues */
	brcmu_pktq_flush(&bus->txq, true, NULL, NULL);

	/* Clear any held glomming stuff */
	if (bus->glomd)
		brcmu_pkt_buf_free_skb(bus->glomd);

	if (bus->glom)
		brcmu_pkt_buf_free_skb(bus->glom);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	bus->glom = bus->glomd = NULL;

	/* Clear rx control and wake any waiters */
	bus->rxlen = 0;
<<<<<<< HEAD
	dhd_os_ioctl_resp_wake(bus->dhd);
=======
	brcmf_os_ioctl_resp_wake(bus->drvr);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	/* Reset some F2 state stuff */
	bus->rxskip = false;
	bus->tx_seq = bus->rx_seq = 0;

	if (enforce_mutex)
<<<<<<< HEAD
		dhd_os_sdunlock(bus->dhd);
}

int dhd_bus_init(dhd_pub_t *dhdp, bool enforce_mutex)
{
	dhd_bus_t *bus = dhdp->bus;
	dhd_timeout_t tmo;
=======
		brcmf_sdbrcm_sdunlock(bus);
}

int brcmf_sdbrcm_bus_init(struct brcmf_pub *drvr, bool enforce_mutex)
{
	struct brcmf_bus *bus = drvr->bus;
	struct brcmf_timeout tmo;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	uint retries = 0;
	u8 ready, enable;
	int err, ret = 0;
	u8 saveclk;

<<<<<<< HEAD
	DHD_TRACE(("%s: Enter\n", __func__));

	ASSERT(bus->dhd);
	if (!bus->dhd)
		return 0;

	if (enforce_mutex)
		dhd_os_sdlock(bus->dhd);

	/* Make sure backplane clock is on, needed to generate F2 interrupt */
	dhdsdio_clkctl(bus, CLK_AVAIL, false);
=======
	BRCMF_TRACE(("%s: Enter\n", __func__));

	/* try to download image and nvram to the dongle */
	if (drvr->busstate == BRCMF_BUS_DOWN) {
		if (!(brcmf_sdbrcm_download_firmware(bus, bus->card)))
			return -1;
	}

	if (!bus->drvr)
		return 0;

	/* Start the watchdog timer */
	bus->drvr->tickcnt = 0;
	brcmf_sdbrcm_wd_timer(bus, brcmf_watchdog_ms);

	if (enforce_mutex)
		brcmf_sdbrcm_sdlock(bus);

	/* Make sure backplane clock is on, needed to generate F2 interrupt */
	brcmf_sdbrcm_clkctl(bus, CLK_AVAIL, false);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	if (bus->clkstate != CLK_AVAIL)
		goto exit;

	/* Force clocks on backplane to be sure F2 interrupt propagates */
	saveclk =
<<<<<<< HEAD
	    bcmsdh_cfg_read(bus->sdh, SDIO_FUNC_1, SBSDIO_FUNC1_CHIPCLKCSR,
			    &err);
	if (!err) {
		bcmsdh_cfg_write(bus->sdh, SDIO_FUNC_1, SBSDIO_FUNC1_CHIPCLKCSR,
				 (saveclk | SBSDIO_FORCE_HT), &err);
	}
	if (err) {
		DHD_ERROR(("%s: Failed to force clock for F2: err %d\n",
			   __func__, err));
=======
	    brcmf_sdcard_cfg_read(bus->card, SDIO_FUNC_1,
				  SBSDIO_FUNC1_CHIPCLKCSR, &err);
	if (!err) {
		brcmf_sdcard_cfg_write(bus->card, SDIO_FUNC_1,
				       SBSDIO_FUNC1_CHIPCLKCSR,
				       (saveclk | SBSDIO_FORCE_HT), &err);
	}
	if (err) {
		BRCMF_ERROR(("%s: Failed to force clock for F2: err %d\n",
			     __func__, err));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		goto exit;
	}

	/* Enable function 2 (frame transfers) */
<<<<<<< HEAD
	W_SDREG((SDPCM_PROT_VERSION << SMB_DATA_VERSION_SHIFT),
		&bus->regs->tosbmailboxdata, retries);
	enable = (SDIO_FUNC_ENABLE_1 | SDIO_FUNC_ENABLE_2);

	bcmsdh_cfg_write(bus->sdh, SDIO_FUNC_0, SDIOD_CCCR_IOEN, enable, NULL);

	/* Give the dongle some time to do its thing and set IOR2 */
	dhd_timeout_start(&tmo, DHD_WAIT_F2RDY * 1000);

	ready = 0;
	while (ready != enable && !dhd_timeout_expired(&tmo))
		ready =
		    bcmsdh_cfg_read(bus->sdh, SDIO_FUNC_0, SDIOD_CCCR_IORDY,
				    NULL);

	DHD_INFO(("%s: enable 0x%02x, ready 0x%02x (waited %uus)\n",
		  __func__, enable, ready, tmo.elapsed));
=======
	w_sdreg32(bus, SDPCM_PROT_VERSION << SMB_DATA_VERSION_SHIFT,
		  offsetof(struct sdpcmd_regs, tosbmailboxdata), &retries);
	enable = (SDIO_FUNC_ENABLE_1 | SDIO_FUNC_ENABLE_2);

	brcmf_sdcard_cfg_write(bus->card, SDIO_FUNC_0, SDIO_CCCR_IOEx, enable,
			       NULL);

	/* Give the dongle some time to do its thing and set IOR2 */
	brcmf_timeout_start(&tmo, BRCMF_WAIT_F2RDY * 1000);

	ready = 0;
	while (ready != enable && !brcmf_timeout_expired(&tmo))
		ready = brcmf_sdcard_cfg_read(bus->card, SDIO_FUNC_0,
					      SDIO_CCCR_IORx, NULL);

	BRCMF_INFO(("%s: enable 0x%02x, ready 0x%02x (waited %uus)\n",
		    __func__, enable, ready, tmo.elapsed));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	/* If F2 successfully enabled, set core and enable interrupts */
	if (ready == enable) {
		/* Set up the interrupt mask and enable interrupts */
		bus->hostintmask = HOSTINTMASK;
<<<<<<< HEAD
		W_SDREG(bus->hostintmask,
			(unsigned int *)CORE_BUS_REG(bus->ci->buscorebase,
			hostintmask), retries);

		bcmsdh_cfg_write(bus->sdh, SDIO_FUNC_1, SBSDIO_WATERMARK,
				 (u8) watermark, &err);

		/* Set bus state according to enable result */
		dhdp->busstate = DHD_BUS_DATA;

		/* bcmsdh_intr_unmask(bus->sdh); */

		bus->intdis = false;
		if (bus->intr) {
			DHD_INTR(("%s: enable SDIO device interrupts\n",
				  __func__));
			bcmsdh_intr_enable(bus->sdh);
		} else {
			DHD_INTR(("%s: disable SDIO interrupts\n", __func__));
			bcmsdh_intr_disable(bus->sdh);
=======
		w_sdreg32(bus, bus->hostintmask,
			  offsetof(struct sdpcmd_regs, hostintmask), &retries);

		brcmf_sdcard_cfg_write(bus->card, SDIO_FUNC_1, SBSDIO_WATERMARK,
				 (u8) watermark, &err);

		/* Set bus state according to enable result */
		drvr->busstate = BRCMF_BUS_DATA;

		bus->intdis = false;
		if (bus->intr) {
			BRCMF_INTR(("%s: enable SDIO device interrupts\n",
				    __func__));
			brcmf_sdcard_intr_enable(bus->card);
		} else {
			BRCMF_INTR(("%s: disable SDIO interrupts\n", __func__));
			brcmf_sdcard_intr_disable(bus->card);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		}

	}

	else {
		/* Disable F2 again */
		enable = SDIO_FUNC_ENABLE_1;
<<<<<<< HEAD
		bcmsdh_cfg_write(bus->sdh, SDIO_FUNC_0, SDIOD_CCCR_IOEN, enable,
				 NULL);
	}

	/* Restore previous clock setting */
	bcmsdh_cfg_write(bus->sdh, SDIO_FUNC_1, SBSDIO_FUNC1_CHIPCLKCSR,
			 saveclk, &err);

	/* If we didn't come up, turn off backplane clock */
	if (dhdp->busstate != DHD_BUS_DATA)
		dhdsdio_clkctl(bus, CLK_NONE, false);

exit:
	if (enforce_mutex)
		dhd_os_sdunlock(bus->dhd);
=======
		brcmf_sdcard_cfg_write(bus->card, SDIO_FUNC_0, SDIO_CCCR_IOEx,
				       enable, NULL);
	}

	/* Restore previous clock setting */
	brcmf_sdcard_cfg_write(bus->card, SDIO_FUNC_1, SBSDIO_FUNC1_CHIPCLKCSR,
			 saveclk, &err);

#if defined(OOB_INTR_ONLY)
	/* Host registration for OOB interrupt */
	if (brcmf_sdio_register_oob_intr(bus->dhd)) {
		brcmf_sdbrcm_wd_timer(bus, 0);
		BRCMF_ERROR(("%s Host failed to resgister for OOB\n",
			     __func__));
		ret = -ENODEV;
		goto exit;
	}

	/* Enable oob at firmware */
	brcmf_sdbrcm_enable_oob_intr(bus, true);
#endif		/* defined(OOB_INTR_ONLY) */

	/* If we didn't come up, turn off backplane clock */
	if (drvr->busstate != BRCMF_BUS_DATA)
		brcmf_sdbrcm_clkctl(bus, CLK_NONE, false);

exit:
	if (enforce_mutex)
		brcmf_sdbrcm_sdunlock(bus);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	return ret;
}

<<<<<<< HEAD
static void dhdsdio_rxfail(dhd_bus_t *bus, bool abort, bool rtx)
{
	bcmsdh_info_t *sdh = bus->sdh;
	sdpcmd_regs_t *regs = bus->regs;
=======
static void brcmf_sdbrcm_rxfail(struct brcmf_bus *bus, bool abort, bool rtx)
{
	struct brcmf_sdio_card *card = bus->card;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	uint retries = 0;
	u16 lastrbc;
	u8 hi, lo;
	int err;

<<<<<<< HEAD
	DHD_ERROR(("%s: %sterminate frame%s\n", __func__,
		   (abort ? "abort command, " : ""),
		   (rtx ? ", send NAK" : "")));

	if (abort)
		bcmsdh_abort(sdh, SDIO_FUNC_2);

	bcmsdh_cfg_write(sdh, SDIO_FUNC_1, SBSDIO_FUNC1_FRAMECTRL, SFC_RF_TERM,
			 &err);
=======
	BRCMF_ERROR(("%s: %sterminate frame%s\n", __func__,
		     (abort ? "abort command, " : ""),
		     (rtx ? ", send NAK" : "")));

	if (abort)
		brcmf_sdcard_abort(card, SDIO_FUNC_2);

	brcmf_sdcard_cfg_write(card, SDIO_FUNC_1, SBSDIO_FUNC1_FRAMECTRL,
			       SFC_RF_TERM, &err);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	bus->f1regdata++;

	/* Wait until the packet has been flushed (device/FIFO stable) */
	for (lastrbc = retries = 0xffff; retries > 0; retries--) {
<<<<<<< HEAD
		hi = bcmsdh_cfg_read(sdh, SDIO_FUNC_1, SBSDIO_FUNC1_RFRAMEBCHI,
				     NULL);
		lo = bcmsdh_cfg_read(sdh, SDIO_FUNC_1, SBSDIO_FUNC1_RFRAMEBCLO,
				     NULL);
=======
		hi = brcmf_sdcard_cfg_read(card, SDIO_FUNC_1,
					   SBSDIO_FUNC1_RFRAMEBCHI, NULL);
		lo = brcmf_sdcard_cfg_read(card, SDIO_FUNC_1,
					   SBSDIO_FUNC1_RFRAMEBCLO, NULL);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		bus->f1regdata += 2;

		if ((hi == 0) && (lo == 0))
			break;

		if ((hi > (lastrbc >> 8)) && (lo > (lastrbc & 0x00ff))) {
<<<<<<< HEAD
			DHD_ERROR(("%s: count growing: last 0x%04x now "
				"0x%04x\n",
				__func__, lastrbc, ((hi << 8) + lo)));
=======
			BRCMF_ERROR(("%s: count growing: last 0x%04x now "
				     "0x%04x\n",
				     __func__, lastrbc, ((hi << 8) + lo)));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		}
		lastrbc = (hi << 8) + lo;
	}

	if (!retries) {
<<<<<<< HEAD
		DHD_ERROR(("%s: count never zeroed: last 0x%04x\n",
			   __func__, lastrbc));
	} else {
		DHD_INFO(("%s: flush took %d iterations\n", __func__,
			  (0xffff - retries)));
=======
		BRCMF_ERROR(("%s: count never zeroed: last 0x%04x\n",
			     __func__, lastrbc));
	} else {
		BRCMF_INFO(("%s: flush took %d iterations\n", __func__,
			    (0xffff - retries)));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	}

	if (rtx) {
		bus->rxrtx++;
<<<<<<< HEAD
		W_SDREG(SMB_NAK, &regs->tosbmailbox, retries);
=======
		w_sdreg32(bus, SMB_NAK,
			  offsetof(struct sdpcmd_regs, tosbmailbox), &retries);

>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		bus->f1regdata++;
		if (retries <= retry_limit)
			bus->rxskip = true;
	}

	/* Clear partial in any case */
	bus->nextlen = 0;

	/* If we can't reach the device, signal failure */
<<<<<<< HEAD
	if (err || bcmsdh_regfail(sdh))
		bus->dhd->busstate = DHD_BUS_DOWN;
}

static void
dhdsdio_read_control(dhd_bus_t *bus, u8 *hdr, uint len, uint doff)
{
	bcmsdh_info_t *sdh = bus->sdh;
=======
	if (err || brcmf_sdcard_regfail(card))
		bus->drvr->busstate = BRCMF_BUS_DOWN;
}

static void
brcmf_sdbrcm_read_control(struct brcmf_bus *bus, u8 *hdr, uint len, uint doff)
{
	struct brcmf_sdio_card *card = bus->card;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	uint rdlen, pad;

	int sdret;

<<<<<<< HEAD
	DHD_TRACE(("%s: Enter\n", __func__));
=======
	BRCMF_TRACE(("%s: Enter\n", __func__));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	/* Control data already received in aligned rxctl */
	if ((bus->bus == SPI_BUS) && (!bus->usebufpool))
		goto gotpkt;

<<<<<<< HEAD
	ASSERT(bus->rxbuf);
	/* Set rxctl for frame (w/optional alignment) */
	bus->rxctl = bus->rxbuf;
	if (dhd_alignctl) {
		bus->rxctl += firstread;
		pad = ((unsigned long)bus->rxctl % DHD_SDALIGN);
		if (pad)
			bus->rxctl += (DHD_SDALIGN - pad);
		bus->rxctl -= firstread;
	}
	ASSERT(bus->rxctl >= bus->rxbuf);
=======
	/* Set rxctl for frame (w/optional alignment) */
	bus->rxctl = bus->rxbuf;
	if (brcmf_alignctl) {
		bus->rxctl += firstread;
		pad = ((unsigned long)bus->rxctl % BRCMF_SDALIGN);
		if (pad)
			bus->rxctl += (BRCMF_SDALIGN - pad);
		bus->rxctl -= firstread;
	}
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	/* Copy the already-read portion over */
	memcpy(bus->rxctl, hdr, firstread);
	if (len <= firstread)
		goto gotpkt;

	/* Copy the full data pkt in gSPI case and process ioctl. */
	if (bus->bus == SPI_BUS) {
		memcpy(bus->rxctl, hdr, len);
		goto gotpkt;
	}

	/* Raise rdlen to next SDIO block to avoid tail command */
	rdlen = len - firstread;
	if (bus->roundup && bus->blocksize && (rdlen > bus->blocksize)) {
		pad = bus->blocksize - (rdlen % bus->blocksize);
		if ((pad <= bus->roundup) && (pad < bus->blocksize) &&
<<<<<<< HEAD
		    ((len + pad) < bus->dhd->maxctl))
			rdlen += pad;
	} else if (rdlen % DHD_SDALIGN) {
		rdlen += DHD_SDALIGN - (rdlen % DHD_SDALIGN);
=======
		    ((len + pad) < bus->drvr->maxctl))
			rdlen += pad;
	} else if (rdlen % BRCMF_SDALIGN) {
		rdlen += BRCMF_SDALIGN - (rdlen % BRCMF_SDALIGN);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	}

	/* Satisfy length-alignment requirements */
	if (forcealign && (rdlen & (ALIGNMENT - 1)))
		rdlen = roundup(rdlen, ALIGNMENT);

	/* Drop if the read is too big or it exceeds our maximum */
<<<<<<< HEAD
	if ((rdlen + firstread) > bus->dhd->maxctl) {
		DHD_ERROR(("%s: %d-byte control read exceeds %d-byte buffer\n",
			   __func__, rdlen, bus->dhd->maxctl));
		bus->dhd->rx_errors++;
		dhdsdio_rxfail(bus, false, false);
		goto done;
	}

	if ((len - doff) > bus->dhd->maxctl) {
		DHD_ERROR(("%s: %d-byte ctl frame (%d-byte ctl data) exceeds "
			"%d-byte limit\n",
			__func__, len, (len - doff), bus->dhd->maxctl));
		bus->dhd->rx_errors++;
		bus->rx_toolong++;
		dhdsdio_rxfail(bus, false, false);
=======
	if ((rdlen + firstread) > bus->drvr->maxctl) {
		BRCMF_ERROR(("%s: %d-byte control read exceeds %d-byte"
			     " buffer\n", __func__, rdlen, bus->drvr->maxctl));
		bus->drvr->rx_errors++;
		brcmf_sdbrcm_rxfail(bus, false, false);
		goto done;
	}

	if ((len - doff) > bus->drvr->maxctl) {
		BRCMF_ERROR(("%s: %d-byte ctl frame (%d-byte ctl data) exceeds "
			     "%d-byte limit\n",
			     __func__, len, (len - doff), bus->drvr->maxctl));
		bus->drvr->rx_errors++;
		bus->rx_toolong++;
		brcmf_sdbrcm_rxfail(bus, false, false);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		goto done;
	}

	/* Read remainder of frame body into the rxctl buffer */
<<<<<<< HEAD
	sdret = bcmsdh_recv_buf(bus, bcmsdh_cur_sbwad(sdh), SDIO_FUNC_2,
				F2SYNC, (bus->rxctl + firstread), rdlen,
				NULL, NULL, NULL);
	bus->f2rxdata++;
	ASSERT(sdret != -BCME_PENDING);

	/* Control frame failures need retransmission */
	if (sdret < 0) {
		DHD_ERROR(("%s: read %d control bytes failed: %d\n",
			   __func__, rdlen, sdret));
		bus->rxc_errors++;	/* dhd.rx_ctlerrs is higher level */
		dhdsdio_rxfail(bus, true, true);
=======
	sdret = brcmf_sdcard_recv_buf(card, brcmf_sdcard_cur_sbwad(card),
				SDIO_FUNC_2,
				F2SYNC, (bus->rxctl + firstread), rdlen,
				NULL, NULL, NULL);
	bus->f2rxdata++;

	/* Control frame failures need retransmission */
	if (sdret < 0) {
		BRCMF_ERROR(("%s: read %d control bytes failed: %d\n",
			     __func__, rdlen, sdret));
		bus->rxc_errors++;
		brcmf_sdbrcm_rxfail(bus, true, true);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		goto done;
	}

gotpkt:

<<<<<<< HEAD
#ifdef DHD_DEBUG
	if (DHD_BYTES_ON() && DHD_CTL_ON()) {
=======
#ifdef BCMDBG
	if (BRCMF_BYTES_ON() && BRCMF_CTL_ON()) {
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		printk(KERN_DEBUG "RxCtrl:\n");
		print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, bus->rxctl, len);
	}
#endif

	/* Point to valid data and indicate its length */
	bus->rxctl += doff;
	bus->rxlen = len - doff;

done:
	/* Awake any waiters */
<<<<<<< HEAD
	dhd_os_ioctl_resp_wake(bus->dhd);
}

static u8 dhdsdio_rxglom(dhd_bus_t *bus, u8 rxseq)
=======
	brcmf_os_ioctl_resp_wake(bus->drvr);
}

static u8 brcmf_sdbrcm_rxglom(struct brcmf_bus *bus, u8 rxseq)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
{
	u16 dlen, totlen;
	u8 *dptr, num = 0;

	u16 sublen, check;
	struct sk_buff *pfirst, *plast, *pnext, *save_pfirst;

	int errcode;
	u8 chan, seq, doff, sfdoff;
	u8 txmax;

	int ifidx = 0;
	bool usechain = bus->use_rxchain;

	/* If packets, issue read(s) and send up packet chain */
	/* Return sequence numbers consumed? */

<<<<<<< HEAD
	DHD_TRACE(("dhdsdio_rxglom: start: glomd %p glom %p\n", bus->glomd,
		   bus->glom));

	/* If there's a descriptor, generate the packet chain */
	if (bus->glomd) {
		dhd_os_sdlock_rxq(bus->dhd);

=======
	BRCMF_TRACE(("brcmf_sdbrcm_rxglom: start: glomd %p glom %p\n",
		     bus->glomd, bus->glom));

	/* If there's a descriptor, generate the packet chain */
	if (bus->glomd) {
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		pfirst = plast = pnext = NULL;
		dlen = (u16) (bus->glomd->len);
		dptr = bus->glomd->data;
		if (!dlen || (dlen & 1)) {
<<<<<<< HEAD
			DHD_ERROR(("%s: bad glomd len(%d), ignore descriptor\n",
			__func__, dlen));
=======
			BRCMF_ERROR(("%s: bad glomd len(%d),"
				     " ignore descriptor\n",
				     __func__, dlen));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			dlen = 0;
		}

		for (totlen = num = 0; dlen; num++) {
			/* Get (and move past) next length */
			sublen = get_unaligned_le16(dptr);
			dlen -= sizeof(u16);
			dptr += sizeof(u16);
			if ((sublen < SDPCM_HDRLEN) ||
			    ((num == 0) && (sublen < (2 * SDPCM_HDRLEN)))) {
<<<<<<< HEAD
				DHD_ERROR(("%s: descriptor len %d bad: %d\n",
					   __func__, num, sublen));
				pnext = NULL;
				break;
			}
			if (sublen % DHD_SDALIGN) {
				DHD_ERROR(("%s: sublen %d not multiple of %d\n",
				__func__, sublen, DHD_SDALIGN));
=======
				BRCMF_ERROR(("%s: descriptor len %d bad: %d\n",
					     __func__, num, sublen));
				pnext = NULL;
				break;
			}
			if (sublen % BRCMF_SDALIGN) {
				BRCMF_ERROR(("%s: sublen %d not multiple of"
					     " %d\n", __func__, sublen,
					     BRCMF_SDALIGN));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				usechain = false;
			}
			totlen += sublen;

			/* For last frame, adjust read len so total
				 is a block multiple */
			if (!dlen) {
				sublen +=
				    (roundup(totlen, bus->blocksize) - totlen);
				totlen = roundup(totlen, bus->blocksize);
			}

			/* Allocate/chain packet for next subframe */
<<<<<<< HEAD
			pnext = bcm_pkt_buf_get_skb(sublen + DHD_SDALIGN);
			if (pnext == NULL) {
				DHD_ERROR(("%s: bcm_pkt_buf_get_skb failed, "
					"num %d len %d\n", __func__,
					num, sublen));
				break;
			}
			ASSERT(!(pnext->prev));
			if (!pfirst) {
				ASSERT(!plast);
				pfirst = plast = pnext;
			} else {
				ASSERT(plast);
=======
			pnext = brcmu_pkt_buf_get_skb(sublen + BRCMF_SDALIGN);
			if (pnext == NULL) {
				BRCMF_ERROR(("%s: bcm_pkt_buf_get_skb failed, "
					     "num %d len %d\n", __func__,
					     num, sublen));
				break;
			}
			if (!pfirst) {
				pfirst = plast = pnext;
			} else {
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				plast->next = pnext;
				plast = pnext;
			}

			/* Adhere to start alignment requirements */
<<<<<<< HEAD
			PKTALIGN(pnext, sublen, DHD_SDALIGN);
=======
			PKTALIGN(pnext, sublen, BRCMF_SDALIGN);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		}

		/* If all allocations succeeded, save packet chain
			 in bus structure */
		if (pnext) {
<<<<<<< HEAD
			DHD_GLOM(("%s: allocated %d-byte packet chain for %d "
				"subframes\n", __func__, totlen, num));
			if (DHD_GLOM_ON() && bus->nextlen) {
				if (totlen != bus->nextlen) {
					DHD_GLOM(("%s: glomdesc mismatch: nextlen %d glomdesc %d " "rxseq %d\n",
						__func__, bus->nextlen,
						totlen, rxseq));
=======
			BRCMF_GLOM(("%s: allocated %d-byte packet chain for %d "
				    "subframes\n", __func__, totlen, num));
			if (BRCMF_GLOM_ON() && bus->nextlen) {
				if (totlen != bus->nextlen) {
					BRCMF_GLOM(("%s: glomdesc mismatch: "
						    "nextlen %d glomdesc %d "
						    "rxseq %d\n", __func__,
						    bus->nextlen,
						    totlen, rxseq));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				}
			}
			bus->glom = pfirst;
			pfirst = pnext = NULL;
		} else {
			if (pfirst)
<<<<<<< HEAD
				bcm_pkt_buf_free_skb(pfirst);
=======
				brcmu_pkt_buf_free_skb(pfirst);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			bus->glom = NULL;
			num = 0;
		}

		/* Done with descriptor packet */
<<<<<<< HEAD
		bcm_pkt_buf_free_skb(bus->glomd);
		bus->glomd = NULL;
		bus->nextlen = 0;

		dhd_os_sdunlock_rxq(bus->dhd);
=======
		brcmu_pkt_buf_free_skb(bus->glomd);
		bus->glomd = NULL;
		bus->nextlen = 0;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	}

	/* Ok -- either we just generated a packet chain,
		 or had one from before */
	if (bus->glom) {
<<<<<<< HEAD
		if (DHD_GLOM_ON()) {
			DHD_GLOM(("%s: try superframe read, packet chain:\n",
				__func__));
			for (pnext = bus->glom; pnext; pnext = pnext->next) {
				DHD_GLOM(("    %p: %p len 0x%04x (%d)\n",
					  pnext, (u8 *) (pnext->data),
					  pnext->len, pnext->len));
=======
		if (BRCMF_GLOM_ON()) {
			BRCMF_GLOM(("%s: try superframe read, packet chain:\n",
				    __func__));
			for (pnext = bus->glom; pnext; pnext = pnext->next) {
				BRCMF_GLOM(("    %p: %p len 0x%04x (%d)\n",
					    pnext, (u8 *) (pnext->data),
					    pnext->len, pnext->len));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			}
		}

		pfirst = bus->glom;
<<<<<<< HEAD
		dlen = (u16) bcm_pkttotlen(pfirst);
=======
		dlen = (u16) brcmu_pkttotlen(pfirst);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

		/* Do an SDIO read for the superframe.  Configurable iovar to
		 * read directly into the chained packet, or allocate a large
		 * packet and and copy into the chain.
		 */
		if (usechain) {
<<<<<<< HEAD
			errcode = bcmsdh_recv_buf(bus,
					bcmsdh_cur_sbwad(bus->sdh), SDIO_FUNC_2,
					F2SYNC, (u8 *) pfirst->data, dlen,
					pfirst, NULL, NULL);
		} else if (bus->dataptr) {
			errcode = bcmsdh_recv_buf(bus,
					bcmsdh_cur_sbwad(bus->sdh), SDIO_FUNC_2,
					F2SYNC, bus->dataptr, dlen,
					NULL, NULL, NULL);
			sublen = (u16) bcm_pktfrombuf(pfirst, 0, dlen,
						bus->dataptr);
			if (sublen != dlen) {
				DHD_ERROR(("%s: FAILED TO COPY, dlen %d sublen %d\n",
					__func__, dlen, sublen));
=======
			errcode = brcmf_sdcard_recv_buf(bus->card,
					brcmf_sdcard_cur_sbwad(bus->card),
					SDIO_FUNC_2,
					F2SYNC, (u8 *) pfirst->data, dlen,
					pfirst, NULL, NULL);
		} else if (bus->dataptr) {
			errcode = brcmf_sdcard_recv_buf(bus->card,
					brcmf_sdcard_cur_sbwad(bus->card),
					SDIO_FUNC_2,
					F2SYNC, bus->dataptr, dlen,
					NULL, NULL, NULL);
			sublen = (u16) brcmu_pktfrombuf(pfirst, 0, dlen,
						bus->dataptr);
			if (sublen != dlen) {
				BRCMF_ERROR(("%s: FAILED TO COPY, dlen %d "
					     "sublen %d\n",
					     __func__, dlen, sublen));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				errcode = -1;
			}
			pnext = NULL;
		} else {
<<<<<<< HEAD
			DHD_ERROR(("COULDN'T ALLOC %d-BYTE GLOM, FORCE FAILURE\n",
				dlen));
			errcode = -1;
		}
		bus->f2rxdata++;
		ASSERT(errcode != -BCME_PENDING);

		/* On failure, kill the superframe, allow a couple retries */
		if (errcode < 0) {
			DHD_ERROR(("%s: glom read of %d bytes failed: %d\n",
				   __func__, dlen, errcode));
			bus->dhd->rx_errors++;

			if (bus->glomerr++ < 3) {
				dhdsdio_rxfail(bus, true, true);
			} else {
				bus->glomerr = 0;
				dhdsdio_rxfail(bus, true, false);
				dhd_os_sdlock_rxq(bus->dhd);
				bcm_pkt_buf_free_skb(bus->glom);
				dhd_os_sdunlock_rxq(bus->dhd);
=======
			BRCMF_ERROR(("COULDN'T ALLOC %d-BYTE GLOM, "
				     "FORCE FAILURE\n", dlen));
			errcode = -1;
		}
		bus->f2rxdata++;

		/* On failure, kill the superframe, allow a couple retries */
		if (errcode < 0) {
			BRCMF_ERROR(("%s: glom read of %d bytes failed: %d\n",
				     __func__, dlen, errcode));
			bus->drvr->rx_errors++;

			if (bus->glomerr++ < 3) {
				brcmf_sdbrcm_rxfail(bus, true, true);
			} else {
				bus->glomerr = 0;
				brcmf_sdbrcm_rxfail(bus, true, false);
				brcmu_pkt_buf_free_skb(bus->glom);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				bus->rxglomfail++;
				bus->glom = NULL;
			}
			return 0;
		}
<<<<<<< HEAD
#ifdef DHD_DEBUG
		if (DHD_GLOM_ON()) {
=======
#ifdef BCMDBG
		if (BRCMF_GLOM_ON()) {
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			printk(KERN_DEBUG "SUPERFRAME:\n");
			print_hex_dump_bytes("", DUMP_PREFIX_OFFSET,
				pfirst->data, min_t(int, pfirst->len, 48));
		}
#endif

		/* Validate the superframe header */
		dptr = (u8 *) (pfirst->data);
		sublen = get_unaligned_le16(dptr);
		check = get_unaligned_le16(dptr + sizeof(u16));

		chan = SDPCM_PACKET_CHANNEL(&dptr[SDPCM_FRAMETAG_LEN]);
		seq = SDPCM_PACKET_SEQUENCE(&dptr[SDPCM_FRAMETAG_LEN]);
		bus->nextlen = dptr[SDPCM_FRAMETAG_LEN + SDPCM_NEXTLEN_OFFSET];
		if ((bus->nextlen << 4) > MAX_RX_DATASZ) {
<<<<<<< HEAD
			DHD_INFO(("%s: nextlen too large (%d) seq %d\n",
				__func__, bus->nextlen, seq));
=======
			BRCMF_INFO(("%s: nextlen too large (%d) seq %d\n",
				    __func__, bus->nextlen, seq));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			bus->nextlen = 0;
		}
		doff = SDPCM_DOFFSET_VALUE(&dptr[SDPCM_FRAMETAG_LEN]);
		txmax = SDPCM_WINDOW_VALUE(&dptr[SDPCM_FRAMETAG_LEN]);

		errcode = 0;
		if ((u16)~(sublen ^ check)) {
<<<<<<< HEAD
			DHD_ERROR(("%s (superframe): HW hdr error: len/check "
				"0x%04x/0x%04x\n", __func__, sublen, check));
			errcode = -1;
		} else if (roundup(sublen, bus->blocksize) != dlen) {
			DHD_ERROR(("%s (superframe): len 0x%04x, rounded "
				"0x%04x, expect 0x%04x\n",
				__func__, sublen,
				roundup(sublen, bus->blocksize), dlen));
			errcode = -1;
		} else if (SDPCM_PACKET_CHANNEL(&dptr[SDPCM_FRAMETAG_LEN]) !=
			   SDPCM_GLOM_CHANNEL) {
			DHD_ERROR(("%s (superframe): bad channel %d\n",
=======
			BRCMF_ERROR(("%s (superframe): HW hdr error: len/check "
				     "0x%04x/0x%04x\n", __func__, sublen,
				     check));
			errcode = -1;
		} else if (roundup(sublen, bus->blocksize) != dlen) {
			BRCMF_ERROR(("%s (superframe): len 0x%04x, rounded "
				     "0x%04x, expect 0x%04x\n",
				     __func__, sublen,
				     roundup(sublen, bus->blocksize), dlen));
			errcode = -1;
		} else if (SDPCM_PACKET_CHANNEL(&dptr[SDPCM_FRAMETAG_LEN]) !=
			   SDPCM_GLOM_CHANNEL) {
			BRCMF_ERROR(("%s (superframe): bad channel %d\n",
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				   __func__,
				   SDPCM_PACKET_CHANNEL(&dptr
							[SDPCM_FRAMETAG_LEN])));
			errcode = -1;
		} else if (SDPCM_GLOMDESC(&dptr[SDPCM_FRAMETAG_LEN])) {
<<<<<<< HEAD
			DHD_ERROR(("%s (superframe): got second descriptor?\n",
				   __func__));
			errcode = -1;
		} else if ((doff < SDPCM_HDRLEN) ||
			   (doff > (pfirst->len - SDPCM_HDRLEN))) {
			DHD_ERROR(("%s (superframe): Bad data offset %d: HW %d "
				"pkt %d min %d\n",
				__func__, doff, sublen,
				pfirst->len, SDPCM_HDRLEN));
=======
			BRCMF_ERROR(("%s (superframe): got 2nd descriptor?\n",
				     __func__));
			errcode = -1;
		} else if ((doff < SDPCM_HDRLEN) ||
			   (doff > (pfirst->len - SDPCM_HDRLEN))) {
			BRCMF_ERROR(("%s (superframe): Bad data offset %d: "
				     "HW %d pkt %d min %d\n",
				     __func__, doff, sublen,
				     pfirst->len, SDPCM_HDRLEN));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			errcode = -1;
		}

		/* Check sequence number of superframe SW header */
		if (rxseq != seq) {
<<<<<<< HEAD
			DHD_INFO(("%s: (superframe) rx_seq %d, expected %d\n",
				  __func__, seq, rxseq));
=======
			BRCMF_INFO(("%s: (superframe) rx_seq %d, expected %d\n",
				    __func__, seq, rxseq));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			bus->rx_badseq++;
			rxseq = seq;
		}

		/* Check window for sanity */
		if ((u8) (txmax - bus->tx_seq) > 0x40) {
<<<<<<< HEAD
			DHD_ERROR(("%s: unlikely tx max %d with tx_seq %d\n",
				__func__, txmax, bus->tx_seq));
=======
			BRCMF_ERROR(("%s: unlikely tx max %d with tx_seq %d\n",
				     __func__, txmax, bus->tx_seq));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			txmax = bus->tx_seq + 2;
		}
		bus->tx_max = txmax;

		/* Remove superframe header, remember offset */
		skb_pull(pfirst, doff);
		sfdoff = doff;

		/* Validate all the subframe headers */
		for (num = 0, pnext = pfirst; pnext && !errcode;
		     num++, pnext = pnext->next) {
			dptr = (u8 *) (pnext->data);
			dlen = (u16) (pnext->len);
			sublen = get_unaligned_le16(dptr);
			check = get_unaligned_le16(dptr + sizeof(u16));
			chan = SDPCM_PACKET_CHANNEL(&dptr[SDPCM_FRAMETAG_LEN]);
			doff = SDPCM_DOFFSET_VALUE(&dptr[SDPCM_FRAMETAG_LEN]);
<<<<<<< HEAD
#ifdef DHD_DEBUG
			if (DHD_GLOM_ON()) {
=======
#ifdef BCMDBG
			if (BRCMF_GLOM_ON()) {
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				printk(KERN_DEBUG "subframe:\n");
				print_hex_dump_bytes("", DUMP_PREFIX_OFFSET,
						     dptr, 32);
			}
#endif

			if ((u16)~(sublen ^ check)) {
<<<<<<< HEAD
				DHD_ERROR(("%s (subframe %d): HW hdr error: "
					   "len/check 0x%04x/0x%04x\n",
					   __func__, num, sublen, check));
				errcode = -1;
			} else if ((sublen > dlen) || (sublen < SDPCM_HDRLEN)) {
				DHD_ERROR(("%s (subframe %d): length mismatch: "
					   "len 0x%04x, expect 0x%04x\n",
					   __func__, num, sublen, dlen));
				errcode = -1;
			} else if ((chan != SDPCM_DATA_CHANNEL) &&
				   (chan != SDPCM_EVENT_CHANNEL)) {
				DHD_ERROR(("%s (subframe %d): bad channel %d\n",
					   __func__, num, chan));
				errcode = -1;
			} else if ((doff < SDPCM_HDRLEN) || (doff > sublen)) {
				DHD_ERROR(("%s (subframe %d): Bad data offset %d: HW %d min %d\n",
					__func__, num, doff, sublen,
					SDPCM_HDRLEN));
=======
				BRCMF_ERROR(("%s (subframe %d): HW hdr error: "
					     "len/check 0x%04x/0x%04x\n",
					     __func__, num, sublen, check));
				errcode = -1;
			} else if ((sublen > dlen) || (sublen < SDPCM_HDRLEN)) {
				BRCMF_ERROR(("%s (subframe %d): length mismatch"
					     ": len 0x%04x, expect 0x%04x\n",
					     __func__, num, sublen, dlen));
				errcode = -1;
			} else if ((chan != SDPCM_DATA_CHANNEL) &&
				   (chan != SDPCM_EVENT_CHANNEL)) {
				BRCMF_ERROR(("%s (subframe %d): bad channel"
					     " %d\n", __func__, num, chan));
				errcode = -1;
			} else if ((doff < SDPCM_HDRLEN) || (doff > sublen)) {
				BRCMF_ERROR(("%s (subframe %d): Bad data offset"
					     " %d: HW %d min %d\n",
					     __func__, num, doff, sublen,
					     SDPCM_HDRLEN));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				errcode = -1;
			}
		}

		if (errcode) {
			/* Terminate frame on error, request
				 a couple retries */
			if (bus->glomerr++ < 3) {
				/* Restore superframe header space */
				skb_push(pfirst, sfdoff);
<<<<<<< HEAD
				dhdsdio_rxfail(bus, true, true);
			} else {
				bus->glomerr = 0;
				dhdsdio_rxfail(bus, true, false);
				dhd_os_sdlock_rxq(bus->dhd);
				bcm_pkt_buf_free_skb(bus->glom);
				dhd_os_sdunlock_rxq(bus->dhd);
=======
				brcmf_sdbrcm_rxfail(bus, true, true);
			} else {
				bus->glomerr = 0;
				brcmf_sdbrcm_rxfail(bus, true, false);
				brcmu_pkt_buf_free_skb(bus->glom);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				bus->rxglomfail++;
				bus->glom = NULL;
			}
			bus->nextlen = 0;
			return 0;
		}

		/* Basic SD framing looks ok - process each packet (header) */
		save_pfirst = pfirst;
		bus->glom = NULL;
		plast = NULL;

<<<<<<< HEAD
		dhd_os_sdlock_rxq(bus->dhd);
=======
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		for (num = 0; pfirst; rxseq++, pfirst = pnext) {
			pnext = pfirst->next;
			pfirst->next = NULL;

			dptr = (u8 *) (pfirst->data);
			sublen = get_unaligned_le16(dptr);
			chan = SDPCM_PACKET_CHANNEL(&dptr[SDPCM_FRAMETAG_LEN]);
			seq = SDPCM_PACKET_SEQUENCE(&dptr[SDPCM_FRAMETAG_LEN]);
			doff = SDPCM_DOFFSET_VALUE(&dptr[SDPCM_FRAMETAG_LEN]);

<<<<<<< HEAD
			DHD_GLOM(("%s: Get subframe %d, %p(%p/%d), sublen %d "
				"chan %d seq %d\n",
				__func__, num, pfirst, pfirst->data,
				pfirst->len, sublen, chan, seq));

			ASSERT((chan == SDPCM_DATA_CHANNEL)
			       || (chan == SDPCM_EVENT_CHANNEL));

			if (rxseq != seq) {
				DHD_GLOM(("%s: rx_seq %d, expected %d\n",
					  __func__, seq, rxseq));
				bus->rx_badseq++;
				rxseq = seq;
			}
#ifdef DHD_DEBUG
			if (DHD_BYTES_ON() && DHD_DATA_ON()) {
=======
			BRCMF_GLOM(("%s: Get subframe %d, %p(%p/%d), sublen %d "
				    "chan %d seq %d\n",
				    __func__, num, pfirst, pfirst->data,
				    pfirst->len, sublen, chan, seq));

			/* precondition: chan == SDPCM_DATA_CHANNEL ||
					 chan == SDPCM_EVENT_CHANNEL */

			if (rxseq != seq) {
				BRCMF_GLOM(("%s: rx_seq %d, expected %d\n",
					    __func__, seq, rxseq));
				bus->rx_badseq++;
				rxseq = seq;
			}
#ifdef BCMDBG
			if (BRCMF_BYTES_ON() && BRCMF_DATA_ON()) {
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				printk(KERN_DEBUG "Rx Subframe Data:\n");
				print_hex_dump_bytes("", DUMP_PREFIX_OFFSET,
						     dptr, dlen);
			}
#endif

			__skb_trim(pfirst, sublen);
			skb_pull(pfirst, doff);

			if (pfirst->len == 0) {
<<<<<<< HEAD
				bcm_pkt_buf_free_skb(pfirst);
				if (plast) {
					plast->next = pnext;
				} else {
					ASSERT(save_pfirst == pfirst);
					save_pfirst = pnext;
				}
				continue;
			} else if (dhd_prot_hdrpull(bus->dhd, &ifidx, pfirst) !=
				   0) {
				DHD_ERROR(("%s: rx protocol error\n",
					   __func__));
				bus->dhd->rx_errors++;
				bcm_pkt_buf_free_skb(pfirst);
				if (plast) {
					plast->next = pnext;
				} else {
					ASSERT(save_pfirst == pfirst);
=======
				brcmu_pkt_buf_free_skb(pfirst);
				if (plast) {
					plast->next = pnext;
				} else {
					save_pfirst = pnext;
				}
				continue;
			} else if (brcmf_proto_hdrpull(bus->drvr, &ifidx, pfirst)
					!= 0) {
				BRCMF_ERROR(("%s: rx protocol error\n",
					     __func__));
				bus->drvr->rx_errors++;
				brcmu_pkt_buf_free_skb(pfirst);
				if (plast) {
					plast->next = pnext;
				} else {
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
					save_pfirst = pnext;
				}
				continue;
			}

			/* this packet will go up, link back into
				 chain and count it */
			pfirst->next = pnext;
			plast = pfirst;
			num++;

<<<<<<< HEAD
#ifdef DHD_DEBUG
			if (DHD_GLOM_ON()) {
				DHD_GLOM(("%s subframe %d to stack, %p(%p/%d) "
				"nxt/lnk %p/%p\n",
				__func__, num, pfirst, pfirst->data,
				pfirst->len, pfirst->next,
				pfirst->prev));
=======
#ifdef BCMDBG
			if (BRCMF_GLOM_ON()) {
				BRCMF_GLOM(("%s subframe %d to stack, %p"
					    "(%p/%d) nxt/lnk %p/%p\n",
					    __func__, num, pfirst, pfirst->data,
					    pfirst->len, pfirst->next,
					    pfirst->prev));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				print_hex_dump_bytes("", DUMP_PREFIX_OFFSET,
						pfirst->data,
						min_t(int, pfirst->len, 32));
			}
<<<<<<< HEAD
#endif				/* DHD_DEBUG */
		}
		dhd_os_sdunlock_rxq(bus->dhd);
		if (num) {
			dhd_os_sdunlock(bus->dhd);
			dhd_rx_frame(bus->dhd, ifidx, save_pfirst, num);
			dhd_os_sdlock(bus->dhd);
=======
#endif				/* BCMDBG */
		}
		if (num) {
			brcmf_sdbrcm_sdunlock(bus);
			brcmf_rx_frame(bus->drvr, ifidx, save_pfirst, num);
			brcmf_sdbrcm_sdlock(bus);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		}

		bus->rxglomframes++;
		bus->rxglompkts += num;
	}
	return num;
}

/* Return true if there may be more frames to read */
<<<<<<< HEAD
static uint dhdsdio_readframes(dhd_bus_t *bus, uint maxframes, bool *finished)
{
	bcmsdh_info_t *sdh = bus->sdh;
=======
static uint
brcmf_sdbrcm_readframes(struct brcmf_bus *bus, uint maxframes, bool *finished)
{
	struct brcmf_sdio_card *card = bus->card;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	u16 len, check;	/* Extracted hardware header fields */
	u8 chan, seq, doff;	/* Extracted software header fields */
	u8 fcbits;		/* Extracted fcbits from software header */

	struct sk_buff *pkt;		/* Packet for event or data frames */
	u16 pad;		/* Number of pad bytes to read */
	u16 rdlen;		/* Total number of bytes to read */
	u8 rxseq;		/* Next sequence number to expect */
	uint rxleft = 0;	/* Remaining number of frames allowed */
<<<<<<< HEAD
	int sdret;		/* Return code from bcmsdh calls */
=======
	int sdret;		/* Return code from calls */
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	u8 txmax;		/* Maximum tx sequence offered */
	bool len_consistent;	/* Result of comparing readahead len and
					 len from hw-hdr */
	u8 *rxbuf;
	int ifidx = 0;
	uint rxcount = 0;	/* Total frames read */

<<<<<<< HEAD
#if defined(DHD_DEBUG) || defined(SDTEST)
	bool sdtest = false;	/* To limit message spew from test mode */
#endif

	DHD_TRACE(("%s: Enter\n", __func__));

	ASSERT(maxframes);

#ifdef SDTEST
	/* Allow pktgen to override maxframes */
	if (bus->pktgen_count && (bus->pktgen_mode == DHD_PKTGEN_RECV)) {
=======
#if defined(BCMDBG) || defined(SDTEST)
	bool sdtest = false;	/* To limit message spew from test mode */
#endif

	BRCMF_TRACE(("%s: Enter\n", __func__));

#ifdef SDTEST
	/* Allow pktgen to override maxframes */
	if (bus->pktgen_count && (bus->pktgen_mode == BRCMF_PKTGEN_RECV)) {
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		maxframes = bus->pktgen_count;
		sdtest = true;
	}
#endif

	/* Not finished unless we encounter no more frames indication */
	*finished = false;

	for (rxseq = bus->rx_seq, rxleft = maxframes;
<<<<<<< HEAD
	     !bus->rxskip && rxleft && bus->dhd->busstate != DHD_BUS_DOWN;
=======
	     !bus->rxskip && rxleft && bus->drvr->busstate != BRCMF_BUS_DOWN;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	     rxseq++, rxleft--) {

		/* Handle glomming separately */
		if (bus->glom || bus->glomd) {
			u8 cnt;
<<<<<<< HEAD
			DHD_GLOM(("%s: calling rxglom: glomd %p, glom %p\n",
				  __func__, bus->glomd, bus->glom));
			cnt = dhdsdio_rxglom(bus, rxseq);
			DHD_GLOM(("%s: rxglom returned %d\n", __func__, cnt));
=======
			BRCMF_GLOM(("%s: calling rxglom: glomd %p, glom %p\n",
				    __func__, bus->glomd, bus->glom));
			cnt = brcmf_sdbrcm_rxglom(bus, rxseq);
			BRCMF_GLOM(("%s: rxglom returned %d\n", __func__, cnt));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			rxseq += cnt - 1;
			rxleft = (rxleft > cnt) ? (rxleft - cnt) : 1;
			continue;
		}

		/* Try doing single read if we can */
<<<<<<< HEAD
		if (dhd_readahead && bus->nextlen) {
=======
		if (brcmf_readahead && bus->nextlen) {
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			u16 nextlen = bus->nextlen;
			bus->nextlen = 0;

			if (bus->bus == SPI_BUS) {
				rdlen = len = nextlen;
			} else {
				rdlen = len = nextlen << 4;

				/* Pad read to blocksize for efficiency */
				if (bus->roundup && bus->blocksize
				    && (rdlen > bus->blocksize)) {
					pad =
					    bus->blocksize -
					    (rdlen % bus->blocksize);
					if ((pad <= bus->roundup)
					    && (pad < bus->blocksize)
					    && ((rdlen + pad + firstread) <
						MAX_RX_DATASZ))
						rdlen += pad;
<<<<<<< HEAD
				} else if (rdlen % DHD_SDALIGN) {
					rdlen +=
					    DHD_SDALIGN - (rdlen % DHD_SDALIGN);
=======
				} else if (rdlen % BRCMF_SDALIGN) {
					rdlen +=
					    BRCMF_SDALIGN - (rdlen % BRCMF_SDALIGN);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				}
			}

			/* We use bus->rxctl buffer in WinXP for initial
			 * control pkt receives.
			 * Later we use buffer-poll for data as well
			 * as control packets.
			 * This is required because dhd receives full
			 * frame in gSPI unlike SDIO.
			 * After the frame is received we have to
			 * distinguish whether it is data
			 * or non-data frame.
			 */
			/* Allocate a packet buffer */
<<<<<<< HEAD
			dhd_os_sdlock_rxq(bus->dhd);
			pkt = bcm_pkt_buf_get_skb(rdlen + DHD_SDALIGN);
=======
			pkt = brcmu_pkt_buf_get_skb(rdlen + BRCMF_SDALIGN);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			if (!pkt) {
				if (bus->bus == SPI_BUS) {
					bus->usebufpool = false;
					bus->rxctl = bus->rxbuf;
<<<<<<< HEAD
					if (dhd_alignctl) {
						bus->rxctl += firstread;
						pad = ((unsigned long)bus->rxctl %
						      DHD_SDALIGN);
						if (pad)
							bus->rxctl +=
							    (DHD_SDALIGN - pad);
						bus->rxctl -= firstread;
					}
					ASSERT(bus->rxctl >= bus->rxbuf);
					rxbuf = bus->rxctl;
					/* Read the entire frame */
					sdret = bcmsdh_recv_buf(bus,
						    bcmsdh_cur_sbwad(sdh),
						    SDIO_FUNC_2, F2SYNC,
						    rxbuf, rdlen,
						    NULL, NULL, NULL);
					bus->f2rxdata++;
					ASSERT(sdret != -BCME_PENDING);
=======
					if (brcmf_alignctl) {
						bus->rxctl += firstread;
						pad = ((unsigned long)bus->rxctl %
						      BRCMF_SDALIGN);
						if (pad)
							bus->rxctl +=
							    (BRCMF_SDALIGN - pad);
						bus->rxctl -= firstread;
					}
					rxbuf = bus->rxctl;
					/* Read the entire frame */
					sdret = brcmf_sdcard_recv_buf(card,
						   brcmf_sdcard_cur_sbwad(card),
						   SDIO_FUNC_2, F2SYNC,
						   rxbuf, rdlen,
						   NULL, NULL, NULL);
					bus->f2rxdata++;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

					/* Control frame failures need
					 retransmission */
					if (sdret < 0) {
<<<<<<< HEAD
						DHD_ERROR(("%s: read %d control bytes failed: %d\n",
							__func__,
							rdlen, sdret));
						/* dhd.rx_ctlerrs is higher */
						bus->rxc_errors++;
						dhd_os_sdunlock_rxq(bus->dhd);
						dhdsdio_rxfail(bus, true,
=======
						BRCMF_ERROR(("%s: read %d "
							     "control bytes "
							     "failed: %d\n",
							     __func__,
							     rdlen, sdret));
						/* dhd.rx_ctlerrs is higher */
						bus->rxc_errors++;
						brcmf_sdbrcm_rxfail(bus, true,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
						       (bus->bus ==
							SPI_BUS) ? false
						       : true);
						continue;
					}
				} else {
					/* Give up on data,
					request rtx of events */
<<<<<<< HEAD
					DHD_ERROR(("%s (nextlen): "
						   "bcm_pkt_buf_get_skb failed:"
						   " len %d rdlen %d expected"
						   " rxseq %d\n", __func__,
						   len, rdlen, rxseq));
					/* Just go try again w/normal
					header read */
					dhd_os_sdunlock_rxq(bus->dhd);
=======
					BRCMF_ERROR(("%s (nextlen): "
						     "brcmu_pkt_buf_get_skb "
						     "failed:"
						     " len %d rdlen %d expected"
						     " rxseq %d\n", __func__,
						     len, rdlen, rxseq));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
					continue;
				}
			} else {
				if (bus->bus == SPI_BUS)
					bus->usebufpool = true;

<<<<<<< HEAD
				ASSERT(!(pkt->prev));
				PKTALIGN(pkt, rdlen, DHD_SDALIGN);
				rxbuf = (u8 *) (pkt->data);
				/* Read the entire frame */
				sdret = bcmsdh_recv_buf(bus,
						bcmsdh_cur_sbwad(sdh),
=======
				PKTALIGN(pkt, rdlen, BRCMF_SDALIGN);
				rxbuf = (u8 *) (pkt->data);
				/* Read the entire frame */
				sdret = brcmf_sdcard_recv_buf(card,
						brcmf_sdcard_cur_sbwad(card),
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
						SDIO_FUNC_2, F2SYNC,
						rxbuf, rdlen,
						pkt, NULL, NULL);
				bus->f2rxdata++;
<<<<<<< HEAD
				ASSERT(sdret != -BCME_PENDING);

				if (sdret < 0) {
					DHD_ERROR(("%s (nextlen): read %d bytes failed: %d\n",
						__func__, rdlen, sdret));
					bcm_pkt_buf_free_skb(pkt);
					bus->dhd->rx_errors++;
					dhd_os_sdunlock_rxq(bus->dhd);
=======

				if (sdret < 0) {
					BRCMF_ERROR(("%s (nextlen): read %d"
						     " bytes failed: %d\n",
						     __func__, rdlen, sdret));
					brcmu_pkt_buf_free_skb(pkt);
					bus->drvr->rx_errors++;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
					/* Force retry w/normal header read.
					 * Don't attempt NAK for
					 * gSPI
					 */
<<<<<<< HEAD
					dhdsdio_rxfail(bus, true,
=======
					brcmf_sdbrcm_rxfail(bus, true,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
						       (bus->bus ==
							SPI_BUS) ? false :
						       true);
					continue;
				}
			}
<<<<<<< HEAD
			dhd_os_sdunlock_rxq(bus->dhd);
=======
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

			/* Now check the header */
			memcpy(bus->rxhdr, rxbuf, SDPCM_HDRLEN);

			/* Extract hardware header fields */
			len = get_unaligned_le16(bus->rxhdr);
			check = get_unaligned_le16(bus->rxhdr + sizeof(u16));

			/* All zeros means readahead info was bad */
			if (!(len | check)) {
<<<<<<< HEAD
				DHD_INFO(("%s (nextlen): read zeros in HW "
					"header???\n", __func__));
				dhdsdio_pktfree2(bus, pkt);
=======
				BRCMF_INFO(("%s (nextlen): read zeros in HW "
					    "header???\n", __func__));
				brcmf_sdbrcm_pktfree2(bus, pkt);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				continue;
			}

			/* Validate check bytes */
			if ((u16)~(len ^ check)) {
<<<<<<< HEAD
				DHD_ERROR(("%s (nextlen): HW hdr error:"
					" nextlen/len/check"
					" 0x%04x/0x%04x/0x%04x\n",
					__func__, nextlen, len, check));
				bus->rx_badhdr++;
				dhdsdio_rxfail(bus, false, false);
				dhdsdio_pktfree2(bus, pkt);
=======
				BRCMF_ERROR(("%s (nextlen): HW hdr error:"
					     " nextlen/len/check"
					     " 0x%04x/0x%04x/0x%04x\n",
					     __func__, nextlen, len, check));
				bus->rx_badhdr++;
				brcmf_sdbrcm_rxfail(bus, false, false);
				brcmf_sdbrcm_pktfree2(bus, pkt);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				continue;
			}

			/* Validate frame length */
			if (len < SDPCM_HDRLEN) {
<<<<<<< HEAD
				DHD_ERROR(("%s (nextlen): HW hdr length "
					"invalid: %d\n", __func__, len));
				dhdsdio_pktfree2(bus, pkt);
=======
				BRCMF_ERROR(("%s (nextlen): HW hdr length "
					     "invalid: %d\n", __func__, len));
				brcmf_sdbrcm_pktfree2(bus, pkt);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				continue;
			}

			/* Check for consistency withreadahead info */
			len_consistent = (nextlen != (roundup(len, 16) >> 4));
			if (len_consistent) {
				/* Mismatch, force retry w/normal
					header (may be >4K) */
<<<<<<< HEAD
				DHD_ERROR(("%s (nextlen): mismatch, "
					"nextlen %d len %d rnd %d; "
					"expected rxseq %d\n",
					__func__, nextlen,
					len, roundup(len, 16), rxseq));
				dhdsdio_rxfail(bus, true, (bus->bus != SPI_BUS));
				dhdsdio_pktfree2(bus, pkt);
=======
				BRCMF_ERROR(("%s (nextlen): mismatch, "
					     "nextlen %d len %d rnd %d; "
					     "expected rxseq %d\n",
					     __func__, nextlen,
					     len, roundup(len, 16), rxseq));
				brcmf_sdbrcm_rxfail(bus, true,
						  bus->bus != SPI_BUS);
				brcmf_sdbrcm_pktfree2(bus, pkt);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				continue;
			}

			/* Extract software header fields */
			chan = SDPCM_PACKET_CHANNEL(
					&bus->rxhdr[SDPCM_FRAMETAG_LEN]);
			seq = SDPCM_PACKET_SEQUENCE(
					&bus->rxhdr[SDPCM_FRAMETAG_LEN]);
			doff = SDPCM_DOFFSET_VALUE(
					&bus->rxhdr[SDPCM_FRAMETAG_LEN]);
			txmax = SDPCM_WINDOW_VALUE(
					&bus->rxhdr[SDPCM_FRAMETAG_LEN]);

			bus->nextlen =
			    bus->rxhdr[SDPCM_FRAMETAG_LEN +
				       SDPCM_NEXTLEN_OFFSET];
			if ((bus->nextlen << 4) > MAX_RX_DATASZ) {
<<<<<<< HEAD
				DHD_INFO(("%s (nextlen): got frame w/nextlen too large" " (%d), seq %d\n",
					__func__, bus->nextlen, seq));
				bus->nextlen = 0;
			}

			bus->dhd->rx_readahead_cnt++;
=======
				BRCMF_INFO(("%s (nextlen): got frame w/nextlen"
					    " too large (%d), seq %d\n",
					    __func__, bus->nextlen, seq));
				bus->nextlen = 0;
			}

			bus->drvr->rx_readahead_cnt++;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

			/* Handle Flow Control */
			fcbits = SDPCM_FCMASK_VALUE(
					&bus->rxhdr[SDPCM_FRAMETAG_LEN]);

			if (bus->flowcontrol != fcbits) {
				if (~bus->flowcontrol & fcbits)
					bus->fc_xoff++;

				if (bus->flowcontrol & ~fcbits)
					bus->fc_xon++;

				bus->fc_rcvd++;
				bus->flowcontrol = fcbits;
			}

			/* Check and update sequence number */
			if (rxseq != seq) {
<<<<<<< HEAD
				DHD_INFO(("%s (nextlen): rx_seq %d, expected "
					"%d\n", __func__, seq, rxseq));
=======
				BRCMF_INFO(("%s (nextlen): rx_seq %d, expected "
					    "%d\n", __func__, seq, rxseq));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				bus->rx_badseq++;
				rxseq = seq;
			}

			/* Check window for sanity */
			if ((u8) (txmax - bus->tx_seq) > 0x40) {
<<<<<<< HEAD
				DHD_ERROR(("%s: got unlikely tx max %d with "
					"tx_seq %d\n",
					__func__, txmax, bus->tx_seq));
=======
				BRCMF_ERROR(("%s: got unlikely tx max %d with "
					     "tx_seq %d\n",
					     __func__, txmax, bus->tx_seq));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				txmax = bus->tx_seq + 2;
			}
			bus->tx_max = txmax;

<<<<<<< HEAD
#ifdef DHD_DEBUG
			if (DHD_BYTES_ON() && DHD_DATA_ON()) {
				printk(KERN_DEBUG "Rx Data:\n");
				print_hex_dump_bytes("", DUMP_PREFIX_OFFSET,
						     rxbuf, len);
			} else if (DHD_HDRS_ON()) {
=======
#ifdef BCMDBG
			if (BRCMF_BYTES_ON() && BRCMF_DATA_ON()) {
				printk(KERN_DEBUG "Rx Data:\n");
				print_hex_dump_bytes("", DUMP_PREFIX_OFFSET,
						     rxbuf, len);
			} else if (BRCMF_HDRS_ON()) {
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				printk(KERN_DEBUG "RxHdr:\n");
				print_hex_dump_bytes("", DUMP_PREFIX_OFFSET,
						     bus->rxhdr, SDPCM_HDRLEN);
			}
#endif

			if (chan == SDPCM_CONTROL_CHANNEL) {
				if (bus->bus == SPI_BUS) {
<<<<<<< HEAD
					dhdsdio_read_control(bus, rxbuf, len,
							     doff);
				} else {
					DHD_ERROR(("%s (nextlen): readahead on control" " packet %d?\n",
						__func__, seq));
					/* Force retry w/normal header read */
					bus->nextlen = 0;
					dhdsdio_rxfail(bus, false, true);
				}
				dhdsdio_pktfree2(bus, pkt);
=======
					brcmf_sdbrcm_read_control(bus, rxbuf,
								  len, doff);
				} else {
					BRCMF_ERROR(("%s (nextlen): readahead"
						     " on control packet %d?\n",
						     __func__, seq));
					/* Force retry w/normal header read */
					bus->nextlen = 0;
					brcmf_sdbrcm_rxfail(bus, false, true);
				}
				brcmf_sdbrcm_pktfree2(bus, pkt);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				continue;
			}

			if ((bus->bus == SPI_BUS) && !bus->usebufpool) {
<<<<<<< HEAD
				DHD_ERROR(("Received %d bytes on %d channel. Running out of " "rx pktbuf's or not yet malloced.\n",
					len, chan));
=======
				BRCMF_ERROR(("Received %d bytes on %d channel."
					     " Running out of " "rx pktbuf's or"
					     " not yet malloced.\n",
					     len, chan));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				continue;
			}

			/* Validate data offset */
			if ((doff < SDPCM_HDRLEN) || (doff > len)) {
<<<<<<< HEAD
				DHD_ERROR(("%s (nextlen): bad data offset %d: HW len %d min %d\n",
					__func__, doff, len, SDPCM_HDRLEN));
				dhdsdio_rxfail(bus, false, false);
				dhdsdio_pktfree2(bus, pkt);
=======
				BRCMF_ERROR(("%s (nextlen): bad data offset %d:"
					     " HW len %d min %d\n", __func__,
					     doff, len, SDPCM_HDRLEN));
				brcmf_sdbrcm_rxfail(bus, false, false);
				brcmf_sdbrcm_pktfree2(bus, pkt);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				continue;
			}

			/* All done with this one -- now deliver the packet */
			goto deliver;
		}
		/* gSPI frames should not be handled in fractions */
		if (bus->bus == SPI_BUS)
			break;

		/* Read frame header (hardware and software) */
<<<<<<< HEAD
		sdret = bcmsdh_recv_buf(bus, bcmsdh_cur_sbwad(sdh),
				SDIO_FUNC_2, F2SYNC, bus->rxhdr, firstread,
				NULL, NULL, NULL);
		bus->f2rxhdrs++;
		ASSERT(sdret != -BCME_PENDING);

		if (sdret < 0) {
			DHD_ERROR(("%s: RXHEADER FAILED: %d\n", __func__,
				   sdret));
			bus->rx_hdrfail++;
			dhdsdio_rxfail(bus, true, true);
			continue;
		}
#ifdef DHD_DEBUG
		if (DHD_BYTES_ON() || DHD_HDRS_ON()) {
=======
		sdret = brcmf_sdcard_recv_buf(card,
				brcmf_sdcard_cur_sbwad(card),
				SDIO_FUNC_2, F2SYNC, bus->rxhdr, firstread,
				NULL, NULL, NULL);
		bus->f2rxhdrs++;

		if (sdret < 0) {
			BRCMF_ERROR(("%s: RXHEADER FAILED: %d\n", __func__,
				     sdret));
			bus->rx_hdrfail++;
			brcmf_sdbrcm_rxfail(bus, true, true);
			continue;
		}
#ifdef BCMDBG
		if (BRCMF_BYTES_ON() || BRCMF_HDRS_ON()) {
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			printk(KERN_DEBUG "RxHdr:\n");
			print_hex_dump_bytes("", DUMP_PREFIX_OFFSET,
					     bus->rxhdr, SDPCM_HDRLEN);
		}
#endif

		/* Extract hardware header fields */
		len = get_unaligned_le16(bus->rxhdr);
		check = get_unaligned_le16(bus->rxhdr + sizeof(u16));

		/* All zeros means no more frames */
		if (!(len | check)) {
			*finished = true;
			break;
		}

		/* Validate check bytes */
		if ((u16) ~(len ^ check)) {
<<<<<<< HEAD
			DHD_ERROR(("%s: HW hdr err: len/check 0x%04x/0x%04x\n",
				__func__, len, check));
			bus->rx_badhdr++;
			dhdsdio_rxfail(bus, false, false);
=======
			BRCMF_ERROR(("%s: HW hdr err: len/check "
				     "0x%04x/0x%04x\n", __func__, len, check));
			bus->rx_badhdr++;
			brcmf_sdbrcm_rxfail(bus, false, false);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			continue;
		}

		/* Validate frame length */
		if (len < SDPCM_HDRLEN) {
<<<<<<< HEAD
			DHD_ERROR(("%s: HW hdr length invalid: %d\n",
				   __func__, len));
=======
			BRCMF_ERROR(("%s: HW hdr length invalid: %d\n",
				     __func__, len));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			continue;
		}

		/* Extract software header fields */
		chan = SDPCM_PACKET_CHANNEL(&bus->rxhdr[SDPCM_FRAMETAG_LEN]);
		seq = SDPCM_PACKET_SEQUENCE(&bus->rxhdr[SDPCM_FRAMETAG_LEN]);
		doff = SDPCM_DOFFSET_VALUE(&bus->rxhdr[SDPCM_FRAMETAG_LEN]);
		txmax = SDPCM_WINDOW_VALUE(&bus->rxhdr[SDPCM_FRAMETAG_LEN]);

		/* Validate data offset */
		if ((doff < SDPCM_HDRLEN) || (doff > len)) {
<<<<<<< HEAD
			DHD_ERROR(("%s: Bad data offset %d: HW len %d, min %d "
				"seq %d\n",
				__func__, doff, len, SDPCM_HDRLEN, seq));
			bus->rx_badhdr++;
			ASSERT(0);
			dhdsdio_rxfail(bus, false, false);
=======
			BRCMF_ERROR(("%s: Bad data offset %d: HW len %d,"
				     " min %d seq %d\n", __func__, doff,
				     len, SDPCM_HDRLEN, seq));
			bus->rx_badhdr++;
			brcmf_sdbrcm_rxfail(bus, false, false);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			continue;
		}

		/* Save the readahead length if there is one */
		bus->nextlen =
		    bus->rxhdr[SDPCM_FRAMETAG_LEN + SDPCM_NEXTLEN_OFFSET];
		if ((bus->nextlen << 4) > MAX_RX_DATASZ) {
<<<<<<< HEAD
			DHD_INFO(("%s (nextlen): got frame w/nextlen too large "
				"(%d), seq %d\n",
				__func__, bus->nextlen, seq));
=======
			BRCMF_INFO(("%s (nextlen): got frame w/nextlen too"
				    " large (%d), seq %d\n",
				    __func__, bus->nextlen, seq));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			bus->nextlen = 0;
		}

		/* Handle Flow Control */
		fcbits = SDPCM_FCMASK_VALUE(&bus->rxhdr[SDPCM_FRAMETAG_LEN]);

		if (bus->flowcontrol != fcbits) {
			if (~bus->flowcontrol & fcbits)
				bus->fc_xoff++;

			if (bus->flowcontrol & ~fcbits)
				bus->fc_xon++;

			bus->fc_rcvd++;
			bus->flowcontrol = fcbits;
		}

		/* Check and update sequence number */
		if (rxseq != seq) {
<<<<<<< HEAD
			DHD_INFO(("%s: rx_seq %d, expected %d\n", __func__,
				  seq, rxseq));
=======
			BRCMF_INFO(("%s: rx_seq %d, expected %d\n", __func__,
				    seq, rxseq));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			bus->rx_badseq++;
			rxseq = seq;
		}

		/* Check window for sanity */
		if ((u8) (txmax - bus->tx_seq) > 0x40) {
<<<<<<< HEAD
			DHD_ERROR(("%s: unlikely tx max %d with tx_seq %d\n",
				__func__, txmax, bus->tx_seq));
=======
			BRCMF_ERROR(("%s: unlikely tx max %d with tx_seq %d\n",
				     __func__, txmax, bus->tx_seq));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			txmax = bus->tx_seq + 2;
		}
		bus->tx_max = txmax;

		/* Call a separate function for control frames */
		if (chan == SDPCM_CONTROL_CHANNEL) {
<<<<<<< HEAD
			dhdsdio_read_control(bus, bus->rxhdr, len, doff);
			continue;
		}

		ASSERT((chan == SDPCM_DATA_CHANNEL)
		       || (chan == SDPCM_EVENT_CHANNEL)
		       || (chan == SDPCM_TEST_CHANNEL)
		       || (chan == SDPCM_GLOM_CHANNEL));
=======
			brcmf_sdbrcm_read_control(bus, bus->rxhdr, len, doff);
			continue;
		}

		/* precondition: chan is either SDPCM_DATA_CHANNEL,
		   SDPCM_EVENT_CHANNEL, SDPCM_TEST_CHANNEL or
		   SDPCM_GLOM_CHANNEL */
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

		/* Length to read */
		rdlen = (len > firstread) ? (len - firstread) : 0;

		/* May pad read to blocksize for efficiency */
		if (bus->roundup && bus->blocksize &&
			(rdlen > bus->blocksize)) {
			pad = bus->blocksize - (rdlen % bus->blocksize);
			if ((pad <= bus->roundup) && (pad < bus->blocksize) &&
			    ((rdlen + pad + firstread) < MAX_RX_DATASZ))
				rdlen += pad;
<<<<<<< HEAD
		} else if (rdlen % DHD_SDALIGN) {
			rdlen += DHD_SDALIGN - (rdlen % DHD_SDALIGN);
=======
		} else if (rdlen % BRCMF_SDALIGN) {
			rdlen += BRCMF_SDALIGN - (rdlen % BRCMF_SDALIGN);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		}

		/* Satisfy length-alignment requirements */
		if (forcealign && (rdlen & (ALIGNMENT - 1)))
			rdlen = roundup(rdlen, ALIGNMENT);

		if ((rdlen + firstread) > MAX_RX_DATASZ) {
			/* Too long -- skip this frame */
<<<<<<< HEAD
			DHD_ERROR(("%s: too long: len %d rdlen %d\n",
				   __func__, len, rdlen));
			bus->dhd->rx_errors++;
			bus->rx_toolong++;
			dhdsdio_rxfail(bus, false, false);
			continue;
		}

		dhd_os_sdlock_rxq(bus->dhd);
		pkt = bcm_pkt_buf_get_skb(rdlen + firstread + DHD_SDALIGN);
		if (!pkt) {
			/* Give up on data, request rtx of events */
			DHD_ERROR(("%s: bcm_pkt_buf_get_skb failed: rdlen %d "
				"chan %d\n", __func__, rdlen, chan));
			bus->dhd->rx_dropped++;
			dhd_os_sdunlock_rxq(bus->dhd);
			dhdsdio_rxfail(bus, false, RETRYCHAN(chan));
			continue;
		}
		dhd_os_sdunlock_rxq(bus->dhd);

		ASSERT(!(pkt->prev));

		/* Leave room for what we already read, and align remainder */
		ASSERT(firstread < pkt->len);
		skb_pull(pkt, firstread);
		PKTALIGN(pkt, rdlen, DHD_SDALIGN);

		/* Read the remaining frame data */
		sdret = bcmsdh_recv_buf(bus, bcmsdh_cur_sbwad(sdh), SDIO_FUNC_2,
					F2SYNC, ((u8 *) (pkt->data)), rdlen,
					pkt, NULL, NULL);
		bus->f2rxdata++;
		ASSERT(sdret != -BCME_PENDING);

		if (sdret < 0) {
			DHD_ERROR(("%s: read %d %s bytes failed: %d\n",
				   __func__, rdlen,
				   ((chan ==
				     SDPCM_EVENT_CHANNEL) ? "event" : ((chan ==
					SDPCM_DATA_CHANNEL)
				       ? "data" : "test")),
				   sdret));
			dhd_os_sdlock_rxq(bus->dhd);
			bcm_pkt_buf_free_skb(pkt);
			dhd_os_sdunlock_rxq(bus->dhd);
			bus->dhd->rx_errors++;
			dhdsdio_rxfail(bus, true, RETRYCHAN(chan));
=======
			BRCMF_ERROR(("%s: too long: len %d rdlen %d\n",
				     __func__, len, rdlen));
			bus->drvr->rx_errors++;
			bus->rx_toolong++;
			brcmf_sdbrcm_rxfail(bus, false, false);
			continue;
		}

		pkt = brcmu_pkt_buf_get_skb(rdlen + firstread + BRCMF_SDALIGN);
		if (!pkt) {
			/* Give up on data, request rtx of events */
			BRCMF_ERROR(("%s: brcmu_pkt_buf_get_skb failed:"
				     " rdlen %d chan %d\n", __func__, rdlen,
				     chan));
			bus->drvr->rx_dropped++;
			brcmf_sdbrcm_rxfail(bus, false, RETRYCHAN(chan));
			continue;
		}

		/* Leave room for what we already read, and align remainder */
		skb_pull(pkt, firstread);
		PKTALIGN(pkt, rdlen, BRCMF_SDALIGN);

		/* Read the remaining frame data */
		sdret = brcmf_sdcard_recv_buf(card,
				brcmf_sdcard_cur_sbwad(card),
				SDIO_FUNC_2, F2SYNC, ((u8 *) (pkt->data)),
				rdlen, pkt, NULL, NULL);
		bus->f2rxdata++;

		if (sdret < 0) {
			BRCMF_ERROR(("%s: read %d %s bytes failed: %d\n",
				     __func__, rdlen,
				     ((chan == SDPCM_EVENT_CHANNEL) ? "event"
				     : ((chan == SDPCM_DATA_CHANNEL) ? "data"
				     : "test")), sdret));
			brcmu_pkt_buf_free_skb(pkt);
			bus->drvr->rx_errors++;
			brcmf_sdbrcm_rxfail(bus, true, RETRYCHAN(chan));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			continue;
		}

		/* Copy the already-read portion */
		skb_push(pkt, firstread);
		memcpy(pkt->data, bus->rxhdr, firstread);

<<<<<<< HEAD
#ifdef DHD_DEBUG
		if (DHD_BYTES_ON() && DHD_DATA_ON()) {
=======
#ifdef BCMDBG
		if (BRCMF_BYTES_ON() && BRCMF_DATA_ON()) {
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			printk(KERN_DEBUG "Rx Data:\n");
			print_hex_dump_bytes("", DUMP_PREFIX_OFFSET,
					     pkt->data, len);
		}
#endif

deliver:
		/* Save superframe descriptor and allocate packet frame */
		if (chan == SDPCM_GLOM_CHANNEL) {
			if (SDPCM_GLOMDESC(&bus->rxhdr[SDPCM_FRAMETAG_LEN])) {
<<<<<<< HEAD
				DHD_GLOM(("%s: glom descriptor, %d bytes:\n",
					__func__, len));
#ifdef DHD_DEBUG
				if (DHD_GLOM_ON()) {
=======
				BRCMF_GLOM(("%s: glom descriptor, %d bytes:\n",
					    __func__, len));
#ifdef BCMDBG
				if (BRCMF_GLOM_ON()) {
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
					printk(KERN_DEBUG "Glom Data:\n");
					print_hex_dump_bytes("",
							     DUMP_PREFIX_OFFSET,
							     pkt->data, len);
				}
#endif
				__skb_trim(pkt, len);
<<<<<<< HEAD
				ASSERT(doff == SDPCM_HDRLEN);
				skb_pull(pkt, SDPCM_HDRLEN);
				bus->glomd = pkt;
			} else {
				DHD_ERROR(("%s: glom superframe w/o "
					"descriptor!\n", __func__));
				dhdsdio_rxfail(bus, false, false);
=======
				skb_pull(pkt, SDPCM_HDRLEN);
				bus->glomd = pkt;
			} else {
				BRCMF_ERROR(("%s: glom superframe w/o "
					     "descriptor!\n", __func__));
				brcmf_sdbrcm_rxfail(bus, false, false);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			}
			continue;
		}

		/* Fill in packet len and prio, deliver upward */
		__skb_trim(pkt, len);
		skb_pull(pkt, doff);

#ifdef SDTEST
		/* Test channel packets are processed separately */
		if (chan == SDPCM_TEST_CHANNEL) {
<<<<<<< HEAD
			dhdsdio_testrcv(bus, pkt, seq);
=======
			brcmf_sdbrcm_checkdied(bus, pkt, seq);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			continue;
		}
#endif				/* SDTEST */

		if (pkt->len == 0) {
<<<<<<< HEAD
			dhd_os_sdlock_rxq(bus->dhd);
			bcm_pkt_buf_free_skb(pkt);
			dhd_os_sdunlock_rxq(bus->dhd);
			continue;
		} else if (dhd_prot_hdrpull(bus->dhd, &ifidx, pkt) != 0) {
			DHD_ERROR(("%s: rx protocol error\n", __func__));
			dhd_os_sdlock_rxq(bus->dhd);
			bcm_pkt_buf_free_skb(pkt);
			dhd_os_sdunlock_rxq(bus->dhd);
			bus->dhd->rx_errors++;
=======
			brcmu_pkt_buf_free_skb(pkt);
			continue;
		} else if (brcmf_proto_hdrpull(bus->drvr, &ifidx, pkt) != 0) {
			BRCMF_ERROR(("%s: rx protocol error\n", __func__));
			brcmu_pkt_buf_free_skb(pkt);
			bus->drvr->rx_errors++;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			continue;
		}

		/* Unlock during rx call */
<<<<<<< HEAD
		dhd_os_sdunlock(bus->dhd);
		dhd_rx_frame(bus->dhd, ifidx, pkt, 1);
		dhd_os_sdlock(bus->dhd);
	}
	rxcount = maxframes - rxleft;
#ifdef DHD_DEBUG
	/* Message if we hit the limit */
	if (!rxleft && !sdtest)
		DHD_DATA(("%s: hit rx limit of %d frames\n", __func__,
			  maxframes));
	else
#endif				/* DHD_DEBUG */
		DHD_DATA(("%s: processed %d frames\n", __func__, rxcount));
=======
		brcmf_sdbrcm_sdunlock(bus);
		brcmf_rx_frame(bus->drvr, ifidx, pkt, 1);
		brcmf_sdbrcm_sdlock(bus);
	}
	rxcount = maxframes - rxleft;
#ifdef BCMDBG
	/* Message if we hit the limit */
	if (!rxleft && !sdtest)
		BRCMF_DATA(("%s: hit rx limit of %d frames\n", __func__,
			    maxframes));
	else
#endif				/* BCMDBG */
		BRCMF_DATA(("%s: processed %d frames\n", __func__, rxcount));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	/* Back off rxseq if awaiting rtx, update rx_seq */
	if (bus->rxskip)
		rxseq--;
	bus->rx_seq = rxseq;

	return rxcount;
}

<<<<<<< HEAD
static u32 dhdsdio_hostmail(dhd_bus_t *bus)
{
	sdpcmd_regs_t *regs = bus->regs;
=======
static u32 brcmf_sdbrcm_hostmail(struct brcmf_bus *bus)
{
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	u32 intstatus = 0;
	u32 hmb_data;
	u8 fcbits;
	uint retries = 0;

<<<<<<< HEAD
	DHD_TRACE(("%s: Enter\n", __func__));

	/* Read mailbox data and ack that we did so */
	R_SDREG(hmb_data, &regs->tohostmailboxdata, retries);
	if (retries <= retry_limit)
		W_SDREG(SMB_INT_ACK, &regs->tosbmailbox, retries);
=======
	BRCMF_TRACE(("%s: Enter\n", __func__));

	/* Read mailbox data and ack that we did so */
	r_sdreg32(bus, &hmb_data,
		  offsetof(struct sdpcmd_regs, tohostmailboxdata), &retries);

	if (retries <= retry_limit)
		w_sdreg32(bus, SMB_INT_ACK,
			  offsetof(struct sdpcmd_regs, tosbmailbox), &retries);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	bus->f1regdata += 2;

	/* Dongle recomposed rx frames, accept them again */
	if (hmb_data & HMB_DATA_NAKHANDLED) {
<<<<<<< HEAD
		DHD_INFO(("Dongle reports NAK handled, expect rtx of %d\n",
			  bus->rx_seq));
		if (!bus->rxskip)
			DHD_ERROR(("%s: unexpected NAKHANDLED!\n", __func__));
=======
		BRCMF_INFO(("Dongle reports NAK handled, expect rtx of %d\n",
			    bus->rx_seq));
		if (!bus->rxskip)
			BRCMF_ERROR(("%s: unexpected NAKHANDLED!\n", __func__));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

		bus->rxskip = false;
		intstatus |= I_HMB_FRAME_IND;
	}

	/*
	 * DEVREADY does not occur with gSPI.
	 */
	if (hmb_data & (HMB_DATA_DEVREADY | HMB_DATA_FWREADY)) {
		bus->sdpcm_ver =
		    (hmb_data & HMB_DATA_VERSION_MASK) >>
		    HMB_DATA_VERSION_SHIFT;
		if (bus->sdpcm_ver != SDPCM_PROT_VERSION)
<<<<<<< HEAD
			DHD_ERROR(("Version mismatch, dongle reports %d, "
				"expecting %d\n",
				bus->sdpcm_ver, SDPCM_PROT_VERSION));
		else
			DHD_INFO(("Dongle ready, protocol version %d\n",
				  bus->sdpcm_ver));
=======
			BRCMF_ERROR(("Version mismatch, dongle reports %d, "
				     "expecting %d\n",
				     bus->sdpcm_ver, SDPCM_PROT_VERSION));
		else
			BRCMF_INFO(("Dongle ready, protocol version %d\n",
				    bus->sdpcm_ver));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	}

	/*
	 * Flow Control has been moved into the RX headers and this out of band
	 * method isn't used any more.
	 * remaining backward compatible with older dongles.
	 */
	if (hmb_data & HMB_DATA_FC) {
		fcbits = (hmb_data & HMB_DATA_FCDATA_MASK) >>
							HMB_DATA_FCDATA_SHIFT;

		if (fcbits & ~bus->flowcontrol)
			bus->fc_xoff++;

		if (bus->flowcontrol & ~fcbits)
			bus->fc_xon++;

		bus->fc_rcvd++;
		bus->flowcontrol = fcbits;
	}

	/* Shouldn't be any others */
	if (hmb_data & ~(HMB_DATA_DEVREADY |
			 HMB_DATA_NAKHANDLED |
			 HMB_DATA_FC |
			 HMB_DATA_FWREADY |
			 HMB_DATA_FCDATA_MASK | HMB_DATA_VERSION_MASK)) {
<<<<<<< HEAD
		DHD_ERROR(("Unknown mailbox data content: 0x%02x\n", hmb_data));
=======
		BRCMF_ERROR(("Unknown mailbox data content: 0x%02x\n",
			     hmb_data));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	}

	return intstatus;
}

<<<<<<< HEAD
bool dhdsdio_dpc(dhd_bus_t *bus)
{
	bcmsdh_info_t *sdh = bus->sdh;
	sdpcmd_regs_t *regs = bus->regs;
	u32 intstatus, newstatus = 0;
	uint retries = 0;
	uint rxlimit = dhd_rxbound;	/* Rx frames to read before resched */
	uint txlimit = dhd_txbound;	/* Tx frames to send before resched */
=======
static bool brcmf_sdbrcm_dpc(struct brcmf_bus *bus)
{
	struct brcmf_sdio_card *card = bus->card;
	u32 intstatus, newstatus = 0;
	uint retries = 0;
	uint rxlimit = brcmf_rxbound;	/* Rx frames to read before resched */
	uint txlimit = brcmf_txbound;	/* Tx frames to send before resched */
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	uint framecnt = 0;	/* Temporary counter of tx/rx frames */
	bool rxdone = true;	/* Flag for no more read data */
	bool resched = false;	/* Flag indicating resched wanted */

<<<<<<< HEAD
	DHD_TRACE(("%s: Enter\n", __func__));
=======
	BRCMF_TRACE(("%s: Enter\n", __func__));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	/* Start with leftover status bits */
	intstatus = bus->intstatus;

<<<<<<< HEAD
	dhd_os_sdlock(bus->dhd);
=======
	brcmf_sdbrcm_sdlock(bus);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	/* If waiting for HTAVAIL, check status */
	if (bus->clkstate == CLK_PENDING) {
		int err;
		u8 clkctl, devctl = 0;

<<<<<<< HEAD
#ifdef DHD_DEBUG
		/* Check for inconsistent device control */
		devctl =
		    bcmsdh_cfg_read(sdh, SDIO_FUNC_1, SBSDIO_DEVICE_CTL, &err);
		if (err) {
			DHD_ERROR(("%s: error reading DEVCTL: %d\n",
				   __func__, err));
			bus->dhd->busstate = DHD_BUS_DOWN;
		} else {
			ASSERT(devctl & SBSDIO_DEVCTL_CA_INT_ONLY);
		}
#endif				/* DHD_DEBUG */

		/* Read CSR, if clock on switch to AVAIL, else ignore */
		clkctl =
		    bcmsdh_cfg_read(sdh, SDIO_FUNC_1, SBSDIO_FUNC1_CHIPCLKCSR,
				    &err);
		if (err) {
			DHD_ERROR(("%s: error reading CSR: %d\n", __func__,
				   err));
			bus->dhd->busstate = DHD_BUS_DOWN;
		}

		DHD_INFO(("DPC: PENDING, devctl 0x%02x clkctl 0x%02x\n", devctl,
			  clkctl));

		if (SBSDIO_HTAV(clkctl)) {
			devctl =
			    bcmsdh_cfg_read(sdh, SDIO_FUNC_1, SBSDIO_DEVICE_CTL,
					    &err);
			if (err) {
				DHD_ERROR(("%s: error reading DEVCTL: %d\n",
					   __func__, err));
				bus->dhd->busstate = DHD_BUS_DOWN;
			}
			devctl &= ~SBSDIO_DEVCTL_CA_INT_ONLY;
			bcmsdh_cfg_write(sdh, SDIO_FUNC_1, SBSDIO_DEVICE_CTL,
					 devctl, &err);
			if (err) {
				DHD_ERROR(("%s: error writing DEVCTL: %d\n",
					   __func__, err));
				bus->dhd->busstate = DHD_BUS_DOWN;
=======
#ifdef BCMDBG
		/* Check for inconsistent device control */
		devctl = brcmf_sdcard_cfg_read(card, SDIO_FUNC_1,
					       SBSDIO_DEVICE_CTL, &err);
		if (err) {
			BRCMF_ERROR(("%s: error reading DEVCTL: %d\n",
				     __func__, err));
			bus->drvr->busstate = BRCMF_BUS_DOWN;
		}
#endif				/* BCMDBG */

		/* Read CSR, if clock on switch to AVAIL, else ignore */
		clkctl = brcmf_sdcard_cfg_read(card, SDIO_FUNC_1,
					       SBSDIO_FUNC1_CHIPCLKCSR, &err);
		if (err) {
			BRCMF_ERROR(("%s: error reading CSR: %d\n", __func__,
				     err));
			bus->drvr->busstate = BRCMF_BUS_DOWN;
		}

		BRCMF_INFO(("DPC: PENDING, devctl 0x%02x clkctl 0x%02x\n",
			    devctl, clkctl));

		if (SBSDIO_HTAV(clkctl)) {
			devctl = brcmf_sdcard_cfg_read(card, SDIO_FUNC_1,
						       SBSDIO_DEVICE_CTL, &err);
			if (err) {
				BRCMF_ERROR(("%s: error reading DEVCTL: %d\n",
					     __func__, err));
				bus->drvr->busstate = BRCMF_BUS_DOWN;
			}
			devctl &= ~SBSDIO_DEVCTL_CA_INT_ONLY;
			brcmf_sdcard_cfg_write(card, SDIO_FUNC_1,
				SBSDIO_DEVICE_CTL, devctl, &err);
			if (err) {
				BRCMF_ERROR(("%s: error writing DEVCTL: %d\n",
					     __func__, err));
				bus->drvr->busstate = BRCMF_BUS_DOWN;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			}
			bus->clkstate = CLK_AVAIL;
		} else {
			goto clkwait;
		}
	}

	BUS_WAKE(bus);

	/* Make sure backplane clock is on */
<<<<<<< HEAD
	dhdsdio_clkctl(bus, CLK_AVAIL, true);
=======
	brcmf_sdbrcm_clkctl(bus, CLK_AVAIL, true);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	if (bus->clkstate == CLK_PENDING)
		goto clkwait;

	/* Pending interrupt indicates new device status */
	if (bus->ipend) {
		bus->ipend = false;
<<<<<<< HEAD
		R_SDREG(newstatus, &regs->intstatus, retries);
		bus->f1regdata++;
		if (bcmsdh_regfail(bus->sdh))
=======
		r_sdreg32(bus, &newstatus,
			  offsetof(struct sdpcmd_regs, intstatus), &retries);
		bus->f1regdata++;
		if (brcmf_sdcard_regfail(bus->card))
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			newstatus = 0;
		newstatus &= bus->hostintmask;
		bus->fcstate = !!(newstatus & I_HMB_FC_STATE);
		if (newstatus) {
<<<<<<< HEAD
			W_SDREG(newstatus, &regs->intstatus, retries);
=======
			w_sdreg32(bus, newstatus,
				  offsetof(struct sdpcmd_regs, intstatus),
				  &retries);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			bus->f1regdata++;
		}
	}

	/* Merge new bits with previous */
	intstatus |= newstatus;
	bus->intstatus = 0;

	/* Handle flow-control change: read new state in case our ack
	 * crossed another change interrupt.  If change still set, assume
	 * FC ON for safety, let next loop through do the debounce.
	 */
	if (intstatus & I_HMB_FC_CHANGE) {
		intstatus &= ~I_HMB_FC_CHANGE;
<<<<<<< HEAD
		W_SDREG(I_HMB_FC_CHANGE, &regs->intstatus, retries);
		R_SDREG(newstatus, &regs->intstatus, retries);
=======
		w_sdreg32(bus, I_HMB_FC_CHANGE,
			  offsetof(struct sdpcmd_regs, intstatus), &retries);

		r_sdreg32(bus, &newstatus,
			  offsetof(struct sdpcmd_regs, intstatus), &retries);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		bus->f1regdata += 2;
		bus->fcstate =
		    !!(newstatus & (I_HMB_FC_STATE | I_HMB_FC_CHANGE));
		intstatus |= (newstatus & bus->hostintmask);
	}

	/* Handle host mailbox indication */
	if (intstatus & I_HMB_HOST_INT) {
		intstatus &= ~I_HMB_HOST_INT;
<<<<<<< HEAD
		intstatus |= dhdsdio_hostmail(bus);
=======
		intstatus |= brcmf_sdbrcm_hostmail(bus);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	}

	/* Generally don't ask for these, can get CRC errors... */
	if (intstatus & I_WR_OOSYNC) {
<<<<<<< HEAD
		DHD_ERROR(("Dongle reports WR_OOSYNC\n"));
=======
		BRCMF_ERROR(("Dongle reports WR_OOSYNC\n"));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		intstatus &= ~I_WR_OOSYNC;
	}

	if (intstatus & I_RD_OOSYNC) {
<<<<<<< HEAD
		DHD_ERROR(("Dongle reports RD_OOSYNC\n"));
=======
		BRCMF_ERROR(("Dongle reports RD_OOSYNC\n"));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		intstatus &= ~I_RD_OOSYNC;
	}

	if (intstatus & I_SBINT) {
<<<<<<< HEAD
		DHD_ERROR(("Dongle reports SBINT\n"));
=======
		BRCMF_ERROR(("Dongle reports SBINT\n"));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		intstatus &= ~I_SBINT;
	}

	/* Would be active due to wake-wlan in gSPI */
	if (intstatus & I_CHIPACTIVE) {
<<<<<<< HEAD
		DHD_INFO(("Dongle reports CHIPACTIVE\n"));
=======
		BRCMF_INFO(("Dongle reports CHIPACTIVE\n"));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		intstatus &= ~I_CHIPACTIVE;
	}

	/* Ignore frame indications if rxskip is set */
	if (bus->rxskip)
		intstatus &= ~I_HMB_FRAME_IND;

	/* On frame indication, read available frames */
	if (PKT_AVAILABLE()) {
<<<<<<< HEAD
		framecnt = dhdsdio_readframes(bus, rxlimit, &rxdone);
=======
		framecnt = brcmf_sdbrcm_readframes(bus, rxlimit, &rxdone);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		if (rxdone || bus->rxskip)
			intstatus &= ~I_HMB_FRAME_IND;
		rxlimit -= min(framecnt, rxlimit);
	}

	/* Keep still-pending events for next scheduling */
	bus->intstatus = intstatus;

clkwait:
<<<<<<< HEAD
#if defined(OOB_INTR_ONLY)
	bcmsdh_oob_intr_set(1);
#endif				/* (OOB_INTR_ONLY) */
=======
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	/* Re-enable interrupts to detect new device events (mailbox, rx frame)
	 * or clock availability.  (Allows tx loop to check ipend if desired.)
	 * (Unless register access seems hosed, as we may not be able to ACK...)
	 */
<<<<<<< HEAD
	if (bus->intr && bus->intdis && !bcmsdh_regfail(sdh)) {
		DHD_INTR(("%s: enable SDIO interrupts, rxdone %d framecnt %d\n",
			  __func__, rxdone, framecnt));
		bus->intdis = false;
		bcmsdh_intr_enable(sdh);
=======
	if (bus->intr && bus->intdis && !brcmf_sdcard_regfail(card)) {
		BRCMF_INTR(("%s: enable SDIO interrupts, rxdone %d"
			    " framecnt %d\n", __func__, rxdone, framecnt));
		bus->intdis = false;
		brcmf_sdcard_intr_enable(card);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	}

	if (DATAOK(bus) && bus->ctrl_frame_stat &&
		(bus->clkstate == CLK_AVAIL)) {
		int ret, i;

<<<<<<< HEAD
		ret =
		    dhd_bcmsdh_send_buf(bus, bcmsdh_cur_sbwad(sdh), SDIO_FUNC_2,
					F2SYNC, (u8 *) bus->ctrl_frame_buf,
					(u32) bus->ctrl_frame_len, NULL,
					NULL, NULL);
		ASSERT(ret != -BCME_PENDING);
=======
		ret = brcmf_sdbrcm_send_buf(bus, brcmf_sdcard_cur_sbwad(card),
			SDIO_FUNC_2, F2SYNC, (u8 *) bus->ctrl_frame_buf,
			(u32) bus->ctrl_frame_len, NULL, NULL, NULL);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

		if (ret < 0) {
			/* On failure, abort the command and
				terminate the frame */
<<<<<<< HEAD
			DHD_INFO(("%s: sdio error %d, abort command and "
				"terminate frame.\n", __func__, ret));
			bus->tx_sderrs++;

			bcmsdh_abort(sdh, SDIO_FUNC_2);

			bcmsdh_cfg_write(sdh, SDIO_FUNC_1,
=======
			BRCMF_INFO(("%s: sdio error %d, abort command and "
				    "terminate frame.\n", __func__, ret));
			bus->tx_sderrs++;

			brcmf_sdcard_abort(card, SDIO_FUNC_2);

			brcmf_sdcard_cfg_write(card, SDIO_FUNC_1,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
					 SBSDIO_FUNC1_FRAMECTRL, SFC_WF_TERM,
					 NULL);
			bus->f1regdata++;

			for (i = 0; i < 3; i++) {
				u8 hi, lo;
<<<<<<< HEAD
				hi = bcmsdh_cfg_read(sdh, SDIO_FUNC_1,
						     SBSDIO_FUNC1_WFRAMEBCHI,
						     NULL);
				lo = bcmsdh_cfg_read(sdh, SDIO_FUNC_1,
=======
				hi = brcmf_sdcard_cfg_read(card, SDIO_FUNC_1,
						     SBSDIO_FUNC1_WFRAMEBCHI,
						     NULL);
				lo = brcmf_sdcard_cfg_read(card, SDIO_FUNC_1,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
						     SBSDIO_FUNC1_WFRAMEBCLO,
						     NULL);
				bus->f1regdata += 2;
				if ((hi == 0) && (lo == 0))
					break;
			}

		}
		if (ret == 0)
			bus->tx_seq = (bus->tx_seq + 1) % SDPCM_SEQUENCE_WRAP;

<<<<<<< HEAD
		DHD_INFO(("Return_dpc value is : %d\n", ret));
		bus->ctrl_frame_stat = false;
		dhd_wait_event_wakeup(bus->dhd);
	}
	/* Send queued frames (limit 1 if rx may still be pending) */
	else if ((bus->clkstate == CLK_AVAIL) && !bus->fcstate &&
		 bcm_pktq_mlen(&bus->txq, ~bus->flowcontrol) && txlimit
		 && DATAOK(bus)) {
		framecnt = rxdone ? txlimit : min(txlimit, dhd_txminmax);
		framecnt = dhdsdio_sendfromq(bus, framecnt);
=======
		BRCMF_INFO(("Return_dpc value is : %d\n", ret));
		bus->ctrl_frame_stat = false;
		brcmf_sdbrcm_wait_event_wakeup(bus);
	}
	/* Send queued frames (limit 1 if rx may still be pending) */
	else if ((bus->clkstate == CLK_AVAIL) && !bus->fcstate &&
		 brcmu_pktq_mlen(&bus->txq, ~bus->flowcontrol) && txlimit
		 && DATAOK(bus)) {
		framecnt = rxdone ? txlimit : min(txlimit, brcmf_txminmax);
		framecnt = brcmf_sdbrcm_sendfromq(bus, framecnt);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		txlimit -= framecnt;
	}

	/* Resched if events or tx frames are pending,
		 else await next interrupt */
	/* On failed register access, all bets are off:
		 no resched or interrupts */
<<<<<<< HEAD
	if ((bus->dhd->busstate == DHD_BUS_DOWN) || bcmsdh_regfail(sdh)) {
		DHD_ERROR(("%s: failed backplane access over SDIO, halting "
			"operation %d\n", __func__, bcmsdh_regfail(sdh)));
		bus->dhd->busstate = DHD_BUS_DOWN;
		bus->intstatus = 0;
	} else if (bus->clkstate == CLK_PENDING) {
		DHD_INFO(("%s: rescheduled due to CLK_PENDING awaiting "
			"I_CHIPACTIVE interrupt\n", __func__));
		resched = true;
	} else if (bus->intstatus || bus->ipend ||
		(!bus->fcstate && bcm_pktq_mlen(&bus->txq, ~bus->flowcontrol) &&
			DATAOK(bus)) || PKT_AVAILABLE()) {
=======
	if ((bus->drvr->busstate == BRCMF_BUS_DOWN) ||
	    brcmf_sdcard_regfail(card)) {
		BRCMF_ERROR(("%s: failed backplane access over SDIO, halting "
			     "operation %d\n", __func__,
			     brcmf_sdcard_regfail(card)));
		bus->drvr->busstate = BRCMF_BUS_DOWN;
		bus->intstatus = 0;
	} else if (bus->clkstate == CLK_PENDING) {
		BRCMF_INFO(("%s: rescheduled due to CLK_PENDING awaiting "
			    "I_CHIPACTIVE interrupt\n", __func__));
		resched = true;
	} else if (bus->intstatus || bus->ipend ||
		(!bus->fcstate && brcmu_pktq_mlen(&bus->txq, ~bus->flowcontrol)
		 && DATAOK(bus)) || PKT_AVAILABLE()) {
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		resched = true;
	}

	bus->dpc_sched = resched;

	/* If we're done for now, turn off clock request. */
	if ((bus->clkstate != CLK_PENDING)
<<<<<<< HEAD
	    && bus->idletime == DHD_IDLE_IMMEDIATE) {
		bus->activity = false;
		dhdsdio_clkctl(bus, CLK_NONE, false);
	}

	dhd_os_sdunlock(bus->dhd);

	return resched;
}

bool dhd_bus_dpc(struct dhd_bus *bus)
{
	bool resched;

	/* Call the DPC directly. */
	DHD_TRACE(("Calling dhdsdio_dpc() from %s\n", __func__));
	resched = dhdsdio_dpc(bus);
=======
	    && bus->idletime == BRCMF_IDLE_IMMEDIATE) {
		bus->activity = false;
		brcmf_sdbrcm_clkctl(bus, CLK_NONE, false);
	}

	brcmf_sdbrcm_sdunlock(bus);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	return resched;
}

<<<<<<< HEAD
void dhdsdio_isr(void *arg)
{
	dhd_bus_t *bus = (dhd_bus_t *) arg;
	bcmsdh_info_t *sdh;

	DHD_TRACE(("%s: Enter\n", __func__));

	if (!bus) {
		DHD_ERROR(("%s : bus is null pointer , exit\n", __func__));
		return;
	}
	sdh = bus->sdh;

	if (bus->dhd->busstate == DHD_BUS_DOWN) {
		DHD_ERROR(("%s : bus is down. we have nothing to do\n",
=======
void brcmf_sdbrcm_isr(void *arg)
{
	struct brcmf_bus *bus = (struct brcmf_bus *) arg;
	struct brcmf_sdio_card *card;

	BRCMF_TRACE(("%s: Enter\n", __func__));

	if (!bus) {
		BRCMF_ERROR(("%s : bus is null pointer , exit\n", __func__));
		return;
	}
	card = bus->card;

	if (bus->drvr->busstate == BRCMF_BUS_DOWN) {
		BRCMF_ERROR(("%s : bus is down. we have nothing to do\n",
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			   __func__));
		return;
	}
	/* Count the interrupt call */
	bus->intrcount++;
	bus->ipend = true;

	/* Shouldn't get this interrupt if we're sleeping? */
	if (bus->sleeping) {
<<<<<<< HEAD
		DHD_ERROR(("INTERRUPT WHILE SLEEPING??\n"));
=======
		BRCMF_ERROR(("INTERRUPT WHILE SLEEPING??\n"));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		return;
	}

	/* Disable additional interrupts (is this needed now)? */
	if (bus->intr)
<<<<<<< HEAD
		DHD_INTR(("%s: disable SDIO interrupts\n", __func__));
	else
		DHD_ERROR(("dhdsdio_isr() w/o interrupt configured!\n"));

	bcmsdh_intr_disable(sdh);
	bus->intdis = true;

#if defined(SDIO_ISR_THREAD)
	DHD_TRACE(("Calling dhdsdio_dpc() from %s\n", __func__));
	while (dhdsdio_dpc(bus))
		;
#else
	bus->dpc_sched = true;
	dhd_sched_dpc(bus->dhd);
=======
		BRCMF_INTR(("%s: disable SDIO interrupts\n", __func__));
	else
		BRCMF_ERROR(("brcmf_sdbrcm_isr() w/o interrupt configured!\n"));

	brcmf_sdcard_intr_disable(card);
	bus->intdis = true;

#if defined(SDIO_ISR_THREAD)
	BRCMF_TRACE(("Calling brcmf_sdbrcm_dpc() from %s\n", __func__));
	while (brcmf_sdbrcm_dpc(bus))
		;
#else
	bus->dpc_sched = true;
	brcmf_sdbrcm_sched_dpc(bus);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
#endif

}

#ifdef SDTEST
<<<<<<< HEAD
static void dhdsdio_pktgen_init(dhd_bus_t *bus)
{
	/* Default to specified length, or full range */
	if (dhd_pktgen_len) {
		bus->pktgen_maxlen = min(dhd_pktgen_len, MAX_PKTGEN_LEN);
		bus->pktgen_minlen = bus->pktgen_maxlen;
	} else {
		bus->pktgen_maxlen = MAX_PKTGEN_LEN;
=======
static void brcmf_sdbrcm_pktgen_init(struct brcmf_bus *bus)
{
	/* Default to specified length, or full range */
	if (brcmf_pktgen_len) {
		bus->pktgen_maxlen = min(brcmf_pktgen_len,
					 BRCMF_MAX_PKTGEN_LEN);
		bus->pktgen_minlen = bus->pktgen_maxlen;
	} else {
		bus->pktgen_maxlen = BRCMF_MAX_PKTGEN_LEN;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		bus->pktgen_minlen = 0;
	}
	bus->pktgen_len = (u16) bus->pktgen_minlen;

	/* Default to per-watchdog burst with 10s print time */
	bus->pktgen_freq = 1;
<<<<<<< HEAD
	bus->pktgen_print = 10000 / dhd_watchdog_ms;
	bus->pktgen_count = (dhd_pktgen * dhd_watchdog_ms + 999) / 1000;

	/* Default to echo mode */
	bus->pktgen_mode = DHD_PKTGEN_ECHO;
	bus->pktgen_stop = 1;
}

static void dhdsdio_pktgen(dhd_bus_t *bus)
=======
	bus->pktgen_print = 10000 / brcmf_watchdog_ms;
	bus->pktgen_count = (brcmf_pktgen * brcmf_watchdog_ms + 999) / 1000;

	/* Default to echo mode */
	bus->pktgen_mode = BRCMF_PKTGEN_ECHO;
	bus->pktgen_stop = 1;
}

static void brcmf_sdbrcm_pktgen(struct brcmf_bus *bus)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
{
	struct sk_buff *pkt;
	u8 *data;
	uint pktcount;
	uint fillbyte;
	u16 len;

	/* Display current count if appropriate */
	if (bus->pktgen_print && (++bus->pktgen_ptick >= bus->pktgen_print)) {
		bus->pktgen_ptick = 0;
		printk(KERN_DEBUG "%s: send attempts %d rcvd %d\n",
		       __func__, bus->pktgen_sent, bus->pktgen_rcvd);
	}

	/* For recv mode, just make sure dongle has started sending */
<<<<<<< HEAD
	if (bus->pktgen_mode == DHD_PKTGEN_RECV) {
		if (!bus->pktgen_rcvd)
			dhdsdio_sdtest_set(bus, true);
=======
	if (bus->pktgen_mode == BRCMF_PKTGEN_RECV) {
		if (!bus->pktgen_rcvd)
			brcmf_sdbrcm_sdtest_set(bus, true);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		return;
	}

	/* Otherwise, generate or request the specified number of packets */
	for (pktcount = 0; pktcount < bus->pktgen_count; pktcount++) {
		/* Stop if total has been reached */
		if (bus->pktgen_total
		    && (bus->pktgen_sent >= bus->pktgen_total)) {
			bus->pktgen_count = 0;
			break;
		}

		/* Allocate an appropriate-sized packet */
		len = bus->pktgen_len;
<<<<<<< HEAD
		pkt = bcm_pkt_buf_get_skb(
			(len + SDPCM_HDRLEN + SDPCM_TEST_HDRLEN + DHD_SDALIGN),
			true);
		if (!pkt) {
			DHD_ERROR(("%s: bcm_pkt_buf_get_skb failed!\n",
				__func__));
			break;
		}
		PKTALIGN(pkt, (len + SDPCM_HDRLEN + SDPCM_TEST_HDRLEN),
			 DHD_SDALIGN);
=======
		pkt = brcmu_pkt_buf_get_skb(
			len + SDPCM_HDRLEN + SDPCM_TEST_HDRLEN + BRCMF_SDALIGN,
			true);
		if (!pkt) {
			BRCMF_ERROR(("%s: brcmu_pkt_buf_get_skb failed!\n",
				     __func__));
			break;
		}
		PKTALIGN(pkt, (len + SDPCM_HDRLEN + SDPCM_TEST_HDRLEN),
			 BRCMF_SDALIGN);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		data = (u8 *) (pkt->data) + SDPCM_HDRLEN;

		/* Write test header cmd and extra based on mode */
		switch (bus->pktgen_mode) {
<<<<<<< HEAD
		case DHD_PKTGEN_ECHO:
=======
		case BRCMF_PKTGEN_ECHO:
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			*data++ = SDPCM_TEST_ECHOREQ;
			*data++ = (u8) bus->pktgen_sent;
			break;

<<<<<<< HEAD
		case DHD_PKTGEN_SEND:
=======
		case BRCMF_PKTGEN_SEND:
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			*data++ = SDPCM_TEST_DISCARD;
			*data++ = (u8) bus->pktgen_sent;
			break;

<<<<<<< HEAD
		case DHD_PKTGEN_RXBURST:
=======
		case BRCMF_PKTGEN_RXBURST:
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			*data++ = SDPCM_TEST_BURST;
			*data++ = (u8) bus->pktgen_count;
			break;

		default:
<<<<<<< HEAD
			DHD_ERROR(("Unrecognized pktgen mode %d\n",
				   bus->pktgen_mode));
			bcm_pkt_buf_free_skb(pkt, true);
=======
			BRCMF_ERROR(("Unrecognized pktgen mode %d\n",
				     bus->pktgen_mode));
			brcmu_pkt_buf_free_skb(pkt, true);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			bus->pktgen_count = 0;
			return;
		}

		/* Write test header length field */
		*data++ = (len >> 0);
		*data++ = (len >> 8);

		/* Then fill in the remainder -- N/A for burst,
			 but who cares... */
		for (fillbyte = 0; fillbyte < len; fillbyte++)
			*data++ =
			    SDPCM_TEST_FILL(fillbyte, (u8) bus->pktgen_sent);

<<<<<<< HEAD
#ifdef DHD_DEBUG
		if (DHD_BYTES_ON() && DHD_DATA_ON()) {
			data = (u8 *) (pkt->data) + SDPCM_HDRLEN;
			printk(KERN_DEBUG "dhdsdio_pktgen: Tx Data:\n");
=======
#ifdef BCMDBG
		if (BRCMF_BYTES_ON() && BRCMF_DATA_ON()) {
			data = (u8 *) (pkt->data) + SDPCM_HDRLEN;
			printk(KERN_DEBUG "brcmf_sdbrcm_pktgen: Tx Data:\n");
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, data,
					     pkt->len - SDPCM_HDRLEN);
		}
#endif

		/* Send it */
<<<<<<< HEAD
		if (dhdsdio_txpkt(bus, pkt, SDPCM_TEST_CHANNEL, true)) {
=======
		if (brcmf_sdbrcm_txpkt(bus, pkt, SDPCM_TEST_CHANNEL, true)) {
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			bus->pktgen_fail++;
			if (bus->pktgen_stop
			    && bus->pktgen_stop == bus->pktgen_fail)
				bus->pktgen_count = 0;
		}
		bus->pktgen_sent++;

		/* Bump length if not fixed, wrap at max */
		if (++bus->pktgen_len > bus->pktgen_maxlen)
			bus->pktgen_len = (u16) bus->pktgen_minlen;

		/* Special case for burst mode: just send one request! */
<<<<<<< HEAD
		if (bus->pktgen_mode == DHD_PKTGEN_RXBURST)
=======
		if (bus->pktgen_mode == BRCMF_PKTGEN_RXBURST)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			break;
	}
}

<<<<<<< HEAD
static void dhdsdio_sdtest_set(dhd_bus_t *bus, bool start)
=======
static void brcmf_sdbrcm_sdtest_set(struct brcmf_bus *bus, bool start)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
{
	struct sk_buff *pkt;
	u8 *data;

	/* Allocate the packet */
<<<<<<< HEAD
	pkt = bcm_pkt_buf_get_skb(SDPCM_HDRLEN + SDPCM_TEST_HDRLEN +
		DHD_SDALIGN, true);
	if (!pkt) {
		DHD_ERROR(("%s: bcm_pkt_buf_get_skb failed!\n", __func__));
		return;
	}
	PKTALIGN(pkt, (SDPCM_HDRLEN + SDPCM_TEST_HDRLEN), DHD_SDALIGN);
=======
	pkt = brcmu_pkt_buf_get_skb(SDPCM_HDRLEN + SDPCM_TEST_HDRLEN +
		BRCMF_SDALIGN, true);
	if (!pkt) {
		BRCMF_ERROR(("%s: brcmu_pkt_buf_get_skb failed!\n", __func__));
		return;
	}
	PKTALIGN(pkt, (SDPCM_HDRLEN + SDPCM_TEST_HDRLEN), BRCMF_SDALIGN);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	data = (u8 *) (pkt->data) + SDPCM_HDRLEN;

	/* Fill in the test header */
	*data++ = SDPCM_TEST_SEND;
	*data++ = start;
	*data++ = (bus->pktgen_maxlen >> 0);
	*data++ = (bus->pktgen_maxlen >> 8);

	/* Send it */
<<<<<<< HEAD
	if (dhdsdio_txpkt(bus, pkt, SDPCM_TEST_CHANNEL, true))
		bus->pktgen_fail++;
}

static void dhdsdio_testrcv(dhd_bus_t *bus, struct sk_buff *pkt, uint seq)
=======
	if (brcmf_sdbrcm_txpkt(bus, pkt, SDPCM_TEST_CHANNEL, true))
		bus->pktgen_fail++;
}

static void
brcmf_sdbrcm_checkdied(struct brcmf_bus *bus, struct sk_buff *pkt, uint seq)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
{
	u8 *data;
	uint pktlen;

	u8 cmd;
	u8 extra;
	u16 len;
	u16 offset;

	/* Check for min length */
	pktlen = pkt->len;
	if (pktlen < SDPCM_TEST_HDRLEN) {
<<<<<<< HEAD
		DHD_ERROR(("dhdsdio_restrcv: toss runt frame, pktlen %d\n",
			   pktlen));
		bcm_pkt_buf_free_skb(pkt, false);
=======
		BRCMF_ERROR(("brcmf_sdbrcm_checkdied: toss runt frame, pktlen "
			     "%d\n", pktlen));
		brcmu_pkt_buf_free_skb(pkt, false);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		return;
	}

	/* Extract header fields */
	data = pkt->data;
	cmd = *data++;
	extra = *data++;
	len = *data++;
	len += *data++ << 8;

	/* Check length for relevant commands */
	if (cmd == SDPCM_TEST_DISCARD || cmd == SDPCM_TEST_ECHOREQ
	    || cmd == SDPCM_TEST_ECHORSP) {
		if (pktlen != len + SDPCM_TEST_HDRLEN) {
<<<<<<< HEAD
			DHD_ERROR(("dhdsdio_testrcv: frame length mismatch, "
				"pktlen %d seq %d" " cmd %d extra %d len %d\n",
				pktlen, seq, cmd, extra, len));
			bcm_pkt_buf_free_skb(pkt, false);
=======
			BRCMF_ERROR(("brcmf_sdbrcm_checkdied: frame length "
				     "mismatch, pktlen %d seq %d"
				     " cmd %d extra %d len %d\n",
				     pktlen, seq, cmd, extra, len));
			brcmu_pkt_buf_free_skb(pkt, false);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			return;
		}
	}

	/* Process as per command */
	switch (cmd) {
	case SDPCM_TEST_ECHOREQ:
		/* Rx->Tx turnaround ok (even on NDIS w/current
			 implementation) */
		*(u8 *) (pkt->data) = SDPCM_TEST_ECHORSP;
<<<<<<< HEAD
		if (dhdsdio_txpkt(bus, pkt, SDPCM_TEST_CHANNEL, true) == 0) {
			bus->pktgen_sent++;
		} else {
			bus->pktgen_fail++;
			bcm_pkt_buf_free_skb(pkt, false);
=======
		if (brcmf_sdbrcm_txpkt(bus, pkt, SDPCM_TEST_CHANNEL, true) == 0)
			bus->pktgen_sent++;
		else {
			bus->pktgen_fail++;
			brcmu_pkt_buf_free_skb(pkt, false);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		}
		bus->pktgen_rcvd++;
		break;

	case SDPCM_TEST_ECHORSP:
		if (bus->ext_loop) {
<<<<<<< HEAD
			bcm_pkt_buf_free_skb(pkt, false);
=======
			brcmu_pkt_buf_free_skb(pkt, false);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			bus->pktgen_rcvd++;
			break;
		}

		for (offset = 0; offset < len; offset++, data++) {
			if (*data != SDPCM_TEST_FILL(offset, extra)) {
<<<<<<< HEAD
				DHD_ERROR(("dhdsdio_testrcv: echo data mismatch: " "offset %d (len %d) expect 0x%02x rcvd 0x%02x\n",
					offset, len,
					SDPCM_TEST_FILL(offset, extra), *data));
				break;
			}
		}
		bcm_pkt_buf_free_skb(pkt, false);
=======
				BRCMF_ERROR(("brcmf_sdbrcm_checkdied: echo"
					     " data mismatch: "
					     "offset %d (len %d) "
					     "expect 0x%02x rcvd 0x%02x\n",
					     offset, len,
					     SDPCM_TEST_FILL(offset, extra),
					     *data));
				break;
			}
		}
		brcmu_pkt_buf_free_skb(pkt, false);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		bus->pktgen_rcvd++;
		break;

	case SDPCM_TEST_DISCARD:
<<<<<<< HEAD
		bcm_pkt_buf_free_skb(pkt, false);
=======
		brcmu_pkt_buf_free_skb(pkt, false);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		bus->pktgen_rcvd++;
		break;

	case SDPCM_TEST_BURST:
	case SDPCM_TEST_SEND:
	default:
<<<<<<< HEAD
		DHD_INFO(("dhdsdio_testrcv: unsupported or unknown command, "
			"pktlen %d seq %d" " cmd %d extra %d len %d\n",
			pktlen, seq, cmd, extra, len));
		bcm_pkt_buf_free_skb(pkt, false);
=======
		BRCMF_INFO(("brcmf_sdbrcm_checkdied: unsupported or unknown "
			    "command, pktlen %d seq %d" " cmd %d extra %d"
			    " len %d\n", pktlen, seq, cmd, extra, len));
		brcmu_pkt_buf_free_skb(pkt, false);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		break;
	}

	/* For recv mode, stop at limie (and tell dongle to stop sending) */
<<<<<<< HEAD
	if (bus->pktgen_mode == DHD_PKTGEN_RECV) {
		if (bus->pktgen_total
		    && (bus->pktgen_rcvd >= bus->pktgen_total)) {
			bus->pktgen_count = 0;
			dhdsdio_sdtest_set(bus, false);
=======
	if (bus->pktgen_mode == BRCMF_PKTGEN_RECV) {
		if (bus->pktgen_total
		    && (bus->pktgen_rcvd >= bus->pktgen_total)) {
			bus->pktgen_count = 0;
			brcmf_sdbrcm_sdtest_set(bus, false);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		}
	}
}
#endif				/* SDTEST */

<<<<<<< HEAD
extern bool dhd_bus_watchdog(dhd_pub_t *dhdp)
{
	dhd_bus_t *bus;

	DHD_TIMER(("%s: Enter\n", __func__));

	bus = dhdp->bus;

	if (bus->dhd->dongle_reset)
=======
extern bool brcmf_sdbrcm_bus_watchdog(struct brcmf_pub *drvr)
{
	struct brcmf_bus *bus;

	BRCMF_TIMER(("%s: Enter\n", __func__));

	bus = drvr->bus;

	if (bus->drvr->dongle_reset)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		return false;

	/* Ignore the timer if simulating bus down */
	if (bus->sleeping)
		return false;

<<<<<<< HEAD
	dhd_os_sdlock(bus->dhd);
=======
	brcmf_sdbrcm_sdlock(bus);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	/* Poll period: check device if appropriate. */
	if (bus->poll && (++bus->polltick >= bus->pollrate)) {
		u32 intstatus = 0;

		/* Reset poll tick */
		bus->polltick = 0;

		/* Check device if no interrupts */
		if (!bus->intr || (bus->intrcount == bus->lastintrs)) {

			if (!bus->dpc_sched) {
				u8 devpend;
<<<<<<< HEAD
				devpend = bcmsdh_cfg_read(bus->sdh, SDIO_FUNC_0,
							  SDIOD_CCCR_INTPEND,
							  NULL);
=======
				devpend = brcmf_sdcard_cfg_read(bus->card,
						SDIO_FUNC_0, SDIO_CCCR_INTx,
						NULL);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				intstatus =
				    devpend & (INTR_STATUS_FUNC1 |
					       INTR_STATUS_FUNC2);
			}

			/* If there is something, make like the ISR and
				 schedule the DPC */
			if (intstatus) {
				bus->pollcnt++;
				bus->ipend = true;
				if (bus->intr)
<<<<<<< HEAD
					bcmsdh_intr_disable(bus->sdh);

				bus->dpc_sched = true;
				dhd_sched_dpc(bus->dhd);
=======
					brcmf_sdcard_intr_disable(bus->card);

				bus->dpc_sched = true;
				brcmf_sdbrcm_sched_dpc(bus);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

			}
		}

		/* Update interrupt tracking */
		bus->lastintrs = bus->intrcount;
	}
<<<<<<< HEAD
#ifdef DHD_DEBUG
	/* Poll for console output periodically */
	if (dhdp->busstate == DHD_BUS_DATA && dhd_console_ms != 0) {
		bus->console.count += dhd_watchdog_ms;
		if (bus->console.count >= dhd_console_ms) {
			bus->console.count -= dhd_console_ms;
			/* Make sure backplane clock is on */
			dhdsdio_clkctl(bus, CLK_AVAIL, false);
			if (dhdsdio_readconsole(bus) < 0)
				dhd_console_ms = 0;	/* On error,
							 stop trying */
		}
	}
#endif				/* DHD_DEBUG */
=======
#ifdef BCMDBG
	/* Poll for console output periodically */
	if (drvr->busstate == BRCMF_BUS_DATA && brcmf_console_ms != 0) {
		bus->console.count += brcmf_watchdog_ms;
		if (bus->console.count >= brcmf_console_ms) {
			bus->console.count -= brcmf_console_ms;
			/* Make sure backplane clock is on */
			brcmf_sdbrcm_clkctl(bus, CLK_AVAIL, false);
			if (brcmf_sdbrcm_readconsole(bus) < 0)
				brcmf_console_ms = 0;	/* On error,
							 stop trying */
		}
	}
#endif				/* BCMDBG */
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

#ifdef SDTEST
	/* Generate packets if configured */
	if (bus->pktgen_count && (++bus->pktgen_tick >= bus->pktgen_freq)) {
		/* Make sure backplane clock is on */
<<<<<<< HEAD
		dhdsdio_clkctl(bus, CLK_AVAIL, false);
		bus->pktgen_tick = 0;
		dhdsdio_pktgen(bus);
=======
		brcmf_sdbrcm_clkctl(bus, CLK_AVAIL, false);
		bus->pktgen_tick = 0;
		brcmf_sdbrcm_pktgen(bus);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	}
#endif

	/* On idle timeout clear activity flag and/or turn off clock */
	if ((bus->idletime > 0) && (bus->clkstate == CLK_AVAIL)) {
		if (++bus->idlecount >= bus->idletime) {
			bus->idlecount = 0;
			if (bus->activity) {
				bus->activity = false;
<<<<<<< HEAD
				dhd_os_wd_timer(bus->dhd, dhd_watchdog_ms);
			} else {
				dhdsdio_clkctl(bus, CLK_NONE, false);
=======
				brcmf_sdbrcm_wd_timer(bus, brcmf_watchdog_ms);
			} else {
				brcmf_sdbrcm_clkctl(bus, CLK_NONE, false);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			}
		}
	}

<<<<<<< HEAD
	dhd_os_sdunlock(bus->dhd);
=======
	brcmf_sdbrcm_sdunlock(bus);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	return bus->ipend;
}

<<<<<<< HEAD
#ifdef DHD_DEBUG
extern int dhd_bus_console_in(dhd_pub_t *dhdp, unsigned char *msg, uint msglen)
{
	dhd_bus_t *bus = dhdp->bus;
=======
#ifdef BCMDBG
static int brcmf_sdbrcm_bus_console_in(struct brcmf_pub *drvr,
				       unsigned char *msg, uint msglen)
{
	struct brcmf_bus *bus = drvr->bus;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	u32 addr, val;
	int rv;
	struct sk_buff *pkt;

	/* Address could be zero if CONSOLE := 0 in dongle Makefile */
	if (bus->console_addr == 0)
		return -ENOTSUPP;

	/* Exclusive bus access */
<<<<<<< HEAD
	dhd_os_sdlock(bus->dhd);

	/* Don't allow input if dongle is in reset */
	if (bus->dhd->dongle_reset) {
		dhd_os_sdunlock(bus->dhd);
=======
	brcmf_sdbrcm_sdlock(bus);

	/* Don't allow input if dongle is in reset */
	if (bus->drvr->dongle_reset) {
		brcmf_sdbrcm_sdunlock(bus);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		return -EPERM;
	}

	/* Request clock to allow SDIO accesses */
	BUS_WAKE(bus);
	/* No pend allowed since txpkt is called later, ht clk has to be on */
<<<<<<< HEAD
	dhdsdio_clkctl(bus, CLK_AVAIL, false);

	/* Zero cbuf_index */
	addr = bus->console_addr + offsetof(hndrte_cons_t, cbuf_idx);
	val = cpu_to_le32(0);
	rv = dhdsdio_membytes(bus, true, addr, (u8 *)&val, sizeof(val));
=======
	brcmf_sdbrcm_clkctl(bus, CLK_AVAIL, false);

	/* Zero cbuf_index */
	addr = bus->console_addr + offsetof(struct rte_console, cbuf_idx);
	val = cpu_to_le32(0);
	rv = brcmf_sdbrcm_membytes(bus, true, addr, (u8 *)&val, sizeof(val));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	if (rv < 0)
		goto done;

	/* Write message into cbuf */
<<<<<<< HEAD
	addr = bus->console_addr + offsetof(hndrte_cons_t, cbuf);
	rv = dhdsdio_membytes(bus, true, addr, (u8 *)msg, msglen);
=======
	addr = bus->console_addr + offsetof(struct rte_console, cbuf);
	rv = brcmf_sdbrcm_membytes(bus, true, addr, (u8 *)msg, msglen);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	if (rv < 0)
		goto done;

	/* Write length into vcons_in */
<<<<<<< HEAD
	addr = bus->console_addr + offsetof(hndrte_cons_t, vcons_in);
	val = cpu_to_le32(msglen);
	rv = dhdsdio_membytes(bus, true, addr, (u8 *)&val, sizeof(val));
=======
	addr = bus->console_addr + offsetof(struct rte_console, vcons_in);
	val = cpu_to_le32(msglen);
	rv = brcmf_sdbrcm_membytes(bus, true, addr, (u8 *)&val, sizeof(val));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	if (rv < 0)
		goto done;

	/* Bump dongle by sending an empty event pkt.
	 * sdpcm_sendup (RX) checks for virtual console input.
	 */
<<<<<<< HEAD
	pkt = bcm_pkt_buf_get_skb(4 + SDPCM_RESERVE);
	if ((pkt != NULL) && bus->clkstate == CLK_AVAIL)
		dhdsdio_txpkt(bus, pkt, SDPCM_EVENT_CHANNEL, true);

done:
	if ((bus->idletime == DHD_IDLE_IMMEDIATE) && !bus->dpc_sched) {
		bus->activity = false;
		dhdsdio_clkctl(bus, CLK_NONE, true);
	}

	dhd_os_sdunlock(bus->dhd);

	return rv;
}
#endif				/* DHD_DEBUG */

#ifdef DHD_DEBUG
static void dhd_dump_cis(uint fn, u8 *cis)
{
	uint byte, tag, tdata;
	DHD_INFO(("Function %d CIS:\n", fn));

	for (tdata = byte = 0; byte < SBSDIO_CIS_SIZE_LIMIT; byte++) {
		if ((byte % 16) == 0)
			DHD_INFO(("    "));
		DHD_INFO(("%02x ", cis[byte]));
		if ((byte % 16) == 15)
			DHD_INFO(("\n"));
		if (!tdata--) {
			tag = cis[byte];
			if (tag == 0xff)
				break;
			else if (!tag)
				tdata = 0;
			else if ((byte + 1) < SBSDIO_CIS_SIZE_LIMIT)
				tdata = cis[byte + 1] + 1;
			else
				DHD_INFO(("]"));
		}
	}
	if ((byte % 16) != 15)
		DHD_INFO(("\n"));
}
#endif				/* DHD_DEBUG */

static bool dhdsdio_chipmatch(u16 chipid)
=======
	pkt = brcmu_pkt_buf_get_skb(4 + SDPCM_RESERVE);
	if ((pkt != NULL) && bus->clkstate == CLK_AVAIL)
		brcmf_sdbrcm_txpkt(bus, pkt, SDPCM_EVENT_CHANNEL, true);

done:
	if ((bus->idletime == BRCMF_IDLE_IMMEDIATE) && !bus->dpc_sched) {
		bus->activity = false;
		brcmf_sdbrcm_clkctl(bus, CLK_NONE, true);
	}

	brcmf_sdbrcm_sdunlock(bus);

	return rv;
}
#endif				/* BCMDBG */

static bool brcmf_sdbrcm_chipmatch(u16 chipid)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
{
	if (chipid == BCM4325_CHIP_ID)
		return true;
	if (chipid == BCM4329_CHIP_ID)
		return true;
	if (chipid == BCM4319_CHIP_ID)
		return true;
	return false;
}

<<<<<<< HEAD
static void *dhdsdio_probe(u16 venid, u16 devid, u16 bus_no,
			   u16 slot, u16 func, uint bustype, void *regsva,
			   void *sdh)
{
	int ret;
	dhd_bus_t *bus;
=======
static void *brcmf_sdbrcm_probe(u16 venid, u16 devid, u16 bus_no,
			   u16 slot, u16 func, uint bustype, u32 regsva,
			   void *card)
{
	int ret;
	struct brcmf_bus *bus;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	/* Init global variables at run-time, not as part of the declaration.
	 * This is required to support init/de-init of the driver.
	 * Initialization
	 * of globals as part of the declaration results in non-deterministic
	 * behavior since the value of the globals may be different on the
	 * first time that the driver is initialized vs subsequent
	 * initializations.
	 */
<<<<<<< HEAD
	dhd_txbound = DHD_TXBOUND;
	dhd_rxbound = DHD_RXBOUND;
	dhd_alignctl = true;
	sd1idle = true;
	dhd_readahead = true;
	retrydata = false;
	dhd_dongle_memsize = 0;
	dhd_txminmax = DHD_TXMINMAX;

	forcealign = true;

	dhd_common_init();

	DHD_TRACE(("%s: Enter\n", __func__));
	DHD_INFO(("%s: venid 0x%04x devid 0x%04x\n", __func__, venid, devid));

	/* We make assumptions about address window mappings */
	ASSERT((unsigned long)regsva == SI_ENUM_BASE);

	/* BCMSDH passes venid and devid based on CIS parsing -- but
=======
	brcmf_txbound = BRCMF_TXBOUND;
	brcmf_rxbound = BRCMF_RXBOUND;
	brcmf_alignctl = true;
	sd1idle = true;
	brcmf_readahead = true;
	retrydata = false;
	brcmf_dongle_memsize = 0;
	brcmf_txminmax = BRCMF_TXMINMAX;

	forcealign = true;

	brcmf_c_init();

	BRCMF_TRACE(("%s: Enter\n", __func__));
	BRCMF_INFO(("%s: venid 0x%04x devid 0x%04x\n", __func__, venid, devid));

	/* We make an assumption about address window mappings:
	 * regsva == SI_ENUM_BASE*/

	/* SDIO car passes venid and devid based on CIS parsing -- but
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	 * low-power start
	 * means early parse could fail, so here we should get either an ID
	 * we recognize OR (-1) indicating we must request power first.
	 */
	/* Check the Vendor ID */
	switch (venid) {
	case 0x0000:
	case PCI_VENDOR_ID_BROADCOM:
		break;
	default:
<<<<<<< HEAD
		DHD_ERROR(("%s: unknown vendor: 0x%04x\n", __func__, venid));
=======
		BRCMF_ERROR(("%s: unknown vendor: 0x%04x\n", __func__, venid));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		return NULL;
	}

	/* Check the Device ID and make sure it's one that we support */
	switch (devid) {
	case BCM4325_D11DUAL_ID:	/* 4325 802.11a/g id */
	case BCM4325_D11G_ID:	/* 4325 802.11g 2.4Ghz band id */
	case BCM4325_D11A_ID:	/* 4325 802.11a 5Ghz band id */
<<<<<<< HEAD
		DHD_INFO(("%s: found 4325 Dongle\n", __func__));
=======
		BRCMF_INFO(("%s: found 4325 Dongle\n", __func__));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		break;
	case BCM4329_D11NDUAL_ID:	/* 4329 802.11n dualband device */
	case BCM4329_D11N2G_ID:	/* 4329 802.11n 2.4G device */
	case BCM4329_D11N5G_ID:	/* 4329 802.11n 5G device */
	case 0x4329:
<<<<<<< HEAD
		DHD_INFO(("%s: found 4329 Dongle\n", __func__));
=======
		BRCMF_INFO(("%s: found 4329 Dongle\n", __func__));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		break;
	case BCM4319_D11N_ID:	/* 4319 802.11n id */
	case BCM4319_D11N2G_ID:	/* 4319 802.11n2g id */
	case BCM4319_D11N5G_ID:	/* 4319 802.11n5g id */
<<<<<<< HEAD
		DHD_INFO(("%s: found 4319 Dongle\n", __func__));
		break;
	case 0:
		DHD_INFO(("%s: allow device id 0, will check chip internals\n",
			  __func__));
		break;

	default:
		DHD_ERROR(("%s: skipping 0x%04x/0x%04x, not a dongle\n",
			   __func__, venid, devid));
=======
		BRCMF_INFO(("%s: found 4319 Dongle\n", __func__));
		break;
	case 0:
		BRCMF_INFO(("%s: allow device id 0, will check chip"
			    " internals\n", __func__));
		break;

	default:
		BRCMF_ERROR(("%s: skipping 0x%04x/0x%04x, not a dongle\n",
			     __func__, venid, devid));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		return NULL;
	}

	/* Allocate private bus interface state */
<<<<<<< HEAD
	bus = kzalloc(sizeof(dhd_bus_t), GFP_ATOMIC);
	if (!bus) {
		DHD_ERROR(("%s: kmalloc of dhd_bus_t failed\n", __func__));
		goto fail;
	}
	bus->sdh = sdh;
	bus->cl_devid = (u16) devid;
	bus->bus = DHD_BUS;
=======
	bus = kzalloc(sizeof(struct brcmf_bus), GFP_ATOMIC);
	if (!bus) {
		BRCMF_ERROR(("%s: kmalloc of struct dhd_bus failed\n",
			     __func__));
		goto fail;
	}
	bus->card = card;
	bus->cl_devid = (u16) devid;
	bus->bus = BRCMF_BUS;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	bus->tx_seq = SDPCM_SEQUENCE_WRAP - 1;
	bus->usebufpool = false;	/* Use bufpool if allocated,
					 else use locally malloced rxbuf */

	/* attempt to attach to the dongle */
<<<<<<< HEAD
	if (!(dhdsdio_probe_attach(bus, sdh, regsva, devid))) {
		DHD_ERROR(("%s: dhdsdio_probe_attach failed\n", __func__));
		goto fail;
	}

	/* Attach to the dhd/OS/network interface */
	bus->dhd = dhd_attach(bus, SDPCM_RESERVE);
	if (!bus->dhd) {
		DHD_ERROR(("%s: dhd_attach failed\n", __func__));
=======
	if (!(brcmf_sdbrcm_probe_attach(bus, card, regsva, devid))) {
		BRCMF_ERROR(("%s: brcmf_sdbrcm_probe_attach failed\n",
			     __func__));
		goto fail;
	}

	spin_lock_init(&bus->txqlock);
	init_waitqueue_head(&bus->ctrl_wait);

	/* Set up the watchdog timer */
	init_timer(&bus->timer);
	bus->timer.data = (unsigned long)bus;
	bus->timer.function = brcmf_sdbrcm_watchdog;

	/* Initialize thread based operation and lock */
	if ((brcmf_watchdog_prio >= 0) && (brcmf_dpc_prio >= 0)) {
		bus->threads_only = true;
		sema_init(&bus->sdsem, 1);
	} else {
		bus->threads_only = false;
		spin_lock_init(&bus->sdlock);
	}

	if (brcmf_dpc_prio >= 0) {
		/* Initialize watchdog thread */
		init_completion(&bus->watchdog_wait);
		bus->watchdog_tsk = kthread_run(brcmf_sdbrcm_watchdog_thread,
						bus, "brcmf_watchdog");
		if (IS_ERR(bus->watchdog_tsk)) {
			printk(KERN_WARNING
			       "brcmf_watchdog thread failed to start\n");
			bus->watchdog_tsk = NULL;
		}
	} else
		bus->watchdog_tsk = NULL;

	/* Set up the bottom half handler */
	if (brcmf_dpc_prio >= 0) {
		/* Initialize DPC thread */
		init_completion(&bus->dpc_wait);
		bus->dpc_tsk = kthread_run(brcmf_sdbrcm_dpc_thread,
					   bus, "brcmf_dpc");
		if (IS_ERR(bus->dpc_tsk)) {
			printk(KERN_WARNING
			       "brcmf_dpc thread failed to start\n");
			bus->dpc_tsk = NULL;
		}
	} else {
		tasklet_init(&bus->tasklet, brcmf_sdbrcm_dpc_tasklet,
			     (unsigned long)bus);
		bus->dpc_tsk = NULL;
	}

	/* Attach to the brcmf/OS/network interface */
	bus->drvr = brcmf_attach(bus, SDPCM_RESERVE);
	if (!bus->drvr) {
		BRCMF_ERROR(("%s: brcmf_attach failed\n", __func__));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		goto fail;
	}

	/* Allocate buffers */
<<<<<<< HEAD
	if (!(dhdsdio_probe_malloc(bus, sdh))) {
		DHD_ERROR(("%s: dhdsdio_probe_malloc failed\n", __func__));
		goto fail;
	}

	if (!(dhdsdio_probe_init(bus, sdh))) {
		DHD_ERROR(("%s: dhdsdio_probe_init failed\n", __func__));
=======
	if (!(brcmf_sdbrcm_probe_malloc(bus, card))) {
		BRCMF_ERROR(("%s: brcmf_sdbrcm_probe_malloc failed\n",
			     __func__));
		goto fail;
	}

	if (!(brcmf_sdbrcm_probe_init(bus, card))) {
		BRCMF_ERROR(("%s: brcmf_sdbrcm_probe_init failed\n", __func__));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		goto fail;
	}

	/* Register interrupt callback, but mask it (not operational yet). */
<<<<<<< HEAD
	DHD_INTR(("%s: disable SDIO interrupts (not interested yet)\n",
		  __func__));
	bcmsdh_intr_disable(sdh);
	ret = bcmsdh_intr_reg(sdh, dhdsdio_isr, bus);
	if (ret != 0) {
		DHD_ERROR(("%s: FAILED: bcmsdh_intr_reg returned %d\n",
			   __func__, ret));
		goto fail;
	}
	DHD_INTR(("%s: registered SDIO interrupt function ok\n", __func__));

	DHD_INFO(("%s: completed!!\n", __func__));

	/* if firmware path present try to download and bring up bus */
	ret = dhd_bus_start(bus->dhd);
	if (ret != 0) {
		if (ret == -ENOLINK) {
			DHD_ERROR(("%s: dongle is not responding\n", __func__));
=======
	BRCMF_INTR(("%s: disable SDIO interrupts (not interested yet)\n",
		    __func__));
	brcmf_sdcard_intr_disable(card);
	ret = brcmf_sdcard_intr_reg(card, brcmf_sdbrcm_isr, bus);
	if (ret != 0) {
		BRCMF_ERROR(("%s: FAILED: sdcard_intr_reg returned %d\n",
			     __func__, ret));
		goto fail;
	}
	BRCMF_INTR(("%s: registered SDIO interrupt function ok\n", __func__));

	BRCMF_INFO(("%s: completed!!\n", __func__));

	/* if firmware path present try to download and bring up bus */
	ret = brcmf_bus_start(bus->drvr);
	if (ret != 0) {
		if (ret == -ENOLINK) {
			BRCMF_ERROR(("%s: dongle is not responding\n",
				     __func__));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			goto fail;
		}
	}
	/* Ok, have the per-port tell the stack we're open for business */
<<<<<<< HEAD
	if (dhd_net_attach(bus->dhd, 0) != 0) {
		DHD_ERROR(("%s: Net attach failed!!\n", __func__));
=======
	if (brcmf_net_attach(bus->drvr, 0) != 0) {
		BRCMF_ERROR(("%s: Net attach failed!!\n", __func__));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		goto fail;
	}

	return bus;

fail:
<<<<<<< HEAD
	dhdsdio_release(bus);
=======
	brcmf_sdbrcm_release(bus);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	return NULL;
}

static bool
<<<<<<< HEAD
dhdsdio_probe_attach(struct dhd_bus *bus, void *sdh, void *regsva, u16 devid)
=======
brcmf_sdbrcm_probe_attach(struct brcmf_bus *bus, void *card, u32 regsva,
			  u16 devid)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
{
	u8 clkctl = 0;
	int err = 0;

	bus->alp_only = true;

	/* Return the window to backplane enumeration space for core access */
<<<<<<< HEAD
	if (dhdsdio_set_siaddr_window(bus, SI_ENUM_BASE))
		DHD_ERROR(("%s: FAILED to return to SI_ENUM_BASE\n", __func__));

#ifdef DHD_DEBUG
	printk(KERN_DEBUG "F1 signature read @0x18000000=0x%4x\n",
	       bcmsdh_reg_read(bus->sdh, SI_ENUM_BASE, 4));

#endif				/* DHD_DEBUG */

	/*
	 * Force PLL off until dhdsdio_chip_attach()
	 * programs PLL control regs
	 */

	bcmsdh_cfg_write(sdh, SDIO_FUNC_1, SBSDIO_FUNC1_CHIPCLKCSR,
			 DHD_INIT_CLKCTL1, &err);
	if (!err)
		clkctl =
		    bcmsdh_cfg_read(sdh, SDIO_FUNC_1, SBSDIO_FUNC1_CHIPCLKCSR,
				    &err);

	if (err || ((clkctl & ~SBSDIO_AVBITS) != DHD_INIT_CLKCTL1)) {
		DHD_ERROR(("dhdsdio_probe: ChipClkCSR access: err %d wrote "
			"0x%02x read 0x%02x\n",
			err, DHD_INIT_CLKCTL1, clkctl));
		goto fail;
	}
#ifdef DHD_DEBUG
	if (DHD_INFO_ON()) {
		uint fn, numfn;
		u8 *cis[SDIOD_MAX_IOFUNCS];
		int err = 0;

		numfn = bcmsdh_query_iofnum(sdh);
		ASSERT(numfn <= SDIOD_MAX_IOFUNCS);

		/* Make sure ALP is available before trying to read CIS */
		SPINWAIT(((clkctl = bcmsdh_cfg_read(sdh, SDIO_FUNC_1,
						    SBSDIO_FUNC1_CHIPCLKCSR,
						    NULL)),
			  !SBSDIO_ALPAV(clkctl)), PMU_MAX_TRANSITION_DLY);

		/* Now request ALP be put on the bus */
		bcmsdh_cfg_write(sdh, SDIO_FUNC_1, SBSDIO_FUNC1_CHIPCLKCSR,
				 DHD_INIT_CLKCTL2, &err);
		udelay(65);

		for (fn = 0; fn <= numfn; fn++) {
			cis[fn] = kzalloc(SBSDIO_CIS_SIZE_LIMIT, GFP_ATOMIC);
			if (!cis[fn]) {
				DHD_INFO(("dhdsdio_probe: fn %d cis malloc "
					"failed\n", fn));
				break;
			}

			err = bcmsdh_cis_read(sdh, fn, cis[fn],
						SBSDIO_CIS_SIZE_LIMIT);
			if (err) {
				DHD_INFO(("dhdsdio_probe: fn %d cis read "
					"err %d\n", fn, err));
				kfree(cis[fn]);
				break;
			}
			dhd_dump_cis(fn, cis[fn]);
		}

		while (fn-- > 0) {
			ASSERT(cis[fn]);
			kfree(cis[fn]);
		}

		if (err) {
			DHD_ERROR(("dhdsdio_probe: error read/parsing CIS\n"));
			goto fail;
		}
	}
#endif				/* DHD_DEBUG */

	if (dhdsdio_chip_attach(bus, regsva)) {
		DHD_ERROR(("%s: dhdsdio_chip_attach failed!\n", __func__));
		goto fail;
	}

	bcmsdh_chipinfo(sdh, bus->ci->chip, bus->ci->chiprev);

	if (!dhdsdio_chipmatch((u16) bus->ci->chip)) {
		DHD_ERROR(("%s: unsupported chip: 0x%04x\n",
			   __func__, bus->ci->chip));
		goto fail;
	}

	dhdsdio_sdiod_drive_strength_init(bus, dhd_sdiod_drive_strength);

	/* Get info on the ARM and SOCRAM cores... */
	if (!DHD_NOPMU(bus)) {
		bus->armrev = SBCOREREV(bcmsdh_reg_read(bus->sdh,
			CORE_SB(bus->ci->armcorebase, sbidhigh), 4));
		bus->orig_ramsize = bus->ci->ramsize;
		if (!(bus->orig_ramsize)) {
			DHD_ERROR(("%s: failed to find SOCRAM memory!\n",
				   __func__));
			goto fail;
		}
		bus->ramsize = bus->orig_ramsize;
		if (dhd_dongle_memsize)
			dhd_dongle_setmemsize(bus, dhd_dongle_memsize);

		DHD_ERROR(("DHD: dongle ram size is set to %d(orig %d)\n",
			   bus->ramsize, bus->orig_ramsize));
	}

	bus->regs = (void *)bus->ci->buscorebase;

	/* Set core control so an SDIO reset does a backplane reset */
	OR_REG(&bus->regs->corecontrol, CC_BPRESEN);

	bcm_pktq_init(&bus->txq, (PRIOMASK + 1), TXQLEN);

	/* Locate an appropriately-aligned portion of hdrbuf */
	bus->rxhdr = (u8 *) roundup((unsigned long)&bus->hdrbuf[0], DHD_SDALIGN);

	/* Set the poll and/or interrupt flags */
	bus->intr = (bool) dhd_intr;
	bus->poll = (bool) dhd_poll;
=======
	if (brcmf_sdbrcm_set_siaddr_window(bus, SI_ENUM_BASE))
		BRCMF_ERROR(("%s: FAILED to return to SI_ENUM_BASE\n",
			     __func__));

#ifdef BCMDBG
	printk(KERN_DEBUG "F1 signature read @0x18000000=0x%4x\n",
	       brcmf_sdcard_reg_read(bus->card, SI_ENUM_BASE, 4));

#endif				/* BCMDBG */

	/*
	 * Force PLL off until brcmf_sdbrcm_chip_attach()
	 * programs PLL control regs
	 */

	brcmf_sdcard_cfg_write(card, SDIO_FUNC_1, SBSDIO_FUNC1_CHIPCLKCSR,
			 BRCMF_INIT_CLKCTL1, &err);
	if (!err)
		clkctl =
		    brcmf_sdcard_cfg_read(card, SDIO_FUNC_1,
					  SBSDIO_FUNC1_CHIPCLKCSR, &err);

	if (err || ((clkctl & ~SBSDIO_AVBITS) != BRCMF_INIT_CLKCTL1)) {
		BRCMF_ERROR(("brcmf_sdbrcm_probe: ChipClkCSR access: err %d"
			     " wrote 0x%02x read 0x%02x\n",
			     err, BRCMF_INIT_CLKCTL1, clkctl));
		goto fail;
	}

	if (brcmf_sdbrcm_chip_attach(bus, regsva)) {
		BRCMF_ERROR(("%s: brcmf_sdbrcm_chip_attach failed!\n",
			     __func__));
		goto fail;
	}

	if (!brcmf_sdbrcm_chipmatch((u16) bus->ci->chip)) {
		BRCMF_ERROR(("%s: unsupported chip: 0x%04x\n",
			     __func__, bus->ci->chip));
		goto fail;
	}

	brcmf_sdbrcm_sdiod_drive_strength_init(bus, brcmf_sdiod_drive_strength);

	/* Get info on the ARM and SOCRAM cores... */
	if (!BRCMF_NOPMU(bus)) {
		brcmf_sdcard_reg_read(bus->card,
			  CORE_SB(bus->ci->armcorebase, sbidhigh), 4);
		bus->orig_ramsize = bus->ci->ramsize;
		if (!(bus->orig_ramsize)) {
			BRCMF_ERROR(("%s: failed to find SOCRAM memory!\n",
				     __func__));
			goto fail;
		}
		bus->ramsize = bus->orig_ramsize;
		if (brcmf_dongle_memsize)
			brcmf_sdbrcm_setmemsize(bus, brcmf_dongle_memsize);

		BRCMF_ERROR(("DHD: dongle ram size is set to %d(orig %d)\n",
			     bus->ramsize, bus->orig_ramsize));
	}

	/* Set core control so an SDIO reset does a backplane reset */
	OR_REG(bus->ci->buscorebase + offsetof(struct sdpcmd_regs,
						       corecontrol),
	       CC_BPRESEN, u32);

	brcmu_pktq_init(&bus->txq, (PRIOMASK + 1), TXQLEN);

	/* Locate an appropriately-aligned portion of hdrbuf */
	bus->rxhdr = (u8 *) roundup((unsigned long)&bus->hdrbuf[0],
				    BRCMF_SDALIGN);

	/* Set the poll and/or interrupt flags */
	bus->intr = (bool) brcmf_intr;
	bus->poll = (bool) brcmf_poll;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	if (bus->poll)
		bus->pollrate = 1;

	return true;

fail:
	return false;
}

<<<<<<< HEAD
static bool dhdsdio_probe_malloc(dhd_bus_t *bus, void *sdh)
{
	DHD_TRACE(("%s: Enter\n", __func__));

	if (bus->dhd->maxctl) {
		bus->rxblen =
		    roundup((bus->dhd->maxctl + SDPCM_HDRLEN),
			    ALIGNMENT) + DHD_SDALIGN;
		bus->rxbuf = kmalloc(bus->rxblen, GFP_ATOMIC);
		if (!(bus->rxbuf)) {
			DHD_ERROR(("%s: kmalloc of %d-byte rxbuf failed\n",
				   __func__, bus->rxblen));
=======
static bool brcmf_sdbrcm_probe_malloc(struct brcmf_bus *bus, void *card)
{
	BRCMF_TRACE(("%s: Enter\n", __func__));

	if (bus->drvr->maxctl) {
		bus->rxblen =
		    roundup((bus->drvr->maxctl + SDPCM_HDRLEN),
			    ALIGNMENT) + BRCMF_SDALIGN;
		bus->rxbuf = kmalloc(bus->rxblen, GFP_ATOMIC);
		if (!(bus->rxbuf)) {
			BRCMF_ERROR(("%s: kmalloc of %d-byte rxbuf failed\n",
				     __func__, bus->rxblen));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			goto fail;
		}
	}

	/* Allocate buffer to receive glomed packet */
	bus->databuf = kmalloc(MAX_DATA_BUF, GFP_ATOMIC);
	if (!(bus->databuf)) {
<<<<<<< HEAD
		DHD_ERROR(("%s: kmalloc of %d-byte databuf failed\n",
			   __func__, MAX_DATA_BUF));
=======
		BRCMF_ERROR(("%s: kmalloc of %d-byte databuf failed\n",
			     __func__, MAX_DATA_BUF));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		/* release rxbuf which was already located as above */
		if (!bus->rxblen)
			kfree(bus->rxbuf);
		goto fail;
	}

	/* Align the buffer */
<<<<<<< HEAD
	if ((unsigned long)bus->databuf % DHD_SDALIGN)
		bus->dataptr =
		    bus->databuf + (DHD_SDALIGN -
				    ((unsigned long)bus->databuf % DHD_SDALIGN));
=======
	if ((unsigned long)bus->databuf % BRCMF_SDALIGN)
		bus->dataptr = bus->databuf + (BRCMF_SDALIGN -
			       ((unsigned long)bus->databuf % BRCMF_SDALIGN));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	else
		bus->dataptr = bus->databuf;

	return true;

fail:
	return false;
}

<<<<<<< HEAD
static bool dhdsdio_probe_init(dhd_bus_t *bus, void *sdh)
{
	s32 fnum;

	DHD_TRACE(("%s: Enter\n", __func__));

#ifdef SDTEST
	dhdsdio_pktgen_init(bus);
#endif				/* SDTEST */

	/* Disable F2 to clear any intermediate frame state on the dongle */
	bcmsdh_cfg_write(sdh, SDIO_FUNC_0, SDIOD_CCCR_IOEN, SDIO_FUNC_ENABLE_1,
			 NULL);

	bus->dhd->busstate = DHD_BUS_DOWN;
	bus->sleeping = false;
	bus->rxflow = false;
	bus->prev_rxlim_hit = 0;

	/* Done with backplane-dependent accesses, can drop clock... */
	bcmsdh_cfg_write(sdh, SDIO_FUNC_1, SBSDIO_FUNC1_CHIPCLKCSR, 0, NULL);

	/* ...and initialize clock/power states */
	bus->clkstate = CLK_SDONLY;
	bus->idletime = (s32) dhd_idletime;
	bus->idleclock = DHD_IDLE_ACTIVE;

	/* Query the SD clock speed */
	if (bcmsdh_iovar_op(sdh, "sd_divisor", NULL, 0,
			    &bus->sd_divisor, sizeof(s32),
			    false) != 0) {
		DHD_ERROR(("%s: fail on %s get\n", __func__, "sd_divisor"));
		bus->sd_divisor = -1;
	} else {
		DHD_INFO(("%s: Initial value for %s is %d\n",
			  __func__, "sd_divisor", bus->sd_divisor));
	}

	/* Query the SD bus mode */
	if (bcmsdh_iovar_op(sdh, "sd_mode", NULL, 0,
			    &bus->sd_mode, sizeof(s32), false) != 0) {
		DHD_ERROR(("%s: fail on %s get\n", __func__, "sd_mode"));
		bus->sd_mode = -1;
	} else {
		DHD_INFO(("%s: Initial value for %s is %d\n",
			  __func__, "sd_mode", bus->sd_mode));
	}

	/* Query the F2 block size, set roundup accordingly */
	fnum = 2;
	if (bcmsdh_iovar_op(sdh, "sd_blocksize", &fnum, sizeof(s32),
			    &bus->blocksize, sizeof(s32), false) != 0) {
		bus->blocksize = 0;
		DHD_ERROR(("%s: fail on %s get\n", __func__, "sd_blocksize"));
	} else {
		DHD_INFO(("%s: Initial value for %s is %d\n",
			  __func__, "sd_blocksize", bus->blocksize));
=======
static bool brcmf_sdbrcm_probe_init(struct brcmf_bus *bus, void *card)
{
	s32 fnum;

	BRCMF_TRACE(("%s: Enter\n", __func__));

#ifdef SDTEST
	brcmf_sdbrcm_pktgen_init(bus);
#endif				/* SDTEST */

	/* Disable F2 to clear any intermediate frame state on the dongle */
	brcmf_sdcard_cfg_write(card, SDIO_FUNC_0, SDIO_CCCR_IOEx,
			       SDIO_FUNC_ENABLE_1, NULL);

	bus->drvr->busstate = BRCMF_BUS_DOWN;
	bus->sleeping = false;
	bus->rxflow = false;

	/* Done with backplane-dependent accesses, can drop clock... */
	brcmf_sdcard_cfg_write(card, SDIO_FUNC_1, SBSDIO_FUNC1_CHIPCLKCSR, 0,
			       NULL);

	/* ...and initialize clock/power states */
	bus->clkstate = CLK_SDONLY;
	bus->idletime = (s32) brcmf_idletime;
	bus->idleclock = BRCMF_IDLE_ACTIVE;

	/* Query the F2 block size, set roundup accordingly */
	fnum = 2;
	if (brcmf_sdcard_iovar_op(card, "sd_blocksize", &fnum, sizeof(s32),
			    &bus->blocksize, sizeof(s32), false) != 0) {
		bus->blocksize = 0;
		BRCMF_ERROR(("%s: fail on %s get\n", __func__, "sd_blocksize"));
	} else {
		BRCMF_INFO(("%s: Initial value for %s is %d\n",
			    __func__, "sd_blocksize", bus->blocksize));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	}
	bus->roundup = min(max_roundup, bus->blocksize);

	/* Query if bus module supports packet chaining,
		 default to use if supported */
<<<<<<< HEAD
	if (bcmsdh_iovar_op(sdh, "sd_rxchain", NULL, 0,
=======
	if (brcmf_sdcard_iovar_op(card, "sd_rxchain", NULL, 0,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			    &bus->sd_rxchain, sizeof(s32),
			    false) != 0) {
		bus->sd_rxchain = false;
	} else {
<<<<<<< HEAD
		DHD_INFO(("%s: bus module (through bcmsdh API) %s chaining\n",
			  __func__,
			  (bus->sd_rxchain ? "supports" : "does not support")));
=======
		BRCMF_INFO(("%s: bus module (through sdiocard API) %s"
			    " chaining\n", __func__, bus->sd_rxchain
			    ? "supports" : "does not support"));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	}
	bus->use_rxchain = (bool) bus->sd_rxchain;

	return true;
}

<<<<<<< HEAD
bool
dhd_bus_download_firmware(struct dhd_bus *bus, char *fw_path, char *nv_path)
{
	bool ret;
	bus->fw_path = fw_path;
	bus->nv_path = nv_path;

	ret = dhdsdio_download_firmware(bus, bus->sdh);

	return ret;
}

static bool
dhdsdio_download_firmware(struct dhd_bus *bus, void *sdh)
=======
static bool
brcmf_sdbrcm_download_firmware(struct brcmf_bus *bus, void *card)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
{
	bool ret;

	/* Download the firmware */
<<<<<<< HEAD
	dhdsdio_clkctl(bus, CLK_AVAIL, false);

	ret = _dhdsdio_download_firmware(bus) == 0;

	dhdsdio_clkctl(bus, CLK_SDONLY, false);
=======
	brcmf_sdbrcm_clkctl(bus, CLK_AVAIL, false);

	ret = _brcmf_sdbrcm_download_firmware(bus) == 0;

	brcmf_sdbrcm_clkctl(bus, CLK_SDONLY, false);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	return ret;
}

/* Detach and free everything */
<<<<<<< HEAD
static void dhdsdio_release(dhd_bus_t *bus)
{
	DHD_TRACE(("%s: Enter\n", __func__));

	if (bus) {
		/* De-register interrupt handler */
		bcmsdh_intr_disable(bus->sdh);
		bcmsdh_intr_dereg(bus->sdh);

		if (bus->dhd) {
			dhd_detach(bus->dhd);
			dhdsdio_release_dongle(bus);
			bus->dhd = NULL;
		}

		dhdsdio_release_malloc(bus);
=======
static void brcmf_sdbrcm_release(struct brcmf_bus *bus)
{
	BRCMF_TRACE(("%s: Enter\n", __func__));

	if (bus) {
		/* De-register interrupt handler */
		brcmf_sdcard_intr_disable(bus->card);
		brcmf_sdcard_intr_dereg(bus->card);

		if (bus->drvr) {
			brcmf_detach(bus->drvr);
			brcmf_sdbrcm_release_dongle(bus);
			bus->drvr = NULL;
		}

		brcmf_sdbrcm_release_malloc(bus);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

		kfree(bus);
	}

<<<<<<< HEAD
	DHD_TRACE(("%s: Disconnected\n", __func__));
}

static void dhdsdio_release_malloc(dhd_bus_t *bus)
{
	DHD_TRACE(("%s: Enter\n", __func__));

	if (bus->dhd && bus->dhd->dongle_reset)
		return;

	if (bus->rxbuf) {
		kfree(bus->rxbuf);
		bus->rxctl = bus->rxbuf = NULL;
		bus->rxlen = 0;
	}
=======
	BRCMF_TRACE(("%s: Disconnected\n", __func__));
}

static void brcmf_sdbrcm_release_malloc(struct brcmf_bus *bus)
{
	BRCMF_TRACE(("%s: Enter\n", __func__));

	if (bus->drvr && bus->drvr->dongle_reset)
		return;

	kfree(bus->rxbuf);
	bus->rxctl = bus->rxbuf = NULL;
	bus->rxlen = 0;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	kfree(bus->databuf);
	bus->databuf = NULL;
}

<<<<<<< HEAD
static void dhdsdio_release_dongle(dhd_bus_t *bus)
{
	DHD_TRACE(("%s: Enter\n", __func__));

	if (bus->dhd && bus->dhd->dongle_reset)
		return;

	if (bus->ci) {
		dhdsdio_clkctl(bus, CLK_AVAIL, false);
		dhdsdio_clkctl(bus, CLK_NONE, false);
		dhdsdio_chip_detach(bus);
=======
static void brcmf_sdbrcm_release_dongle(struct brcmf_bus *bus)
{
	BRCMF_TRACE(("%s: Enter\n", __func__));

	if (bus->drvr && bus->drvr->dongle_reset)
		return;

	if (bus->ci) {
		brcmf_sdbrcm_clkctl(bus, CLK_AVAIL, false);
		brcmf_sdbrcm_clkctl(bus, CLK_NONE, false);
		brcmf_sdbrcm_chip_detach(bus);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		if (bus->vars && bus->varsz)
			kfree(bus->vars);
		bus->vars = NULL;
	}

<<<<<<< HEAD
	DHD_TRACE(("%s: Disconnected\n", __func__));
}

static void dhdsdio_disconnect(void *ptr)
{
	dhd_bus_t *bus = (dhd_bus_t *)ptr;

	DHD_TRACE(("%s: Enter\n", __func__));

	if (bus) {
		ASSERT(bus->dhd);
		dhdsdio_release(bus);
	}

	DHD_TRACE(("%s: Disconnected\n", __func__));
=======
	BRCMF_TRACE(("%s: Disconnected\n", __func__));
}

static void brcmf_sdbrcm_disconnect(void *ptr)
{
	struct brcmf_bus *bus = (struct brcmf_bus *)ptr;

	BRCMF_TRACE(("%s: Enter\n", __func__));

	if (bus) {
		brcmf_sdbrcm_release(bus);
	}

	BRCMF_TRACE(("%s: Disconnected\n", __func__));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
}

/* Register/Unregister functions are called by the main DHD entry
 * point (e.g. module insertion) to link with the bus driver, in
 * order to look for or await the device.
 */

<<<<<<< HEAD
static bcmsdh_driver_t dhd_sdio = {
	dhdsdio_probe,
	dhdsdio_disconnect
};

int dhd_bus_register(void)
{
	DHD_TRACE(("%s: Enter\n", __func__));

	return bcmsdh_register(&dhd_sdio);
}

void dhd_bus_unregister(void)
{
	DHD_TRACE(("%s: Enter\n", __func__));

	bcmsdh_unregister();
}

#ifdef BCMEMBEDIMAGE
static int dhdsdio_download_code_array(struct dhd_bus *bus)
{
	int bcmerror = -1;
	int offset = 0;

	DHD_INFO(("%s: download embedded firmware...\n", __func__));

	/* Download image */
	while ((offset + MEMBLOCK) < sizeof(dlarray)) {
		bcmerror =
		    dhdsdio_membytes(bus, true, offset, dlarray + offset,
				     MEMBLOCK);
		if (bcmerror) {
			DHD_ERROR(("%s: error %d on writing %d membytes at "
				"0x%08x\n",
				__func__, bcmerror, MEMBLOCK, offset));
			goto err;
		}

		offset += MEMBLOCK;
	}

	if (offset < sizeof(dlarray)) {
		bcmerror = dhdsdio_membytes(bus, true, offset,
					    dlarray + offset,
					    sizeof(dlarray) - offset);
		if (bcmerror) {
			DHD_ERROR(("%s: error %d on writing %d membytes at "
				"0x%08x\n", __func__, bcmerror,
				sizeof(dlarray) - offset, offset));
			goto err;
		}
	}
#ifdef DHD_DEBUG
	/* Upload and compare the downloaded code */
	{
		unsigned char *ularray;

		ularray = kmalloc(bus->ramsize, GFP_ATOMIC);
		if (!ularray) {
			bcmerror = -ENOMEM;
			goto err;
		}
		/* Upload image to verify downloaded contents. */
		offset = 0;
		memset(ularray, 0xaa, bus->ramsize);
		while ((offset + MEMBLOCK) < sizeof(dlarray)) {
			bcmerror =
			    dhdsdio_membytes(bus, false, offset,
					     ularray + offset, MEMBLOCK);
			if (bcmerror) {
				DHD_ERROR(("%s: error %d on reading %d membytes"
					" at 0x%08x\n",
					__func__, bcmerror, MEMBLOCK, offset));
				goto free;
			}

			offset += MEMBLOCK;
		}

		if (offset < sizeof(dlarray)) {
			bcmerror = dhdsdio_membytes(bus, false, offset,
						    ularray + offset,
						    sizeof(dlarray) - offset);
			if (bcmerror) {
				DHD_ERROR(("%s: error %d on reading %d membytes at 0x%08x\n",
				__func__, bcmerror,
				sizeof(dlarray) - offset, offset));
				goto free;
			}
		}

		if (memcmp(dlarray, ularray, sizeof(dlarray))) {
			DHD_ERROR(("%s: Downloaded image is corrupted.\n",
				   __func__));
			ASSERT(0);
			goto free;
		} else
			DHD_ERROR(("%s: Download/Upload/Compare succeeded.\n",
				__func__));
free:
		kfree(ularray);
	}
#endif				/* DHD_DEBUG */

err:
	return bcmerror;
}
#endif				/* BCMEMBEDIMAGE */

static int dhdsdio_download_code_file(struct dhd_bus *bus, char *fw_path)
{
	int bcmerror = -1;
	int offset = 0;
	uint len;
	void *image = NULL;
	u8 *memblock = NULL, *memptr;

	DHD_INFO(("%s: download firmware %s\n", __func__, fw_path));

	image = dhd_os_open_image(fw_path);
	if (image == NULL)
		goto err;

	memptr = memblock = kmalloc(MEMBLOCK + DHD_SDALIGN, GFP_ATOMIC);
	if (memblock == NULL) {
		DHD_ERROR(("%s: Failed to allocate memory %d bytes\n",
			   __func__, MEMBLOCK));
		goto err;
	}
	if ((u32)(unsigned long)memblock % DHD_SDALIGN)
		memptr +=
		    (DHD_SDALIGN - ((u32)(unsigned long)memblock % DHD_SDALIGN));

	/* Download image */
	while ((len =
		dhd_os_get_image_block((char *)memptr, MEMBLOCK, image))) {
		bcmerror = dhdsdio_membytes(bus, true, offset, memptr, len);
		if (bcmerror) {
			DHD_ERROR(("%s: error %d on writing %d membytes at "
			"0x%08x\n", __func__, bcmerror, MEMBLOCK, offset));
=======
static struct brcmf_sdioh_driver brcmf_sdio = {
	brcmf_sdbrcm_probe,
	brcmf_sdbrcm_disconnect
};

int brcmf_bus_register(void)
{
	BRCMF_TRACE(("%s: Enter\n", __func__));

	/* Sanity check on the module parameters */
	do {
		/* Both watchdog and DPC as tasklets are ok */
		if ((brcmf_watchdog_prio < 0) && (brcmf_dpc_prio < 0))
			break;

		/* If both watchdog and DPC are threads, TX must be deferred */
		if ((brcmf_watchdog_prio >= 0) && (brcmf_dpc_prio >= 0)
		    && brcmf_deferred_tx)
			break;

		BRCMF_ERROR(("Invalid module parameters.\n"));
		return -EINVAL;
	} while (0);

	return brcmf_sdio_register(&brcmf_sdio);
}

void brcmf_bus_unregister(void)
{
	BRCMF_TRACE(("%s: Enter\n", __func__));

	brcmf_sdio_unregister();
}

static int brcmf_sdbrcm_download_code_file(struct brcmf_bus *bus)
{
	int offset = 0;
	uint len;
	u8 *memblock = NULL, *memptr;
	int ret;

	BRCMF_INFO(("%s: Enter\n", __func__));

	bus->fw_name = BCM4329_FW_NAME;
	ret = request_firmware(&bus->firmware, bus->fw_name,
			       &gInstance->func[2]->dev);
	if (ret) {
		BRCMF_ERROR(("%s: Fail to request firmware %d\n",
			     __func__, ret));
		return ret;
	}
	bus->fw_ptr = 0;

	memptr = memblock = kmalloc(MEMBLOCK + BRCMF_SDALIGN, GFP_ATOMIC);
	if (memblock == NULL) {
		BRCMF_ERROR(("%s: Failed to allocate memory %d bytes\n",
			     __func__, MEMBLOCK));
		ret = -ENOMEM;
		goto err;
	}
	if ((u32)(unsigned long)memblock % BRCMF_SDALIGN)
		memptr += (BRCMF_SDALIGN -
			   ((u32)(unsigned long)memblock % BRCMF_SDALIGN));

	/* Download image */
	while ((len =
		brcmf_sdbrcm_get_image((char *)memptr, MEMBLOCK, bus))) {
		ret = brcmf_sdbrcm_membytes(bus, true, offset, memptr, len);
		if (ret) {
			BRCMF_ERROR(("%s: error %d on writing %d membytes at "
				     "0x%08x\n", __func__, ret, MEMBLOCK,
				     offset));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			goto err;
		}

		offset += MEMBLOCK;
	}

err:
	kfree(memblock);

<<<<<<< HEAD
	if (image)
		dhd_os_close_image(image);

	return bcmerror;
=======
	release_firmware(bus->firmware);
	bus->fw_ptr = 0;

	return ret;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
}

/*
 * ProcessVars:Takes a buffer of "<var>=<value>\n" lines read from a file
 * and ending in a NUL.
 * Removes carriage returns, empty lines, comment lines, and converts
 * newlines to NULs.
 * Shortens buffer as needed and pads with NULs.  End of buffer is marked
 * by two NULs.
*/

<<<<<<< HEAD
static uint process_nvram_vars(char *varbuf, uint len)
=======
static uint brcmf_process_nvram_vars(char *varbuf, uint len)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
{
	char *dp;
	bool findNewline;
	int column;
	uint buf_len, n;

	dp = varbuf;

	findNewline = false;
	column = 0;

	for (n = 0; n < len; n++) {
		if (varbuf[n] == 0)
			break;
		if (varbuf[n] == '\r')
			continue;
		if (findNewline && varbuf[n] != '\n')
			continue;
		findNewline = false;
		if (varbuf[n] == '#') {
			findNewline = true;
			continue;
		}
		if (varbuf[n] == '\n') {
			if (column == 0)
				continue;
			*dp++ = 0;
			column = 0;
			continue;
		}
		*dp++ = varbuf[n];
		column++;
	}
	buf_len = dp - varbuf;

	while (dp < varbuf + n)
		*dp++ = 0;

	return buf_len;
}

<<<<<<< HEAD
/*
	EXAMPLE: nvram_array
	nvram_arry format:
	name=value
	Use carriage return at the end of each assignment,
	 and an empty string with
	carriage return at the end of array.

	For example:
	unsigned char  nvram_array[] = {"name1=value1\n",
	"name2=value2\n", "\n"};
	Hex values start with 0x, and mac addr format: xx:xx:xx:xx:xx:xx.

	Search "EXAMPLE: nvram_array" to see how the array is activated.
*/

void dhd_bus_set_nvram_params(struct dhd_bus *bus, const char *nvram_params)
{
	bus->nvram_params = nvram_params;
}

static int dhdsdio_download_nvram(struct dhd_bus *bus)
{
	int bcmerror = -1;
	uint len;
	void *image = NULL;
	char *memblock = NULL;
	char *bufp;
	char *nv_path;
	bool nvram_file_exists;

	nv_path = bus->nv_path;

	nvram_file_exists = ((nv_path != NULL) && (nv_path[0] != '\0'));
	if (!nvram_file_exists && (bus->nvram_params == NULL))
		return 0;

	if (nvram_file_exists) {
		image = dhd_os_open_image(nv_path);
		if (image == NULL)
			goto err;
	}

	memblock = kmalloc(MEMBLOCK, GFP_ATOMIC);
	if (memblock == NULL) {
		DHD_ERROR(("%s: Failed to allocate memory %d bytes\n",
			   __func__, MEMBLOCK));
		goto err;
	}

	/* Download variables */
	if (nvram_file_exists) {
		len = dhd_os_get_image_block(memblock, MEMBLOCK, image);
	} else {
		len = strlen(bus->nvram_params);
		ASSERT(len <= MEMBLOCK);
		if (len > MEMBLOCK)
			len = MEMBLOCK;
		memcpy(memblock, bus->nvram_params, len);
	}
=======
static int brcmf_sdbrcm_download_nvram(struct brcmf_bus *bus)
{
	uint len;
	char *memblock = NULL;
	char *bufp;
	int ret;

	bus->nv_name = BCM4329_NV_NAME;
	ret = request_firmware(&bus->firmware, bus->nv_name,
			       &gInstance->func[2]->dev);
	if (ret) {
		BRCMF_ERROR(("%s: Fail to request nvram %d\n", __func__, ret));
		return ret;
	}
	bus->fw_ptr = 0;

	memblock = kmalloc(MEMBLOCK, GFP_ATOMIC);
	if (memblock == NULL) {
		BRCMF_ERROR(("%s: Failed to allocate memory %d bytes\n",
			     __func__, MEMBLOCK));
		ret = -ENOMEM;
		goto err;
	}

	len = brcmf_sdbrcm_get_image(memblock, MEMBLOCK, bus);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	if (len > 0 && len < MEMBLOCK) {
		bufp = (char *)memblock;
		bufp[len] = 0;
<<<<<<< HEAD
		len = process_nvram_vars(bufp, len);
		bufp += len;
		*bufp++ = 0;
		if (len)
			bcmerror = dhdsdio_downloadvars(bus, memblock, len + 1);
		if (bcmerror) {
			DHD_ERROR(("%s: error downloading vars: %d\n",
				   __func__, bcmerror));
		}
	} else {
		DHD_ERROR(("%s: error reading nvram file: %d\n",
			   __func__, len));
		bcmerror = -EIO;
=======
		len = brcmf_process_nvram_vars(bufp, len);
		bufp += len;
		*bufp++ = 0;
		if (len)
			ret = brcmf_sdbrcm_downloadvars(bus, memblock, len + 1);
		if (ret)
			BRCMF_ERROR(("%s: error downloading vars: %d\n",
				     __func__, ret));
	} else {
		BRCMF_ERROR(("%s: error reading nvram file: %d\n",
			     __func__, len));
		ret = -EIO;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	}

err:
	kfree(memblock);

<<<<<<< HEAD
	if (image)
		dhd_os_close_image(image);

	return bcmerror;
}

static int _dhdsdio_download_firmware(struct dhd_bus *bus)
{
	int bcmerror = -1;

	bool embed = false;	/* download embedded firmware */
	bool dlok = false;	/* download firmware succeeded */

	/* Out immediately if no image to download */
	if ((bus->fw_path == NULL) || (bus->fw_path[0] == '\0')) {
#ifdef BCMEMBEDIMAGE
		embed = true;
#else
		return bcmerror;
#endif
	}

	/* Keep arm in reset */
	if (dhdsdio_download_state(bus, true)) {
		DHD_ERROR(("%s: error placing ARM core in reset\n", __func__));
=======
	release_firmware(bus->firmware);
	bus->fw_ptr = 0;

	return ret;
}

static int _brcmf_sdbrcm_download_firmware(struct brcmf_bus *bus)
{
	int bcmerror = -1;

	/* Keep arm in reset */
	if (brcmf_sdbrcm_download_state(bus, true)) {
		BRCMF_ERROR(("%s: error placing ARM core in reset\n",
			     __func__));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		goto err;
	}

	/* External image takes precedence if specified */
<<<<<<< HEAD
	if ((bus->fw_path != NULL) && (bus->fw_path[0] != '\0')) {
		if (dhdsdio_download_code_file(bus, bus->fw_path)) {
			DHD_ERROR(("%s: dongle image file download failed\n",
				   __func__));
#ifdef BCMEMBEDIMAGE
			embed = true;
#else
			goto err;
#endif
		} else {
			embed = false;
			dlok = true;
		}
	}
#ifdef BCMEMBEDIMAGE
	if (embed) {
		if (dhdsdio_download_code_array(bus)) {
			DHD_ERROR(("%s: dongle image array download failed\n",
				   __func__));
			goto err;
		} else {
			dlok = true;
		}
	}
#endif
	if (!dlok) {
		DHD_ERROR(("%s: dongle image download failed\n", __func__));
		goto err;
	}

	/* EXAMPLE: nvram_array */
	/* If a valid nvram_arry is specified as above, it can be passed
		 down to dongle */
	/* dhd_bus_set_nvram_params(bus, (char *)&nvram_array); */

	/* External nvram takes precedence if specified */
	if (dhdsdio_download_nvram(bus)) {
		DHD_ERROR(("%s: dongle nvram file download failed\n",
			   __func__));
	}

	/* Take arm out of reset */
	if (dhdsdio_download_state(bus, false)) {
		DHD_ERROR(("%s: error getting out of ARM core reset\n",
			   __func__));
=======
	if (brcmf_sdbrcm_download_code_file(bus)) {
		BRCMF_ERROR(("%s: dongle image file download failed\n",
			     __func__));
		goto err;
	}

	/* External nvram takes precedence if specified */
	if (brcmf_sdbrcm_download_nvram(bus)) {
		BRCMF_ERROR(("%s: dongle nvram file download failed\n",
			     __func__));
	}

	/* Take arm out of reset */
	if (brcmf_sdbrcm_download_state(bus, false)) {
		BRCMF_ERROR(("%s: error getting out of ARM core reset\n",
			     __func__));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		goto err;
	}

	bcmerror = 0;

err:
	return bcmerror;
}


static int
<<<<<<< HEAD
dhd_bcmsdh_send_buf(dhd_bus_t *bus, u32 addr, uint fn, uint flags,
		    u8 *buf, uint nbytes, struct sk_buff *pkt,
		    bcmsdh_cmplt_fn_t complete, void *handle)
{
	return bcmsdh_send_buf
		(bus->sdh, addr, fn, flags, buf, nbytes, pkt, complete,
		 handle);
}

uint dhd_bus_chip(struct dhd_bus *bus)
{
	ASSERT(bus->ci != NULL);
	return bus->ci->chip;
}

void *dhd_bus_pub(struct dhd_bus *bus)
{
	return bus->dhd;
}

void *dhd_bus_txq(struct dhd_bus *bus)
{
	return &bus->txq;
}

uint dhd_bus_hdrlen(struct dhd_bus *bus)
{
	return SDPCM_HDRLEN;
}

int dhd_bus_devreset(dhd_pub_t *dhdp, u8 flag)
{
	int bcmerror = 0;
	dhd_bus_t *bus;

	bus = dhdp->bus;

	if (flag == true) {
		if (!bus->dhd->dongle_reset) {
			/* Expect app to have torn down any
			 connection before calling */
			/* Stop the bus, disable F2 */
			dhd_bus_stop(bus, false);

			/* Clean tx/rx buffer pointers,
			 detach from the dongle */
			dhdsdio_release_dongle(bus);

			bus->dhd->dongle_reset = true;
			bus->dhd->up = false;

			DHD_TRACE(("%s:  WLAN OFF DONE\n", __func__));
=======
brcmf_sdbrcm_send_buf(struct brcmf_bus *bus, u32 addr, uint fn, uint flags,
		    u8 *buf, uint nbytes, struct sk_buff *pkt,
		    void (*complete)(void *handle, int status,
				     bool sync_waiting),
		    void *handle)
{
	return brcmf_sdcard_send_buf
		(bus->card, addr, fn, flags, buf, nbytes, pkt, complete,
		 handle);
}

int brcmf_bus_devreset(struct brcmf_pub *drvr, u8 flag)
{
	int bcmerror = 0;
	struct brcmf_bus *bus;

	bus = drvr->bus;

	if (flag == true) {
		brcmf_sdbrcm_wd_timer(bus, 0);
		if (!bus->drvr->dongle_reset) {
			/* Expect app to have torn down any
			 connection before calling */
			/* Stop the bus, disable F2 */
			brcmf_sdbrcm_bus_stop(bus, false);

			/* Clean tx/rx buffer pointers,
			 detach from the dongle */
			brcmf_sdbrcm_release_dongle(bus);

			bus->drvr->dongle_reset = true;
			bus->drvr->up = false;

			BRCMF_TRACE(("%s:  WLAN OFF DONE\n", __func__));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			/* App can now remove power from device */
		} else
			bcmerror = -EIO;
	} else {
		/* App must have restored power to device before calling */

<<<<<<< HEAD
		DHD_TRACE(("\n\n%s: == WLAN ON ==\n", __func__));

		if (bus->dhd->dongle_reset) {
			/* Turn on WLAN */
			/* Reset SD client */
			bcmsdh_reset(bus->sdh);

			/* Attempt to re-attach & download */
			if (dhdsdio_probe_attach(bus, bus->sdh,
						 (u32 *) SI_ENUM_BASE,
						 bus->cl_devid)) {
				/* Attempt to download binary to the dongle */
				if (dhdsdio_probe_init
				    (bus, bus->sdh)
				    && dhdsdio_download_firmware(bus,
								 bus->sdh)) {

					/* Re-init bus, enable F2 transfer */
					dhd_bus_init((dhd_pub_t *) bus->dhd,
						     false);

#if defined(OOB_INTR_ONLY)
					dhd_enable_oob_intr(bus, true);
#endif				/* defined(OOB_INTR_ONLY) */

					bus->dhd->dongle_reset = false;
					bus->dhd->up = true;

					DHD_TRACE(("%s: WLAN ON DONE\n",
						   __func__));
=======
		BRCMF_TRACE(("\n\n%s: == WLAN ON ==\n", __func__));

		if (bus->drvr->dongle_reset) {
			/* Turn on WLAN */

			/* Attempt to re-attach & download */
			if (brcmf_sdbrcm_probe_attach(bus, bus->card,
						      SI_ENUM_BASE,
						      bus->cl_devid)) {
				/* Attempt to download binary to the dongle */
				if (brcmf_sdbrcm_probe_init(bus, bus->card)) {
					/* Re-init bus, enable F2 transfer */
					brcmf_sdbrcm_bus_init(bus->drvr, false);

					bus->drvr->dongle_reset = false;
					bus->drvr->up = true;

					BRCMF_TRACE(("%s: WLAN ON DONE\n",
						     __func__));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				} else
					bcmerror = -EIO;
			} else
				bcmerror = -EIO;
		} else {
			bcmerror = -EISCONN;
<<<<<<< HEAD
			DHD_ERROR(("%s: Set DEVRESET=false invoked when device "
				"is on\n", __func__));
			bcmerror = -EIO;
		}
=======
			BRCMF_ERROR(("%s: Set DEVRESET=false invoked when"
				     " device is on\n", __func__));
			bcmerror = -EIO;
		}
		brcmf_sdbrcm_wd_timer(bus, brcmf_watchdog_ms);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	}
	return bcmerror;
}

static int
<<<<<<< HEAD
dhdsdio_chip_recognition(bcmsdh_info_t *sdh, struct chip_info *ci, void *regs)
=======
brcmf_sdbrcm_chip_recognition(struct brcmf_sdio_card *card,
			      struct chip_info *ci, u32 regs)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
{
	u32 regdata;

	/*
	 * Get CC core rev
	 * Chipid is assume to be at offset 0 from regs arg
	 * For different chiptypes or old sdio hosts w/o chipcommon,
	 * other ways of recognition should be added here.
	 */
<<<<<<< HEAD
	ci->cccorebase = (u32)regs;
	regdata = bcmsdh_reg_read(sdh, CORE_CC_REG(ci->cccorebase, chipid), 4);
	ci->chip = regdata & CID_ID_MASK;
	ci->chiprev = (regdata & CID_REV_MASK) >> CID_REV_SHIFT;

	DHD_INFO(("%s: chipid=0x%x chiprev=%d\n",
		__func__, ci->chip, ci->chiprev));
=======
	ci->cccorebase = regs;
	regdata = brcmf_sdcard_reg_read(card,
				CORE_CC_REG(ci->cccorebase, chipid), 4);
	ci->chip = regdata & CID_ID_MASK;
	ci->chiprev = (regdata & CID_REV_MASK) >> CID_REV_SHIFT;

	BRCMF_INFO(("%s: chipid=0x%x chiprev=%d\n",
		    __func__, ci->chip, ci->chiprev));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	/* Address of cores for new chips should be added here */
	switch (ci->chip) {
	case BCM4329_CHIP_ID:
		ci->buscorebase = BCM4329_CORE_BUS_BASE;
		ci->ramcorebase = BCM4329_CORE_SOCRAM_BASE;
		ci->armcorebase	= BCM4329_CORE_ARM_BASE;
		ci->ramsize = BCM4329_RAMSIZE;
		break;
	default:
<<<<<<< HEAD
		DHD_ERROR(("%s: chipid 0x%x is not supported\n",
			__func__, ci->chip));
		return -ENODEV;
	}

	regdata = bcmsdh_reg_read(sdh,
		CORE_SB(ci->cccorebase, sbidhigh), 4);
	ci->ccrev = SBCOREREV(regdata);

	regdata = bcmsdh_reg_read(sdh,
		CORE_CC_REG(ci->cccorebase, pmucapabilities), 4);
	ci->pmurev = regdata & PCAP_REV_MASK;

	regdata = bcmsdh_reg_read(sdh, CORE_SB(ci->buscorebase, sbidhigh), 4);
	ci->buscorerev = SBCOREREV(regdata);
	ci->buscoretype = (regdata & SBIDH_CC_MASK) >> SBIDH_CC_SHIFT;

	DHD_INFO(("%s: ccrev=%d, pmurev=%d, buscore rev/type=%d/0x%x\n",
		__func__, ci->ccrev, ci->pmurev,
		ci->buscorerev, ci->buscoretype));

	/* get chipcommon capabilites */
	ci->cccaps = bcmsdh_reg_read(sdh,
=======
		BRCMF_ERROR(("%s: chipid 0x%x is not supported\n",
			     __func__, ci->chip));
		return -ENODEV;
	}

	regdata = brcmf_sdcard_reg_read(card,
		CORE_SB(ci->cccorebase, sbidhigh), 4);
	ci->ccrev = SBCOREREV(regdata);

	regdata = brcmf_sdcard_reg_read(card,
		CORE_CC_REG(ci->cccorebase, pmucapabilities), 4);
	ci->pmurev = regdata & PCAP_REV_MASK;

	regdata = brcmf_sdcard_reg_read(card,
					CORE_SB(ci->buscorebase, sbidhigh), 4);
	ci->buscorerev = SBCOREREV(regdata);
	ci->buscoretype = (regdata & SBIDH_CC_MASK) >> SBIDH_CC_SHIFT;

	BRCMF_INFO(("%s: ccrev=%d, pmurev=%d, buscore rev/type=%d/0x%x\n",
		    __func__, ci->ccrev, ci->pmurev,
		    ci->buscorerev, ci->buscoretype));

	/* get chipcommon capabilites */
	ci->cccaps = brcmf_sdcard_reg_read(card,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		CORE_CC_REG(ci->cccorebase, capabilities), 4);

	return 0;
}

static void
<<<<<<< HEAD
dhdsdio_chip_disablecore(bcmsdh_info_t *sdh, u32 corebase)
{
	u32 regdata;

	regdata = bcmsdh_reg_read(sdh,
=======
brcmf_sdbrcm_chip_disablecore(struct brcmf_sdio_card *card, u32 corebase)
{
	u32 regdata;

	regdata = brcmf_sdcard_reg_read(card,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		CORE_SB(corebase, sbtmstatelow), 4);
	if (regdata & SBTML_RESET)
		return;

<<<<<<< HEAD
	regdata = bcmsdh_reg_read(sdh,
=======
	regdata = brcmf_sdcard_reg_read(card,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		CORE_SB(corebase, sbtmstatelow), 4);
	if ((regdata & (SICF_CLOCK_EN << SBTML_SICF_SHIFT)) != 0) {
		/*
		 * set target reject and spin until busy is clear
		 * (preserve core-specific bits)
		 */
<<<<<<< HEAD
		regdata = bcmsdh_reg_read(sdh,
			CORE_SB(corebase, sbtmstatelow), 4);
		bcmsdh_reg_write(sdh, CORE_SB(corebase, sbtmstatelow), 4,
			regdata | SBTML_REJ);

		regdata = bcmsdh_reg_read(sdh,
			CORE_SB(corebase, sbtmstatelow), 4);
		udelay(1);
		SPINWAIT((bcmsdh_reg_read(sdh,
			CORE_SB(corebase, sbtmstatehigh), 4) &
			SBTMH_BUSY), 100000);

		regdata = bcmsdh_reg_read(sdh,
			CORE_SB(corebase, sbtmstatehigh), 4);
		if (regdata & SBTMH_BUSY)
			DHD_ERROR(("%s: ARM core still busy\n", __func__));

		regdata = bcmsdh_reg_read(sdh,
			CORE_SB(corebase, sbidlow), 4);
		if (regdata & SBIDL_INIT) {
			regdata = bcmsdh_reg_read(sdh,
				CORE_SB(corebase, sbimstate), 4) |
				SBIM_RJ;
			bcmsdh_reg_write(sdh,
				CORE_SB(corebase, sbimstate), 4,
				regdata);
			regdata = bcmsdh_reg_read(sdh,
				CORE_SB(corebase, sbimstate), 4);
			udelay(1);
			SPINWAIT((bcmsdh_reg_read(sdh,
=======
		regdata = brcmf_sdcard_reg_read(card,
			CORE_SB(corebase, sbtmstatelow), 4);
		brcmf_sdcard_reg_write(card, CORE_SB(corebase, sbtmstatelow), 4,
			regdata | SBTML_REJ);

		regdata = brcmf_sdcard_reg_read(card,
			CORE_SB(corebase, sbtmstatelow), 4);
		udelay(1);
		SPINWAIT((brcmf_sdcard_reg_read(card,
			CORE_SB(corebase, sbtmstatehigh), 4) &
			SBTMH_BUSY), 100000);

		regdata = brcmf_sdcard_reg_read(card,
			CORE_SB(corebase, sbtmstatehigh), 4);
		if (regdata & SBTMH_BUSY)
			BRCMF_ERROR(("%s: ARM core still busy\n", __func__));

		regdata = brcmf_sdcard_reg_read(card,
			CORE_SB(corebase, sbidlow), 4);
		if (regdata & SBIDL_INIT) {
			regdata = brcmf_sdcard_reg_read(card,
				CORE_SB(corebase, sbimstate), 4) |
				SBIM_RJ;
			brcmf_sdcard_reg_write(card,
				CORE_SB(corebase, sbimstate), 4,
				regdata);
			regdata = brcmf_sdcard_reg_read(card,
				CORE_SB(corebase, sbimstate), 4);
			udelay(1);
			SPINWAIT((brcmf_sdcard_reg_read(card,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				CORE_SB(corebase, sbimstate), 4) &
				SBIM_BY), 100000);
		}

		/* set reset and reject while enabling the clocks */
<<<<<<< HEAD
		bcmsdh_reg_write(sdh,
			CORE_SB(corebase, sbtmstatelow), 4,
			(((SICF_FGC | SICF_CLOCK_EN) << SBTML_SICF_SHIFT) |
			SBTML_REJ | SBTML_RESET));
		regdata = bcmsdh_reg_read(sdh,
=======
		brcmf_sdcard_reg_write(card,
			CORE_SB(corebase, sbtmstatelow), 4,
			(((SICF_FGC | SICF_CLOCK_EN) << SBTML_SICF_SHIFT) |
			SBTML_REJ | SBTML_RESET));
		regdata = brcmf_sdcard_reg_read(card,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			CORE_SB(corebase, sbtmstatelow), 4);
		udelay(10);

		/* clear the initiator reject bit */
<<<<<<< HEAD
		regdata = bcmsdh_reg_read(sdh,
			CORE_SB(corebase, sbidlow), 4);
		if (regdata & SBIDL_INIT) {
			regdata = bcmsdh_reg_read(sdh,
				CORE_SB(corebase, sbimstate), 4) &
				~SBIM_RJ;
			bcmsdh_reg_write(sdh,
=======
		regdata = brcmf_sdcard_reg_read(card,
			CORE_SB(corebase, sbidlow), 4);
		if (regdata & SBIDL_INIT) {
			regdata = brcmf_sdcard_reg_read(card,
				CORE_SB(corebase, sbimstate), 4) &
				~SBIM_RJ;
			brcmf_sdcard_reg_write(card,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				CORE_SB(corebase, sbimstate), 4,
				regdata);
		}
	}

	/* leave reset and reject asserted */
<<<<<<< HEAD
	bcmsdh_reg_write(sdh, CORE_SB(corebase, sbtmstatelow), 4,
=======
	brcmf_sdcard_reg_write(card, CORE_SB(corebase, sbtmstatelow), 4,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		(SBTML_REJ | SBTML_RESET));
	udelay(1);
}

static int
<<<<<<< HEAD
dhdsdio_chip_attach(struct dhd_bus *bus, void *regs)
=======
brcmf_sdbrcm_chip_attach(struct brcmf_bus *bus, u32 regs)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
{
	struct chip_info *ci;
	int err;
	u8 clkval, clkset;

<<<<<<< HEAD
	DHD_TRACE(("%s: Enter\n", __func__));
=======
	BRCMF_TRACE(("%s: Enter\n", __func__));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	/* alloc chip_info_t */
	ci = kmalloc(sizeof(struct chip_info), GFP_ATOMIC);
	if (NULL == ci) {
<<<<<<< HEAD
		DHD_ERROR(("%s: malloc failed!\n", __func__));
=======
		BRCMF_ERROR(("%s: malloc failed!\n", __func__));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		return -ENOMEM;
	}

	memset((unsigned char *)ci, 0, sizeof(struct chip_info));

	/* bus/core/clk setup for register access */
	/* Try forcing SDIO core to do ALPAvail request only */
	clkset = SBSDIO_FORCE_HW_CLKREQ_OFF | SBSDIO_ALP_AVAIL_REQ;
<<<<<<< HEAD
	bcmsdh_cfg_write(bus->sdh, SDIO_FUNC_1, SBSDIO_FUNC1_CHIPCLKCSR,
			clkset, &err);
	if (err) {
		DHD_ERROR(("%s: error writing for HT off\n", __func__));
=======
	brcmf_sdcard_cfg_write(bus->card, SDIO_FUNC_1, SBSDIO_FUNC1_CHIPCLKCSR,
			clkset, &err);
	if (err) {
		BRCMF_ERROR(("%s: error writing for HT off\n", __func__));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		goto fail;
	}

	/* If register supported, wait for ALPAvail and then force ALP */
	/* This may take up to 15 milliseconds */
<<<<<<< HEAD
	clkval = bcmsdh_cfg_read(bus->sdh, SDIO_FUNC_1,
			SBSDIO_FUNC1_CHIPCLKCSR, NULL);
	if ((clkval & ~SBSDIO_AVBITS) == clkset) {
		SPINWAIT(((clkval =
				bcmsdh_cfg_read(bus->sdh, SDIO_FUNC_1,
=======
	clkval = brcmf_sdcard_cfg_read(bus->card, SDIO_FUNC_1,
			SBSDIO_FUNC1_CHIPCLKCSR, NULL);
	if ((clkval & ~SBSDIO_AVBITS) == clkset) {
		SPINWAIT(((clkval =
				brcmf_sdcard_cfg_read(bus->card, SDIO_FUNC_1,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
						SBSDIO_FUNC1_CHIPCLKCSR,
						NULL)),
				!SBSDIO_ALPAV(clkval)),
				PMU_MAX_TRANSITION_DLY);
		if (!SBSDIO_ALPAV(clkval)) {
<<<<<<< HEAD
			DHD_ERROR(("%s: timeout on ALPAV wait, clkval 0x%02x\n",
				__func__, clkval));
=======
			BRCMF_ERROR(("%s: timeout on ALPAV wait,"
				     " clkval 0x%02x\n", __func__, clkval));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			err = -EBUSY;
			goto fail;
		}
		clkset = SBSDIO_FORCE_HW_CLKREQ_OFF |
				SBSDIO_FORCE_ALP;
<<<<<<< HEAD
		bcmsdh_cfg_write(bus->sdh, SDIO_FUNC_1,
=======
		brcmf_sdcard_cfg_write(bus->card, SDIO_FUNC_1,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				SBSDIO_FUNC1_CHIPCLKCSR,
				clkset, &err);
		udelay(65);
	} else {
<<<<<<< HEAD
		DHD_ERROR(("%s: ChipClkCSR access: wrote 0x%02x read 0x%02x\n",
			__func__, clkset, clkval));
=======
		BRCMF_ERROR(("%s: ChipClkCSR access: wrote 0x%02x"
			     " read 0x%02x\n", __func__, clkset, clkval));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		err = -EACCES;
		goto fail;
	}

	/* Also, disable the extra SDIO pull-ups */
<<<<<<< HEAD
	bcmsdh_cfg_write(bus->sdh, SDIO_FUNC_1, SBSDIO_FUNC1_SDIOPULLUP, 0,
			 NULL);

	err = dhdsdio_chip_recognition(bus->sdh, ci, regs);
=======
	brcmf_sdcard_cfg_write(bus->card, SDIO_FUNC_1, SBSDIO_FUNC1_SDIOPULLUP,
			       0, NULL);

	err = brcmf_sdbrcm_chip_recognition(bus->card, ci, regs);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	if (err)
		goto fail;

	/*
	 * Make sure any on-chip ARM is off (in case strapping is wrong),
	 * or downloaded code was already running.
	 */
<<<<<<< HEAD
	dhdsdio_chip_disablecore(bus->sdh, ci->armcorebase);

	bcmsdh_reg_write(bus->sdh,
		CORE_CC_REG(ci->cccorebase, gpiopullup), 4, 0);
	bcmsdh_reg_write(bus->sdh,
		CORE_CC_REG(ci->cccorebase, gpiopulldown), 4, 0);

	/* Disable F2 to clear any intermediate frame state on the dongle */
	bcmsdh_cfg_write(bus->sdh, SDIO_FUNC_0, SDIOD_CCCR_IOEN,
		SDIO_FUNC_ENABLE_1, NULL);

	/* WAR: cmd52 backplane read so core HW will drop ALPReq */
	clkval = bcmsdh_cfg_read(bus->sdh, SDIO_FUNC_1,
			0, NULL);

	/* Done with backplane-dependent accesses, can drop clock... */
	bcmsdh_cfg_write(bus->sdh, SDIO_FUNC_1, SBSDIO_FUNC1_CHIPCLKCSR, 0,
			 NULL);
=======
	brcmf_sdbrcm_chip_disablecore(bus->card, ci->armcorebase);

	brcmf_sdcard_reg_write(bus->card,
		CORE_CC_REG(ci->cccorebase, gpiopullup), 4, 0);
	brcmf_sdcard_reg_write(bus->card,
		CORE_CC_REG(ci->cccorebase, gpiopulldown), 4, 0);

	/* Disable F2 to clear any intermediate frame state on the dongle */
	brcmf_sdcard_cfg_write(bus->card, SDIO_FUNC_0, SDIO_CCCR_IOEx,
		SDIO_FUNC_ENABLE_1, NULL);

	/* WAR: cmd52 backplane read so core HW will drop ALPReq */
	clkval = brcmf_sdcard_cfg_read(bus->card, SDIO_FUNC_1,
			0, NULL);

	/* Done with backplane-dependent accesses, can drop clock... */
	brcmf_sdcard_cfg_write(bus->card, SDIO_FUNC_1, SBSDIO_FUNC1_CHIPCLKCSR,
			       0, NULL);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	bus->ci = ci;
	return 0;
fail:
	bus->ci = NULL;
	kfree(ci);
	return err;
}

static void
<<<<<<< HEAD
dhdsdio_chip_resetcore(bcmsdh_info_t *sdh, u32 corebase)
=======
brcmf_sdbrcm_chip_resetcore(struct brcmf_sdio_card *card, u32 corebase)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
{
	u32 regdata;

	/*
	 * Must do the disable sequence first to work for
	 * arbitrary current core state.
	 */
<<<<<<< HEAD
	dhdsdio_chip_disablecore(sdh, corebase);
=======
	brcmf_sdbrcm_chip_disablecore(card, corebase);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	/*
	 * Now do the initialization sequence.
	 * set reset while enabling the clock and
	 * forcing them on throughout the core
	 */
<<<<<<< HEAD
	bcmsdh_reg_write(sdh, CORE_SB(corebase, sbtmstatelow), 4,
=======
	brcmf_sdcard_reg_write(card, CORE_SB(corebase, sbtmstatelow), 4,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		((SICF_FGC | SICF_CLOCK_EN) << SBTML_SICF_SHIFT) |
		SBTML_RESET);
	udelay(1);

<<<<<<< HEAD
	regdata = bcmsdh_reg_read(sdh, CORE_SB(corebase, sbtmstatehigh), 4);
	if (regdata & SBTMH_SERR)
		bcmsdh_reg_write(sdh, CORE_SB(corebase, sbtmstatehigh), 4, 0);

	regdata = bcmsdh_reg_read(sdh, CORE_SB(corebase, sbimstate), 4);
	if (regdata & (SBIM_IBE | SBIM_TO))
		bcmsdh_reg_write(sdh, CORE_SB(corebase, sbimstate), 4,
			regdata & ~(SBIM_IBE | SBIM_TO));

	/* clear reset and allow it to propagate throughout the core */
	bcmsdh_reg_write(sdh, CORE_SB(corebase, sbtmstatelow), 4,
=======
	regdata = brcmf_sdcard_reg_read(card, CORE_SB(corebase, sbtmstatehigh),
					4);
	if (regdata & SBTMH_SERR)
		brcmf_sdcard_reg_write(card, CORE_SB(corebase, sbtmstatehigh),
				       4, 0);

	regdata = brcmf_sdcard_reg_read(card, CORE_SB(corebase, sbimstate), 4);
	if (regdata & (SBIM_IBE | SBIM_TO))
		brcmf_sdcard_reg_write(card, CORE_SB(corebase, sbimstate), 4,
			regdata & ~(SBIM_IBE | SBIM_TO));

	/* clear reset and allow it to propagate throughout the core */
	brcmf_sdcard_reg_write(card, CORE_SB(corebase, sbtmstatelow), 4,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		(SICF_FGC << SBTML_SICF_SHIFT) |
		(SICF_CLOCK_EN << SBTML_SICF_SHIFT));
	udelay(1);

	/* leave clock enabled */
<<<<<<< HEAD
	bcmsdh_reg_write(sdh, CORE_SB(corebase, sbtmstatelow), 4,
=======
	brcmf_sdcard_reg_write(card, CORE_SB(corebase, sbtmstatelow), 4,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		(SICF_CLOCK_EN << SBTML_SICF_SHIFT));
	udelay(1);
}

/* SDIO Pad drive strength to select value mappings */
struct sdiod_drive_str {
	u8 strength;	/* Pad Drive Strength in mA */
	u8 sel;		/* Chip-specific select value */
};

/* SDIO Drive Strength to sel value table for PMU Rev 1 */
static const struct sdiod_drive_str sdiod_drive_strength_tab1[] = {
	{
	4, 0x2}, {
	2, 0x3}, {
	1, 0x0}, {
	0, 0x0}
	};

/* SDIO Drive Strength to sel value table for PMU Rev 2, 3 */
static const struct sdiod_drive_str sdiod_drive_strength_tab2[] = {
	{
	12, 0x7}, {
	10, 0x6}, {
	8, 0x5}, {
	6, 0x4}, {
	4, 0x2}, {
	2, 0x1}, {
	0, 0x0}
	};

/* SDIO Drive Strength to sel value table for PMU Rev 8 (1.8V) */
static const struct sdiod_drive_str sdiod_drive_strength_tab3[] = {
	{
	32, 0x7}, {
	26, 0x6}, {
	22, 0x5}, {
	16, 0x4}, {
	12, 0x3}, {
	8, 0x2}, {
	4, 0x1}, {
	0, 0x0}
	};

#define SDIOD_DRVSTR_KEY(chip, pmu)     (((chip) << 16) | (pmu))

static void
<<<<<<< HEAD
dhdsdio_sdiod_drive_strength_init(struct dhd_bus *bus, u32 drivestrength) {
=======
brcmf_sdbrcm_sdiod_drive_strength_init(struct brcmf_bus *bus, u32 drivestrength) {
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	struct sdiod_drive_str *str_tab = NULL;
	u32 str_mask = 0;
	u32 str_shift = 0;
	char chn[8];

	if (!(bus->ci->cccaps & CC_CAP_PMU))
		return;

	switch (SDIOD_DRVSTR_KEY(bus->ci->chip, bus->ci->pmurev)) {
	case SDIOD_DRVSTR_KEY(BCM4325_CHIP_ID, 1):
		str_tab = (struct sdiod_drive_str *)&sdiod_drive_strength_tab1;
		str_mask = 0x30000000;
		str_shift = 28;
		break;
	case SDIOD_DRVSTR_KEY(BCM4325_CHIP_ID, 2):
	case SDIOD_DRVSTR_KEY(BCM4325_CHIP_ID, 3):
		str_tab = (struct sdiod_drive_str *)&sdiod_drive_strength_tab2;
		str_mask = 0x00003800;
		str_shift = 11;
		break;
	case SDIOD_DRVSTR_KEY(BCM4336_CHIP_ID, 8):
		str_tab = (struct sdiod_drive_str *)&sdiod_drive_strength_tab3;
		str_mask = 0x00003800;
		str_shift = 11;
		break;
	default:
<<<<<<< HEAD
		DHD_ERROR(("No SDIO Drive strength init"
			"done for chip %s rev %d pmurev %d\n",
			bcm_chipname(bus->ci->chip, chn, 8),
			bus->ci->chiprev, bus->ci->pmurev));
=======
		BRCMF_ERROR(("No SDIO Drive strength init"
			     "done for chip %s rev %d pmurev %d\n",
			     brcmu_chipname(bus->ci->chip, chn, 8),
			     bus->ci->chiprev, bus->ci->pmurev));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		break;
	}

	if (str_tab != NULL) {
		u32 drivestrength_sel = 0;
		u32 cc_data_temp;
		int i;

		for (i = 0; str_tab[i].strength != 0; i++) {
			if (drivestrength >= str_tab[i].strength) {
				drivestrength_sel = str_tab[i].sel;
				break;
			}
		}

<<<<<<< HEAD
		bcmsdh_reg_write(bus->sdh,
			CORE_CC_REG(bus->ci->cccorebase, chipcontrol_addr),
			4, 1);
		cc_data_temp = bcmsdh_reg_read(bus->sdh,
=======
		brcmf_sdcard_reg_write(bus->card,
			CORE_CC_REG(bus->ci->cccorebase, chipcontrol_addr),
			4, 1);
		cc_data_temp = brcmf_sdcard_reg_read(bus->card,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			CORE_CC_REG(bus->ci->cccorebase, chipcontrol_addr), 4);
		cc_data_temp &= ~str_mask;
		drivestrength_sel <<= str_shift;
		cc_data_temp |= drivestrength_sel;
<<<<<<< HEAD
		bcmsdh_reg_write(bus->sdh,
			CORE_CC_REG(bus->ci->cccorebase, chipcontrol_addr),
			4, cc_data_temp);

		DHD_INFO(("SDIO: %dmA drive strength selected, set to 0x%08x\n",
			drivestrength, cc_data_temp));
=======
		brcmf_sdcard_reg_write(bus->card,
			CORE_CC_REG(bus->ci->cccorebase, chipcontrol_addr),
			4, cc_data_temp);

		BRCMF_INFO(("SDIO: %dmA drive strength selected, "
			    "set to 0x%08x\n", drivestrength, cc_data_temp));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	}
}

static void
<<<<<<< HEAD
dhdsdio_chip_detach(struct dhd_bus *bus)
{
	DHD_TRACE(("%s: Enter\n", __func__));
=======
brcmf_sdbrcm_chip_detach(struct brcmf_bus *bus)
{
	BRCMF_TRACE(("%s: Enter\n", __func__));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	kfree(bus->ci);
	bus->ci = NULL;
}
<<<<<<< HEAD
=======

static void
brcmf_sdbrcm_wait_for_event(struct brcmf_bus *bus, bool *lockvar)
{
	brcmf_sdbrcm_sdunlock(bus);
	wait_event_interruptible_timeout(bus->ctrl_wait,
					 (*lockvar == false), HZ * 2);
	brcmf_sdbrcm_sdlock(bus);
	return;
}

static void
brcmf_sdbrcm_wait_event_wakeup(struct brcmf_bus *bus)
{
	if (waitqueue_active(&bus->ctrl_wait))
		wake_up_interruptible(&bus->ctrl_wait);
	return;
}

static int
brcmf_sdbrcm_watchdog_thread(void *data)
{
	struct brcmf_bus *bus = (struct brcmf_bus *)data;

	/* This thread doesn't need any user-level access,
	* so get rid of all our resources
	*/
	if (brcmf_watchdog_prio > 0) {
		struct sched_param param;
		param.sched_priority = (brcmf_watchdog_prio < MAX_RT_PRIO) ?
				       brcmf_watchdog_prio : (MAX_RT_PRIO - 1);
		sched_setscheduler(current, SCHED_FIFO, &param);
	}

	allow_signal(SIGTERM);
	/* Run until signal received */
	while (1) {
		if (kthread_should_stop())
			break;
		if (!wait_for_completion_interruptible(&bus->watchdog_wait)) {
			if (bus->drvr->dongle_reset == false)
				brcmf_sdbrcm_bus_watchdog(bus->drvr);
			/* Count the tick for reference */
			bus->drvr->tickcnt++;
		} else
			break;
	}
	return 0;
}

static void
brcmf_sdbrcm_watchdog(unsigned long data)
{
	struct brcmf_bus *bus = (struct brcmf_bus *)data;

	if (brcmf_watchdog_prio >= 0) {
		if (bus->watchdog_tsk)
			complete(&bus->watchdog_wait);
		else
			return;
	} else {
		brcmf_sdbrcm_bus_watchdog(bus->drvr);

		/* Count the tick for reference */
		bus->drvr->tickcnt++;
	}

	/* Reschedule the watchdog */
	if (bus->wd_timer_valid)
		mod_timer(&bus->timer, jiffies + brcmf_watchdog_ms * HZ / 1000);
}

void
brcmf_sdbrcm_wd_timer(struct brcmf_bus *bus, uint wdtick)
{
	static uint save_ms;

	/* don't start the wd until fw is loaded */
	if (bus->drvr->busstate == BRCMF_BUS_DOWN)
		return;

	/* Totally stop the timer */
	if (!wdtick && bus->wd_timer_valid == true) {
		del_timer_sync(&bus->timer);
		bus->wd_timer_valid = false;
		save_ms = wdtick;
		return;
	}

	if (wdtick) {
		brcmf_watchdog_ms = (uint) wdtick;

		if (save_ms != brcmf_watchdog_ms) {
			if (bus->wd_timer_valid == true)
				/* Stop timer and restart at new value */
				del_timer_sync(&bus->timer);

			/* Create timer again when watchdog period is
			   dynamically changed or in the first instance
			 */
			bus->timer.expires =
				jiffies + brcmf_watchdog_ms * HZ / 1000;
			add_timer(&bus->timer);

		} else {
			/* Re arm the timer, at last watchdog period */
			mod_timer(&bus->timer,
				jiffies + brcmf_watchdog_ms * HZ / 1000);
		}

		bus->wd_timer_valid = true;
		save_ms = wdtick;
	}
}

static int brcmf_sdbrcm_dpc_thread(void *data)
{
	struct brcmf_bus *bus = (struct brcmf_bus *) data;

	/* This thread doesn't need any user-level access,
	 * so get rid of all our resources
	 */
	if (brcmf_dpc_prio > 0) {
		struct sched_param param;
		param.sched_priority = (brcmf_dpc_prio < MAX_RT_PRIO) ?
				       brcmf_dpc_prio : (MAX_RT_PRIO - 1);
		sched_setscheduler(current, SCHED_FIFO, &param);
	}

	allow_signal(SIGTERM);
	/* Run until signal received */
	while (1) {
		if (kthread_should_stop())
			break;
		if (!wait_for_completion_interruptible(&bus->dpc_wait)) {
			/* Call bus dpc unless it indicated down
			(then clean stop) */
			if (bus->drvr->busstate != BRCMF_BUS_DOWN) {
				if (brcmf_sdbrcm_dpc(bus))
					complete(&bus->dpc_wait);
			} else {
				brcmf_sdbrcm_bus_stop(bus, true);
			}
		} else
			break;
	}
	return 0;
}

static void brcmf_sdbrcm_dpc_tasklet(unsigned long data)
{
	struct brcmf_bus *bus = (struct brcmf_bus *) data;

	/* Call bus dpc unless it indicated down (then clean stop) */
	if (bus->drvr->busstate != BRCMF_BUS_DOWN) {
		if (brcmf_sdbrcm_dpc(bus))
			tasklet_schedule(&bus->tasklet);
	} else
		brcmf_sdbrcm_bus_stop(bus, true);
}

static void brcmf_sdbrcm_sched_dpc(struct brcmf_bus *bus)
{
	if (bus->dpc_tsk) {
		complete(&bus->dpc_wait);
		return;
	}

	tasklet_schedule(&bus->tasklet);
}

static void brcmf_sdbrcm_sdlock(struct brcmf_bus *bus)
{
	if (bus->threads_only)
		down(&bus->sdsem);
	else
		spin_lock_bh(&bus->sdlock);
}

static void brcmf_sdbrcm_sdunlock(struct brcmf_bus *bus)
{
	if (bus->threads_only)
		up(&bus->sdsem);
	else
		spin_unlock_bh(&bus->sdlock);
}

static int brcmf_sdbrcm_get_image(char *buf, int len, struct brcmf_bus *bus)
{
	if (bus->firmware->size < bus->fw_ptr + len)
		len = bus->firmware->size - bus->fw_ptr;

	memcpy(buf, &bus->firmware->data[bus->fw_ptr], len);
	bus->fw_ptr += len;
	return len;
}

MODULE_FIRMWARE(BCM4329_FW_NAME);
MODULE_FIRMWARE(BCM4329_NV_NAME);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
