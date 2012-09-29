/***************************************************************************
 *   Copyright (C) 2006-2010 by Marin Mitov                                *
 *   mitov@issp.bas.bg                                                     *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <linux/version.h>
#include <linux/stringify.h>
#include <linux/delay.h>
#include <linux/kthread.h>
<<<<<<< HEAD
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf-dma-contig.h>
=======
#include <linux/slab.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

#include "dt3155v4l.h"

#define DT3155_VENDOR_ID 0x8086
#define DT3155_DEVICE_ID 0x1223

/* DT3155_CHUNK_SIZE is 4M (2^22) 8 full size buffers */
#define DT3155_CHUNK_SIZE (1U << 22)

#define DT3155_COH_FLAGS (GFP_KERNEL | GFP_DMA32 | __GFP_COLD | __GFP_NOWARN)

#define DT3155_BUF_SIZE (768 * 576)

<<<<<<< HEAD
=======
#ifdef CONFIG_DT3155_STREAMING
#define DT3155_CAPTURE_METHOD V4L2_CAP_STREAMING
#else
#define DT3155_CAPTURE_METHOD V4L2_CAP_READWRITE
#endif

>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
/*  global initializers (for all boards)  */
#ifdef CONFIG_DT3155_CCIR
static const u8 csr2_init = VT_50HZ;
#define DT3155_CURRENT_NORM V4L2_STD_625_50
static const unsigned int img_width = 768;
static const unsigned int img_height = 576;
static const unsigned int frames_per_sec = 25;
static const struct v4l2_fmtdesc frame_std[] = {
	{
	.index = 0,
	.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
	.flags = 0,
	.description = "CCIR/50Hz 8 bits gray",
	.pixelformat = V4L2_PIX_FMT_GREY,
	},
};
#else
static const u8 csr2_init = VT_60HZ;
#define DT3155_CURRENT_NORM V4L2_STD_525_60
static const unsigned int img_width = 640;
static const unsigned int img_height = 480;
static const unsigned int frames_per_sec = 30;
static const struct v4l2_fmtdesc frame_std[] = {
	{
	.index = 0,
	.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
	.flags = 0,
	.description = "RS-170/60Hz 8 bits gray",
	.pixelformat = V4L2_PIX_FMT_GREY,
	},
};
#endif

#define NUM_OF_FORMATS ARRAY_SIZE(frame_std)

static u8 config_init = ACQ_MODE_EVEN;

/**
 * read_i2c_reg - reads an internal i2c register
 *
 * @addr:	dt3155 mmio base address
 * @index:	index (internal address) of register to read
 * @data:	pointer to byte the read data will be placed in
 *
 * returns:	zero on success or error code
 *
 * This function starts reading the specified (by index) register
 * and busy waits for the process to finish. The result is placed
 * in a byte pointed by data.
 */
static int
read_i2c_reg(void __iomem *addr, u8 index, u8 *data)
{
	u32 tmp = index;

	iowrite32((tmp<<17) | IIC_READ, addr + IIC_CSR2);
	mmiowb();
	udelay(45); /* wait at least 43 usec for NEW_CYCLE to clear */
	if (ioread32(addr + IIC_CSR2) & NEW_CYCLE) {
		/* error: NEW_CYCLE not cleared */
		printk(KERN_ERR "dt3155: NEW_CYCLE not cleared\n");
		return -EIO;
	}
	tmp = ioread32(addr + IIC_CSR1);
	if (tmp & DIRECT_ABORT) {
		/* error: DIRECT_ABORT set */
		printk(KERN_ERR "dt3155: DIRECT_ABORT set\n");
		/* reset DIRECT_ABORT bit */
		iowrite32(DIRECT_ABORT, addr + IIC_CSR1);
		return -EIO;
	}
	*data = tmp>>24;
	return 0;
}

/**
 * write_i2c_reg - writes to an internal i2c register
 *
 * @addr:	dt3155 mmio base address
 * @index:	index (internal address) of register to read
 * @data:	data to be written
 *
 * returns:	zero on success or error code
 *
 * This function starts writting the specified (by index) register
 * and busy waits for the process to finish.
 */
static int
write_i2c_reg(void __iomem *addr, u8 index, u8 data)
{
	u32 tmp = index;

	iowrite32((tmp<<17) | IIC_WRITE | data, addr + IIC_CSR2);
	mmiowb();
	udelay(65); /* wait at least 63 usec for NEW_CYCLE to clear */
	if (ioread32(addr + IIC_CSR2) & NEW_CYCLE) {
		/* error: NEW_CYCLE not cleared */
		printk(KERN_ERR "dt3155: NEW_CYCLE not cleared\n");
		return -EIO;
	}
	if (ioread32(addr + IIC_CSR1) & DIRECT_ABORT) {
		/* error: DIRECT_ABORT set */
		printk(KERN_ERR "dt3155: DIRECT_ABORT set\n");
		/* reset DIRECT_ABORT bit */
		iowrite32(DIRECT_ABORT, addr + IIC_CSR1);
		return -EIO;
	}
	return 0;
}

/**
 * write_i2c_reg_nowait - writes to an internal i2c register
 *
 * @addr:	dt3155 mmio base address
 * @index:	index (internal address) of register to read
 * @data:	data to be written
 *
 * This function starts writting the specified (by index) register
 * and then returns.
 */
static void write_i2c_reg_nowait(void __iomem *addr, u8 index, u8 data)
{
	u32 tmp = index;

	iowrite32((tmp<<17) | IIC_WRITE | data, addr + IIC_CSR2);
	mmiowb();
}

/**
 * wait_i2c_reg - waits the read/write to finish
 *
 * @addr:	dt3155 mmio base address
 *
 * returns:	zero on success or error code
 *
 * This function waits reading/writting to finish.
 */
static int wait_i2c_reg(void __iomem *addr)
{
	if (ioread32(addr + IIC_CSR2) & NEW_CYCLE)
		udelay(65); /* wait at least 63 usec for NEW_CYCLE to clear */
	if (ioread32(addr + IIC_CSR2) & NEW_CYCLE) {
		/* error: NEW_CYCLE not cleared */
		printk(KERN_ERR "dt3155: NEW_CYCLE not cleared\n");
		return -EIO;
	}
	if (ioread32(addr + IIC_CSR1) & DIRECT_ABORT) {
		/* error: DIRECT_ABORT set */
		printk(KERN_ERR "dt3155: DIRECT_ABORT set\n");
		/* reset DIRECT_ABORT bit */
		iowrite32(DIRECT_ABORT, addr + IIC_CSR1);
		return -EIO;
	}
	return 0;
}

static int
dt3155_start_acq(struct dt3155_priv *pd)
{
<<<<<<< HEAD
	struct videobuf_buffer *vb = pd->curr_buf;
	dma_addr_t dma_addr;

	dma_addr = videobuf_to_dma_contig(vb);
	iowrite32(dma_addr, pd->regs + EVEN_DMA_START);
	iowrite32(dma_addr + vb->width, pd->regs + ODD_DMA_START);
	iowrite32(vb->width, pd->regs + EVEN_DMA_STRIDE);
	iowrite32(vb->width, pd->regs + ODD_DMA_STRIDE);
=======
	struct vb2_buffer *vb = pd->curr_buf;
	dma_addr_t dma_addr;

	dma_addr = vb2_dma_contig_plane_paddr(vb, 0);
	iowrite32(dma_addr, pd->regs + EVEN_DMA_START);
	iowrite32(dma_addr + img_width, pd->regs + ODD_DMA_START);
	iowrite32(img_width, pd->regs + EVEN_DMA_STRIDE);
	iowrite32(img_width, pd->regs + ODD_DMA_STRIDE);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	/* enable interrupts, clear all irq flags */
	iowrite32(FLD_START_EN | FLD_END_ODD_EN | FLD_START |
			FLD_END_EVEN | FLD_END_ODD, pd->regs + INT_CSR);
	iowrite32(FIFO_EN | SRST | FLD_CRPT_ODD | FLD_CRPT_EVEN |
		  FLD_DN_ODD | FLD_DN_EVEN | CAP_CONT_EVEN | CAP_CONT_ODD,
							pd->regs + CSR1);
	wait_i2c_reg(pd->regs);
	write_i2c_reg(pd->regs, CONFIG, pd->config);
	write_i2c_reg(pd->regs, EVEN_CSR, CSR_ERROR | CSR_DONE);
	write_i2c_reg(pd->regs, ODD_CSR, CSR_ERROR | CSR_DONE);

	/*  start the board  */
	write_i2c_reg(pd->regs, CSR2, pd->csr2 | BUSY_EVEN | BUSY_ODD);
	return 0; /* success  */
}

<<<<<<< HEAD
static int
dt3155_stop_acq(struct dt3155_priv *pd)
{
	int tmp;

	/*  stop the board  */
	wait_i2c_reg(pd->regs);
	write_i2c_reg(pd->regs, CSR2, pd->csr2);

	/* disable all irqs, clear all irq flags */
	iowrite32(FLD_START | FLD_END_EVEN | FLD_END_ODD, pd->regs + INT_CSR);
	write_i2c_reg(pd->regs, EVEN_CSR, CSR_ERROR | CSR_DONE);
	write_i2c_reg(pd->regs, ODD_CSR, CSR_ERROR | CSR_DONE);
	tmp = ioread32(pd->regs + CSR1) & (FLD_CRPT_EVEN | FLD_CRPT_ODD);
	if (tmp)
		printk(KERN_ERR "dt3155: corrupted field %u\n", tmp);
	iowrite32(FIFO_EN | SRST | FLD_CRPT_ODD | FLD_CRPT_EVEN |
		  FLD_DN_ODD | FLD_DN_EVEN | CAP_CONT_EVEN | CAP_CONT_ODD,
							pd->regs + CSR1);
	return 0;
}

/* Locking: Caller holds q->vb_lock */
static int
dt3155_buf_setup(struct videobuf_queue *q, unsigned int *count,
							unsigned int *size)
{
	*size = img_width * img_height;
	return 0;
}

/* Locking: Caller holds q->vb_lock */
static int
dt3155_buf_prepare(struct videobuf_queue *q, struct videobuf_buffer *vb,
							enum v4l2_field field)
{
	int ret = 0;

	vb->width = img_width;
	vb->height = img_height;
	vb->size = img_width * img_height;
	vb->field = field;
	if (vb->state == VIDEOBUF_NEEDS_INIT)
		ret = videobuf_iolock(q, vb, NULL);
	if (ret) {
		vb->state = VIDEOBUF_ERROR;
		printk(KERN_ERR "ERROR: videobuf_iolock() failed\n");
		videobuf_dma_contig_free(q, vb); /* FIXME: needed? */
	} else
		vb->state = VIDEOBUF_PREPARED;
	return ret;
}

/* Locking: Caller holds q->vb_lock & q->irqlock */
static void
dt3155_buf_queue(struct videobuf_queue *q, struct videobuf_buffer *vb)
{
	struct dt3155_priv *pd = q->priv_data;

	if (vb->state != VIDEOBUF_NEEDS_INIT) {
		vb->state = VIDEOBUF_QUEUED;
		list_add_tail(&vb->queue, &pd->dmaq);
		wake_up_interruptible_sync(&pd->do_dma);
	} else
		vb->state = VIDEOBUF_ERROR;
}

/* Locking: Caller holds q->vb_lock */
static void
dt3155_buf_release(struct videobuf_queue *q, struct videobuf_buffer *vb)
{
	if (vb->state == VIDEOBUF_ACTIVE)
		videobuf_waiton(q, vb, 0, 0); /* FIXME: cannot be interrupted */
	videobuf_dma_contig_free(q, vb);
	vb->state = VIDEOBUF_NEEDS_INIT;
}

static struct videobuf_queue_ops vbq_ops = {
	.buf_setup = dt3155_buf_setup,
	.buf_prepare = dt3155_buf_prepare,
	.buf_queue = dt3155_buf_queue,
	.buf_release = dt3155_buf_release,
=======
/*
 *	driver-specific callbacks (vb2_ops)
 */
static int
dt3155_queue_setup(struct vb2_queue *q, unsigned int *num_buffers,
			unsigned int *num_planes, unsigned long sizes[],
						void *alloc_ctxs[])
{
	struct dt3155_priv *pd = vb2_get_drv_priv(q);
	void *ret;

	if (*num_buffers == 0)
		*num_buffers = 1;
	*num_planes = 1;
	sizes[0] = img_width * img_height;
	if (pd->q->alloc_ctx[0])
		return 0;
	ret = vb2_dma_contig_init_ctx(&pd->pdev->dev);
	if (IS_ERR(ret))
		return PTR_ERR(ret);
	pd->q->alloc_ctx[0] = ret;
	return 0;
}

static void
dt3155_wait_prepare(struct vb2_queue *q)
{
	struct dt3155_priv *pd = vb2_get_drv_priv(q);

	mutex_unlock(pd->vdev->lock);
}

static void
dt3155_wait_finish(struct vb2_queue *q)
{
	struct dt3155_priv *pd = vb2_get_drv_priv(q);

	mutex_lock(pd->vdev->lock);
}

static int
dt3155_buf_prepare(struct vb2_buffer *vb)
{
	vb2_set_plane_payload(vb, 0, img_width * img_height);
	return 0;
}

static int
dt3155_start_streaming(struct vb2_queue *q)
{
	return 0;
}

static int
dt3155_stop_streaming(struct vb2_queue *q)
{
	struct dt3155_priv *pd = vb2_get_drv_priv(q);
	struct vb2_buffer *vb;

	spin_lock_irq(&pd->lock);
	while (!list_empty(&pd->dmaq)) {
		vb = list_first_entry(&pd->dmaq, typeof(*vb), done_entry);
		list_del(&vb->done_entry);
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irq(&pd->lock);
	msleep(45); /* irq hendler will stop the hardware */
	return 0;
}

static void
dt3155_buf_queue(struct vb2_buffer *vb)
{
	struct dt3155_priv *pd = vb2_get_drv_priv(vb->vb2_queue);

	/*  pd->q->streaming = 1 when dt3155_buf_queue() is invoked  */
	spin_lock_irq(&pd->lock);
	if (pd->curr_buf)
		list_add_tail(&vb->done_entry, &pd->dmaq);
	else {
		pd->curr_buf = vb;
		dt3155_start_acq(pd);
	}
	spin_unlock_irq(&pd->lock);
}
/*
 *	end driver-specific callbacks
 */

const struct vb2_ops q_ops = {
	.queue_setup = dt3155_queue_setup,
	.wait_prepare = dt3155_wait_prepare,
	.wait_finish = dt3155_wait_finish,
	.buf_prepare = dt3155_buf_prepare,
	.start_streaming = dt3155_start_streaming,
	.stop_streaming = dt3155_stop_streaming,
	.buf_queue = dt3155_buf_queue,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
};

static irqreturn_t
dt3155_irq_handler_even(int irq, void *dev_id)
{
	struct dt3155_priv *ipd = dev_id;
<<<<<<< HEAD
	struct videobuf_buffer *ivb;
=======
	struct vb2_buffer *ivb;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	dma_addr_t dma_addr;
	u32 tmp;

	tmp = ioread32(ipd->regs + INT_CSR) & (FLD_START | FLD_END_ODD);
	if (!tmp)
		return IRQ_NONE;  /* not our irq */
	if ((tmp & FLD_START) && !(tmp & FLD_END_ODD)) {
		iowrite32(FLD_START_EN | FLD_END_ODD_EN | FLD_START,
							ipd->regs + INT_CSR);
		ipd->field_count++;
		return IRQ_HANDLED; /* start of field irq */
	}
	if ((tmp & FLD_START) && (tmp & FLD_END_ODD)) {
		if (!ipd->stats.start_before_end++)
			printk(KERN_ERR "dt3155: irq: START before END\n");
	}
	/*	check for corrupted fields     */
/*	write_i2c_reg(ipd->regs, EVEN_CSR, CSR_ERROR | CSR_DONE);	*/
/*	write_i2c_reg(ipd->regs, ODD_CSR, CSR_ERROR | CSR_DONE);	*/
	tmp = ioread32(ipd->regs + CSR1) & (FLD_CRPT_EVEN | FLD_CRPT_ODD);
	if (tmp) {
		if (!ipd->stats.corrupted_fields++)
			printk(KERN_ERR "dt3155: corrupted field %u\n", tmp);
		iowrite32(FIFO_EN | SRST | FLD_CRPT_ODD | FLD_CRPT_EVEN |
						FLD_DN_ODD | FLD_DN_EVEN |
						CAP_CONT_EVEN | CAP_CONT_ODD,
							ipd->regs + CSR1);
		mmiowb();
	}

	spin_lock(&ipd->lock);
<<<<<<< HEAD
	if (ipd->curr_buf && ipd->curr_buf->state == VIDEOBUF_ACTIVE) {
		if (waitqueue_active(&ipd->curr_buf->done)) {
			do_gettimeofday(&ipd->curr_buf->ts);
			ipd->curr_buf->field_count = ipd->field_count;
			ipd->curr_buf->state = VIDEOBUF_DONE;
			wake_up(&ipd->curr_buf->done);
		} else {
			ivb = ipd->curr_buf;
			goto load_dma;
		}
	} else
		goto stop_dma;
	if (list_empty(&ipd->dmaq))
		goto stop_dma;
	ivb = list_first_entry(&ipd->dmaq, typeof(*ivb), queue);
	list_del(&ivb->queue);
	if (ivb->state == VIDEOBUF_QUEUED) {
		ivb->state = VIDEOBUF_ACTIVE;
		ipd->curr_buf = ivb;
	} else
		goto stop_dma;
load_dma:
	dma_addr = videobuf_to_dma_contig(ivb);
	iowrite32(dma_addr, ipd->regs + EVEN_DMA_START);
	iowrite32(dma_addr + ivb->width, ipd->regs + ODD_DMA_START);
	iowrite32(ivb->width, ipd->regs + EVEN_DMA_STRIDE);
	iowrite32(ivb->width, ipd->regs + ODD_DMA_STRIDE);
=======
	if (ipd->curr_buf) {
		do_gettimeofday(&ipd->curr_buf->v4l2_buf.timestamp);
		ipd->curr_buf->v4l2_buf.sequence = (ipd->field_count) >> 1;
		vb2_buffer_done(ipd->curr_buf, VB2_BUF_STATE_DONE);
	}

	if (!ipd->q->streaming || list_empty(&ipd->dmaq))
		goto stop_dma;
	ivb = list_first_entry(&ipd->dmaq, typeof(*ivb), done_entry);
	list_del(&ivb->done_entry);
	ipd->curr_buf = ivb;
	dma_addr = vb2_dma_contig_plane_paddr(ivb, 0);
	iowrite32(dma_addr, ipd->regs + EVEN_DMA_START);
	iowrite32(dma_addr + img_width, ipd->regs + ODD_DMA_START);
	iowrite32(img_width, ipd->regs + EVEN_DMA_STRIDE);
	iowrite32(img_width, ipd->regs + ODD_DMA_STRIDE);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	mmiowb();
	/* enable interrupts, clear all irq flags */
	iowrite32(FLD_START_EN | FLD_END_ODD_EN | FLD_START |
			FLD_END_EVEN | FLD_END_ODD, ipd->regs + INT_CSR);
	spin_unlock(&ipd->lock);
	return IRQ_HANDLED;

stop_dma:
	ipd->curr_buf = NULL;
	/* stop the board */
	write_i2c_reg_nowait(ipd->regs, CSR2, ipd->csr2);
<<<<<<< HEAD
=======
	iowrite32(FIFO_EN | SRST | FLD_CRPT_ODD | FLD_CRPT_EVEN |
		  FLD_DN_ODD | FLD_DN_EVEN, ipd->regs + CSR1);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	/* disable interrupts, clear all irq flags */
	iowrite32(FLD_START | FLD_END_EVEN | FLD_END_ODD, ipd->regs + INT_CSR);
	spin_unlock(&ipd->lock);
	return IRQ_HANDLED;
}

static int
<<<<<<< HEAD
dt3155_threadfn(void *arg)
{
	struct dt3155_priv *pd = arg;
	struct videobuf_buffer *vb;
	unsigned long flags;

	while (1) {
		wait_event_interruptible(pd->do_dma,
			kthread_should_stop() || !list_empty(&pd->dmaq));
		if (kthread_should_stop())
			break;

		spin_lock_irqsave(&pd->lock, flags);
		if (pd->curr_buf) /* dma is active */
			goto done;
		if (list_empty(&pd->dmaq)) /* no empty biffers */
			goto done;
		vb = list_first_entry(&pd->dmaq, typeof(*vb), queue);
		list_del(&vb->queue);
		if (vb->state == VIDEOBUF_QUEUED) {
			vb->state = VIDEOBUF_ACTIVE;
			pd->curr_buf = vb;
			spin_unlock_irqrestore(&pd->lock, flags);
			/* start dma */
			dt3155_start_acq(pd);
			continue;
		} else
			printk(KERN_DEBUG "%s(): This is a BUG\n", __func__);
done:
		spin_unlock_irqrestore(&pd->lock, flags);
	}
	return 0;
}

static int
=======
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
dt3155_open(struct file *filp)
{
	int ret = 0;
	struct dt3155_priv *pd = video_drvdata(filp);

<<<<<<< HEAD
	printk(KERN_INFO "dt3155: open(): minor: %i\n", pd->vdev->minor);

	if (mutex_lock_interruptible(&pd->mux) == -EINTR)
		return -ERESTARTSYS;
	if (!pd->users) {
		pd->vidq = kzalloc(sizeof(*pd->vidq), GFP_KERNEL);
		if (!pd->vidq) {
=======
	printk(KERN_INFO "dt3155: open(): minor: %i, users: %i\n",
						pd->vdev->minor, pd->users);

	if (!pd->users) {
		pd->q = kzalloc(sizeof(*pd->q), GFP_KERNEL);
		if (!pd->q) {
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			printk(KERN_ERR "dt3155: error: alloc queue\n");
			ret = -ENOMEM;
			goto err_alloc_queue;
		}
<<<<<<< HEAD
		videobuf_queue_dma_contig_init(pd->vidq, &vbq_ops,
				&pd->pdev->dev, &pd->lock,
				V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_FIELD_NONE,
				sizeof(struct videobuf_buffer), pd, NULL);
=======
		pd->q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		pd->q->io_modes = VB2_READ | VB2_MMAP;
		pd->q->ops = &q_ops;
		pd->q->mem_ops = &vb2_dma_contig_memops;
		pd->q->drv_priv = pd;
		pd->curr_buf = NULL;
		pd->field_count = 0;
		vb2_queue_init(pd->q); /* cannot fail */
		INIT_LIST_HEAD(&pd->dmaq);
		spin_lock_init(&pd->lock);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
		/* disable all irqs, clear all irq flags */
		iowrite32(FLD_START | FLD_END_EVEN | FLD_END_ODD,
						pd->regs + INT_CSR);
		pd->irq_handler = dt3155_irq_handler_even;
		ret = request_irq(pd->pdev->irq, pd->irq_handler,
						IRQF_SHARED, DT3155_NAME, pd);
		if (ret) {
			printk(KERN_ERR "dt3155: error: request_irq\n");
			goto err_request_irq;
		}
<<<<<<< HEAD
		pd->curr_buf = NULL;
		pd->thread = kthread_run(dt3155_threadfn, pd,
					"dt3155_thread_%i", pd->vdev->minor);
		if (IS_ERR(pd->thread)) {
			printk(KERN_ERR "dt3155: kthread_run() failed\n");
			ret = PTR_ERR(pd->thread);
			goto err_thread;
		}
		pd->field_count = 0;
	}
	pd->users++;
	goto done;
err_thread:
	free_irq(pd->pdev->irq, pd);
err_request_irq:
	kfree(pd->vidq);
	pd->vidq = NULL;
err_alloc_queue:
done:
	mutex_unlock(&pd->mux);
=======
	}
	pd->users++;
	return 0; /* success */
err_request_irq:
	kfree(pd->q);
	pd->q = NULL;
err_alloc_queue:
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	return ret;
}

static int
dt3155_release(struct file *filp)
{
	struct dt3155_priv *pd = video_drvdata(filp);
<<<<<<< HEAD
	struct videobuf_buffer *tmp;
	unsigned long flags;
	int ret = 0;

	printk(KERN_INFO "dt3155: release(): minor: %i\n", pd->vdev->minor);

	if (mutex_lock_interruptible(&pd->mux) == -EINTR)
		return -ERESTARTSYS;
	pd->users--;
	BUG_ON(pd->users < 0);
	if (pd->acq_fp == filp) {
		spin_lock_irqsave(&pd->lock, flags);
		INIT_LIST_HEAD(&pd->dmaq); /* queue is emptied */
		tmp = pd->curr_buf;
		spin_unlock_irqrestore(&pd->lock, flags);
		if (tmp)
			videobuf_waiton(pd->vidq, tmp, 0, 1); /* block, interruptible */
		dt3155_stop_acq(pd);
		videobuf_stop(pd->vidq);
		pd->acq_fp = NULL;
		pd->streaming = 0;
	}
	if (!pd->users) {
		kthread_stop(pd->thread);
		free_irq(pd->pdev->irq, pd);
		kfree(pd->vidq);
		pd->vidq = NULL;
	}
	mutex_unlock(&pd->mux);
	return ret;
=======

	printk(KERN_INFO "dt3155: release(): minor: %i, users: %i\n",
					pd->vdev->minor, pd->users - 1);

	pd->users--;
	BUG_ON(pd->users < 0);
	if (!pd->users) {
		vb2_queue_release(pd->q);
		free_irq(pd->pdev->irq, pd);
		if (pd->q->alloc_ctx[0])
			vb2_dma_contig_cleanup_ctx(pd->q->alloc_ctx[0]);
		kfree(pd->q);
		pd->q = NULL;
	}
	return 0;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
}

static ssize_t
dt3155_read(struct file *filp, char __user *user, size_t size, loff_t *loff)
{
	struct dt3155_priv *pd = video_drvdata(filp);
<<<<<<< HEAD
	int ret;

	if (mutex_lock_interruptible(&pd->mux) == -EINTR)
		return -ERESTARTSYS;
	if (!pd->acq_fp) {
		pd->acq_fp = filp;
		pd->streaming = 0;
	} else if (pd->acq_fp != filp) {
		ret = -EBUSY;
		goto done;
	} else if (pd->streaming == 1) {
		ret = -EINVAL;
		goto done;
	}
	ret = videobuf_read_stream(pd->vidq, user, size, loff, 0,
						filp->f_flags & O_NONBLOCK);
done:
	mutex_unlock(&pd->mux);
	return ret;
=======

	return vb2_read(pd->q, user, size, loff, filp->f_flags & O_NONBLOCK);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
}

static unsigned int
dt3155_poll(struct file *filp, struct poll_table_struct *polltbl)
{
	struct dt3155_priv *pd = video_drvdata(filp);

<<<<<<< HEAD
	return videobuf_poll_stream(filp, pd->vidq, polltbl);
=======
	return vb2_poll(pd->q, filp, polltbl);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
}

static int
dt3155_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct dt3155_priv *pd = video_drvdata(filp);

<<<<<<< HEAD
	return videobuf_mmap_mapper(pd->vidq, vma);
=======
	return vb2_mmap(pd->q, vma);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
}

static const struct v4l2_file_operations dt3155_fops = {
	.owner = THIS_MODULE,
	.open = dt3155_open,
	.release = dt3155_release,
	.read = dt3155_read,
	.poll = dt3155_poll,
	.unlocked_ioctl = video_ioctl2, /* V4L2 ioctl handler */
	.mmap = dt3155_mmap,
};

static int
dt3155_ioc_streamon(struct file *filp, void *p, enum v4l2_buf_type type)
{
	struct dt3155_priv *pd = video_drvdata(filp);
<<<<<<< HEAD
	int ret = -ERESTARTSYS;

	if (mutex_lock_interruptible(&pd->mux) == -EINTR)
		return ret;
	if (!pd->acq_fp) {
		ret = videobuf_streamon(pd->vidq);
		if (ret)
			goto unlock;
		pd->acq_fp = filp;
		pd->streaming = 1;
		wake_up_interruptible_sync(&pd->do_dma);
	} else if (pd->acq_fp == filp) {
		pd->streaming = 1;
		ret = videobuf_streamon(pd->vidq);
		if (!ret)
			wake_up_interruptible_sync(&pd->do_dma);
	} else
		ret = -EBUSY;
unlock:
	mutex_unlock(&pd->mux);
	return ret;
=======

	return vb2_streamon(pd->q, type);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
}

static int
dt3155_ioc_streamoff(struct file *filp, void *p, enum v4l2_buf_type type)
{
	struct dt3155_priv *pd = video_drvdata(filp);
<<<<<<< HEAD
	struct videobuf_buffer *tmp;
	unsigned long flags;
	int ret;

	ret = videobuf_streamoff(pd->vidq);
	if (ret)
		return ret;
	spin_lock_irqsave(&pd->lock, flags);
	tmp = pd->curr_buf;
	spin_unlock_irqrestore(&pd->lock, flags);
	if (tmp)
		videobuf_waiton(pd->vidq, tmp, 0, 1); /* block, interruptible */
	return ret;
=======

	return vb2_streamoff(pd->q, type);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
}

static int
dt3155_ioc_querycap(struct file *filp, void *p, struct v4l2_capability *cap)
{
	struct dt3155_priv *pd = video_drvdata(filp);

	strcpy(cap->driver, DT3155_NAME);
	strcpy(cap->card, DT3155_NAME " frame grabber");
	sprintf(cap->bus_info, "PCI:%s", pci_name(pd->pdev));
	cap->version =
	       KERNEL_VERSION(DT3155_VER_MAJ, DT3155_VER_MIN, DT3155_VER_EXT);
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE |
<<<<<<< HEAD
				V4L2_CAP_STREAMING |
				V4L2_CAP_READWRITE;
=======
				DT3155_CAPTURE_METHOD;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	return 0;
}

static int
dt3155_ioc_enum_fmt_vid_cap(struct file *filp, void *p, struct v4l2_fmtdesc *f)
{
	if (f->index >= NUM_OF_FORMATS)
		return -EINVAL;
	*f = frame_std[f->index];
	return 0;
}

static int
dt3155_ioc_g_fmt_vid_cap(struct file *filp, void *p, struct v4l2_format *f)
{
	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	f->fmt.pix.width = img_width;
	f->fmt.pix.height = img_height;
	f->fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
	f->fmt.pix.field = V4L2_FIELD_NONE;
	f->fmt.pix.bytesperline = f->fmt.pix.width;
	f->fmt.pix.sizeimage = f->fmt.pix.width * f->fmt.pix.height;
	f->fmt.pix.colorspace = 0;
	f->fmt.pix.priv = 0;
	return 0;
}

static int
dt3155_ioc_try_fmt_vid_cap(struct file *filp, void *p, struct v4l2_format *f)
{
	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (f->fmt.pix.width == img_width &&
		f->fmt.pix.height == img_height &&
		f->fmt.pix.pixelformat == V4L2_PIX_FMT_GREY &&
		f->fmt.pix.field == V4L2_FIELD_NONE &&
		f->fmt.pix.bytesperline == f->fmt.pix.width &&
		f->fmt.pix.sizeimage == f->fmt.pix.width * f->fmt.pix.height)
			return 0;
	else
		return -EINVAL;
}

static int
dt3155_ioc_s_fmt_vid_cap(struct file *filp, void *p, struct v4l2_format *f)
{
<<<<<<< HEAD
	struct dt3155_priv *pd = video_drvdata(filp);
	int ret =  -ERESTARTSYS;

	if (mutex_lock_interruptible(&pd->mux) == -EINTR)
		return ret;
	if (!pd->acq_fp) {
		pd->acq_fp = filp;
		pd->streaming = 0;
	} else if (pd->acq_fp != filp) {
		ret = -EBUSY;
		goto done;
	}
/*	FIXME: we don't change the format for now
	if (pd->vidq->streaming || pd->vidq->reading || pd->curr_buff) {
		ret = -EBUSY;
		goto done;
	}
*/
	ret = dt3155_ioc_g_fmt_vid_cap(filp, p, f);
done:
	mutex_unlock(&pd->mux);
	return ret;
=======
	return dt3155_ioc_g_fmt_vid_cap(filp, p, f);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
}

static int
dt3155_ioc_reqbufs(struct file *filp, void *p, struct v4l2_requestbuffers *b)
{
	struct dt3155_priv *pd = video_drvdata(filp);
<<<<<<< HEAD
	struct videobuf_queue *q = pd->vidq;
	int ret = -ERESTARTSYS;

	if (b->memory != V4L2_MEMORY_MMAP)
		return -EINVAL;
	if (mutex_lock_interruptible(&pd->mux) == -EINTR)
		return ret;
	if (!pd->acq_fp)
		pd->acq_fp = filp;
	else if (pd->acq_fp != filp) {
		ret = -EBUSY;
		goto done;
	}
	pd->streaming = 1;
	ret = 0;
done:
	mutex_unlock(&pd->mux);
	if (ret)
		return ret;
	if (b->count)
		ret = videobuf_reqbufs(q, b);
	else { /* FIXME: is it necessary? */
		printk(KERN_DEBUG "dt3155: request to free buffers\n");
		/* ret = videobuf_mmap_free(q); */
		ret = dt3155_ioc_streamoff(filp, p,
						V4L2_BUF_TYPE_VIDEO_CAPTURE);
	}
	return ret;
=======

	return vb2_reqbufs(pd->q, b);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
}

static int
dt3155_ioc_querybuf(struct file *filp, void *p, struct v4l2_buffer *b)
{
	struct dt3155_priv *pd = video_drvdata(filp);
<<<<<<< HEAD
	struct videobuf_queue *q = pd->vidq;

	return videobuf_querybuf(q, b);
=======

	return vb2_querybuf(pd->q, b);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
}

static int
dt3155_ioc_qbuf(struct file *filp, void *p, struct v4l2_buffer *b)
{
	struct dt3155_priv *pd = video_drvdata(filp);
<<<<<<< HEAD
	struct videobuf_queue *q = pd->vidq;
	int ret;

	ret = videobuf_qbuf(q, b);
	if (ret)
		return ret;
	return videobuf_querybuf(q, b);
=======

	return vb2_qbuf(pd->q, b);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
}

static int
dt3155_ioc_dqbuf(struct file *filp, void *p, struct v4l2_buffer *b)
{
	struct dt3155_priv *pd = video_drvdata(filp);
<<<<<<< HEAD
	struct videobuf_queue *q = pd->vidq;

	return videobuf_dqbuf(q, b, filp->f_flags & O_NONBLOCK);
=======

	return vb2_dqbuf(pd->q, b, filp->f_flags & O_NONBLOCK);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
}

static int
dt3155_ioc_querystd(struct file *filp, void *p, v4l2_std_id *norm)
{
	*norm = DT3155_CURRENT_NORM;
	return 0;
}

static int
dt3155_ioc_g_std(struct file *filp, void *p, v4l2_std_id *norm)
{
	*norm = DT3155_CURRENT_NORM;
	return 0;
}

static int
dt3155_ioc_s_std(struct file *filp, void *p, v4l2_std_id *norm)
{
	if (*norm & DT3155_CURRENT_NORM)
		return 0;
	return -EINVAL;
}

static int
dt3155_ioc_enum_input(struct file *filp, void *p, struct v4l2_input *input)
{
	if (input->index)
		return -EINVAL;
	strcpy(input->name, "Coax in");
	input->type = V4L2_INPUT_TYPE_CAMERA;
	/*
	 * FIXME: input->std = 0 according to v4l2 API
	 * VIDIOC_G_STD, VIDIOC_S_STD, VIDIOC_QUERYSTD and VIDIOC_ENUMSTD
	 * should return -EINVAL
	 */
	input->std = DT3155_CURRENT_NORM;
	input->status = 0;/* FIXME: add sync detection & V4L2_IN_ST_NO_H_LOCK */
	return 0;
}

static int
dt3155_ioc_g_input(struct file *filp, void *p, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int
dt3155_ioc_s_input(struct file *filp, void *p, unsigned int i)
{
	if (i)
		return -EINVAL;
	return 0;
}

static int
dt3155_ioc_g_parm(struct file *filp, void *p, struct v4l2_streamparm *parms)
{
	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	parms->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	parms->parm.capture.capturemode = 0;
	parms->parm.capture.timeperframe.numerator = 1001;
	parms->parm.capture.timeperframe.denominator = frames_per_sec * 1000;
	parms->parm.capture.extendedmode = 0;
	parms->parm.capture.readbuffers = 1; /* FIXME: 2 buffers? */
	return 0;
}

static int
dt3155_ioc_s_parm(struct file *filp, void *p, struct v4l2_streamparm *parms)
{
	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	parms->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	parms->parm.capture.capturemode = 0;
	parms->parm.capture.timeperframe.numerator = 1001;
	parms->parm.capture.timeperframe.denominator = frames_per_sec * 1000;
	parms->parm.capture.extendedmode = 0;
	parms->parm.capture.readbuffers = 1; /* FIXME: 2 buffers? */
	return 0;
}

static const struct v4l2_ioctl_ops dt3155_ioctl_ops = {
	.vidioc_streamon = dt3155_ioc_streamon,
	.vidioc_streamoff = dt3155_ioc_streamoff,
	.vidioc_querycap = dt3155_ioc_querycap,
/*
	.vidioc_g_priority = dt3155_ioc_g_priority,
	.vidioc_s_priority = dt3155_ioc_s_priority,
*/
	.vidioc_enum_fmt_vid_cap = dt3155_ioc_enum_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap = dt3155_ioc_try_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap = dt3155_ioc_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = dt3155_ioc_s_fmt_vid_cap,
	.vidioc_reqbufs = dt3155_ioc_reqbufs,
	.vidioc_querybuf = dt3155_ioc_querybuf,
	.vidioc_qbuf = dt3155_ioc_qbuf,
	.vidioc_dqbuf = dt3155_ioc_dqbuf,
	.vidioc_querystd = dt3155_ioc_querystd,
	.vidioc_g_std = dt3155_ioc_g_std,
	.vidioc_s_std = dt3155_ioc_s_std,
	.vidioc_enum_input = dt3155_ioc_enum_input,
	.vidioc_g_input = dt3155_ioc_g_input,
	.vidioc_s_input = dt3155_ioc_s_input,
/*
	.vidioc_queryctrl = dt3155_ioc_queryctrl,
	.vidioc_g_ctrl = dt3155_ioc_g_ctrl,
	.vidioc_s_ctrl = dt3155_ioc_s_ctrl,
	.vidioc_querymenu = dt3155_ioc_querymenu,
	.vidioc_g_ext_ctrls = dt3155_ioc_g_ext_ctrls,
	.vidioc_s_ext_ctrls = dt3155_ioc_s_ext_ctrls,
*/
	.vidioc_g_parm = dt3155_ioc_g_parm,
	.vidioc_s_parm = dt3155_ioc_s_parm,
/*
	.vidioc_cropcap = dt3155_ioc_cropcap,
	.vidioc_g_crop = dt3155_ioc_g_crop,
	.vidioc_s_crop = dt3155_ioc_s_crop,
	.vidioc_enum_framesizes = dt3155_ioc_enum_framesizes,
	.vidioc_enum_frameintervals = dt3155_ioc_enum_frameintervals,
*/
};

static int __devinit
<<<<<<< HEAD
dt3155_init_board(struct pci_dev *dev)
{
	struct dt3155_priv *pd = pci_get_drvdata(dev);
=======
dt3155_init_board(struct pci_dev *pdev)
{
	struct dt3155_priv *pd = pci_get_drvdata(pdev);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	void *buf_cpu;
	dma_addr_t buf_dma;
	int i;
	u8 tmp;

<<<<<<< HEAD
	pci_set_master(dev); /* dt3155 needs it */
=======
	pci_set_master(pdev); /* dt3155 needs it */
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	/*  resetting the adapter  */
	iowrite32(FLD_CRPT_ODD | FLD_CRPT_EVEN | FLD_DN_ODD | FLD_DN_EVEN,
							pd->regs + CSR1);
	mmiowb();
<<<<<<< HEAD
	msleep(10);
=======
	msleep(20);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	/*  initializing adaper registers  */
	iowrite32(FIFO_EN | SRST, pd->regs + CSR1);
	mmiowb();
	iowrite32(0xEEEEEE01, pd->regs + EVEN_PIXEL_FMT);
	iowrite32(0xEEEEEE01, pd->regs + ODD_PIXEL_FMT);
	iowrite32(0x00000020, pd->regs + FIFO_TRIGER);
	iowrite32(0x00000103, pd->regs + XFER_MODE);
	iowrite32(0, pd->regs + RETRY_WAIT_CNT);
	iowrite32(0, pd->regs + INT_CSR);
	iowrite32(1, pd->regs + EVEN_FLD_MASK);
	iowrite32(1, pd->regs + ODD_FLD_MASK);
	iowrite32(0, pd->regs + MASK_LENGTH);
	iowrite32(0x0005007C, pd->regs + FIFO_FLAG_CNT);
	iowrite32(0x01010101, pd->regs + IIC_CLK_DUR);
	mmiowb();

	/* verifying that we have a DT3155 board (not just a SAA7116 chip) */
	read_i2c_reg(pd->regs, DT_ID, &tmp);
	if (tmp != DT3155_ID)
		return -ENODEV;

	/* initialize AD LUT */
	write_i2c_reg(pd->regs, AD_ADDR, 0);
	for (i = 0; i < 256; i++)
		write_i2c_reg(pd->regs, AD_LUT, i);

	/* initialize ADC references */
	/* FIXME: pos_ref & neg_ref depend on VT_50HZ */
	write_i2c_reg(pd->regs, AD_ADDR, AD_CMD_REG);
	write_i2c_reg(pd->regs, AD_CMD, VIDEO_CNL_1 | SYNC_CNL_1 | SYNC_LVL_3);
	write_i2c_reg(pd->regs, AD_ADDR, AD_POS_REF);
	write_i2c_reg(pd->regs, AD_CMD, 34);
	write_i2c_reg(pd->regs, AD_ADDR, AD_NEG_REF);
	write_i2c_reg(pd->regs, AD_CMD, 0);

	/* initialize PM LUT */
	write_i2c_reg(pd->regs, CONFIG, pd->config | PM_LUT_PGM);
	for (i = 0; i < 256; i++) {
		write_i2c_reg(pd->regs, PM_LUT_ADDR, i);
		write_i2c_reg(pd->regs, PM_LUT_DATA, i);
	}
	write_i2c_reg(pd->regs, CONFIG, pd->config | PM_LUT_PGM | PM_LUT_SEL);
	for (i = 0; i < 256; i++) {
		write_i2c_reg(pd->regs, PM_LUT_ADDR, i);
		write_i2c_reg(pd->regs, PM_LUT_DATA, i);
	}
	write_i2c_reg(pd->regs, CONFIG, pd->config); /*  ACQ_MODE_EVEN  */

	/* select chanel 1 for input and set sync level */
	write_i2c_reg(pd->regs, AD_ADDR, AD_CMD_REG);
	write_i2c_reg(pd->regs, AD_CMD, VIDEO_CNL_1 | SYNC_CNL_1 | SYNC_LVL_3);

	/* allocate memory, and initialize the DMA machine */
<<<<<<< HEAD
	buf_cpu = dma_alloc_coherent(&dev->dev, DT3155_BUF_SIZE, &buf_dma,
=======
	buf_cpu = dma_alloc_coherent(&pdev->dev, DT3155_BUF_SIZE, &buf_dma,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
								GFP_KERNEL);
	if (!buf_cpu) {
		printk(KERN_ERR "dt3155: dma_alloc_coherent "
					"(in dt3155_init_board) failed\n");
		return -ENOMEM;
	}
	iowrite32(buf_dma, pd->regs + EVEN_DMA_START);
	iowrite32(buf_dma, pd->regs + ODD_DMA_START);
	iowrite32(0, pd->regs + EVEN_DMA_STRIDE);
	iowrite32(0, pd->regs + ODD_DMA_STRIDE);

	/*  Perform a pseudo even field acquire    */
	iowrite32(FIFO_EN | SRST | CAP_CONT_ODD, pd->regs + CSR1);
	write_i2c_reg(pd->regs, CSR2, pd->csr2 | SYNC_SNTL);
	write_i2c_reg(pd->regs, CONFIG, pd->config);
	write_i2c_reg(pd->regs, EVEN_CSR, CSR_SNGL);
	write_i2c_reg(pd->regs, CSR2, pd->csr2 | BUSY_EVEN | SYNC_SNTL);
	msleep(100);
	read_i2c_reg(pd->regs, CSR2, &tmp);
	write_i2c_reg(pd->regs, EVEN_CSR, CSR_ERROR | CSR_SNGL | CSR_DONE);
	write_i2c_reg(pd->regs, ODD_CSR, CSR_ERROR | CSR_SNGL | CSR_DONE);
	write_i2c_reg(pd->regs, CSR2, pd->csr2);
	iowrite32(FIFO_EN | SRST | FLD_DN_EVEN | FLD_DN_ODD, pd->regs + CSR1);

	/*  deallocate memory  */
<<<<<<< HEAD
	dma_free_coherent(&dev->dev, DT3155_BUF_SIZE, buf_cpu, buf_dma);
=======
	dma_free_coherent(&pdev->dev, DT3155_BUF_SIZE, buf_cpu, buf_dma);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	if (tmp & BUSY_EVEN) {
		printk(KERN_ERR "dt3155: BUSY_EVEN not cleared\n");
		return -EIO;
	}
	return 0;
}

static struct video_device dt3155_vdev = {
	.name = DT3155_NAME,
	.fops = &dt3155_fops,
	.ioctl_ops = &dt3155_ioctl_ops,
	.minor = -1,
	.release = video_device_release,
	.tvnorms = DT3155_CURRENT_NORM,
	.current_norm = DT3155_CURRENT_NORM,
};

/* same as in drivers/base/dma-coherent.c */
struct dma_coherent_mem {
	void		*virt_base;
<<<<<<< HEAD
	u32		device_base;
=======
	dma_addr_t	device_base;
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	int		size;
	int		flags;
	unsigned long	*bitmap;
};

static int __devinit
dt3155_alloc_coherent(struct device *dev, size_t size, int flags)
{
	struct dma_coherent_mem *mem;
	dma_addr_t dev_base;
	int pages = size >> PAGE_SHIFT;
	int bitmap_size = BITS_TO_LONGS(pages) * sizeof(long);

	if ((flags & DMA_MEMORY_MAP) == 0)
		goto out;
	if (!size)
		goto out;
	if (dev->dma_mem)
		goto out;

	mem = kzalloc(sizeof(*mem), GFP_KERNEL);
	if (!mem)
		goto out;
	mem->virt_base = dma_alloc_coherent(dev, size, &dev_base,
							DT3155_COH_FLAGS);
	if (!mem->virt_base)
		goto err_alloc_coherent;
	mem->bitmap = kzalloc(bitmap_size, GFP_KERNEL);
	if (!mem->bitmap)
		goto err_bitmap;

	/* coherent_dma_mask is already set to 32 bits */
	mem->device_base = dev_base;
	mem->size = pages;
	mem->flags = flags;
	dev->dma_mem = mem;
	return DMA_MEMORY_MAP;

err_bitmap:
	dma_free_coherent(dev, size, mem->virt_base, dev_base);
err_alloc_coherent:
	kfree(mem);
out:
	return 0;
}

static void __devexit
dt3155_free_coherent(struct device *dev)
{
	struct dma_coherent_mem *mem = dev->dma_mem;

	if (!mem)
		return;
	dev->dma_mem = NULL;
	dma_free_coherent(dev, mem->size << PAGE_SHIFT,
					mem->virt_base, mem->device_base);
	kfree(mem->bitmap);
	kfree(mem);
}

static int __devinit
<<<<<<< HEAD
dt3155_probe(struct pci_dev *dev, const struct pci_device_id *id)
=======
dt3155_probe(struct pci_dev *pdev, const struct pci_device_id *id)
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
{
	int err;
	struct dt3155_priv *pd;

	printk(KERN_INFO "dt3155: probe()\n");
<<<<<<< HEAD
	err = dma_set_mask(&dev->dev, DMA_BIT_MASK(32));
=======
	err = dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	if (err) {
		printk(KERN_ERR "dt3155: cannot set dma_mask\n");
		return -ENODEV;
	}
<<<<<<< HEAD
	err = dma_set_coherent_mask(&dev->dev, DMA_BIT_MASK(32));
=======
	err = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	if (err) {
		printk(KERN_ERR "dt3155: cannot set dma_coherent_mask\n");
		return -ENODEV;
	}
	pd = kzalloc(sizeof(*pd), GFP_KERNEL);
	if (!pd) {
		printk(KERN_ERR "dt3155: cannot allocate dt3155_priv\n");
		return -ENOMEM;
	}
	pd->vdev = video_device_alloc();
	if (!pd->vdev) {
		printk(KERN_ERR "dt3155: cannot allocate vdev structure\n");
		goto err_video_device_alloc;
	}
	*pd->vdev = dt3155_vdev;
<<<<<<< HEAD
	pci_set_drvdata(dev, pd);    /* for use in dt3155_remove() */
	video_set_drvdata(pd->vdev, pd);  /* for use in video_fops */
	pd->users = 0;
	pd->acq_fp = NULL;
	pd->pdev = dev;
	INIT_LIST_HEAD(&pd->dmaq);
	init_waitqueue_head(&pd->do_dma);
	mutex_init(&pd->mux);
	pd->csr2 = csr2_init;
	pd->config = config_init;
	err = pci_enable_device(pd->pdev);
=======
	pci_set_drvdata(pdev, pd);    /* for use in dt3155_remove() */
	video_set_drvdata(pd->vdev, pd);  /* for use in video_fops */
	pd->users = 0;
	pd->pdev = pdev;
	INIT_LIST_HEAD(&pd->dmaq);
	mutex_init(&pd->mux);
	pd->vdev->lock = &pd->mux; /* for locking v4l2_file_operations */
	spin_lock_init(&pd->lock);
	pd->csr2 = csr2_init;
	pd->config = config_init;
	err = pci_enable_device(pdev);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	if (err) {
		printk(KERN_ERR "dt3155: pci_dev not enabled\n");
		goto err_enable_dev;
	}
<<<<<<< HEAD
	err = pci_request_region(pd->pdev, 0, pci_name(pd->pdev));
	if (err)
		goto err_req_region;
	pd->regs = pci_iomap(pd->pdev, 0, pci_resource_len(pd->pdev, 0));
=======
	err = pci_request_region(pdev, 0, pci_name(pdev));
	if (err)
		goto err_req_region;
	pd->regs = pci_iomap(pdev, 0, pci_resource_len(pd->pdev, 0));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	if (!pd->regs) {
		err = -ENOMEM;
		printk(KERN_ERR "dt3155: pci_iomap failed\n");
		goto err_pci_iomap;
	}
<<<<<<< HEAD
	err = dt3155_init_board(pd->pdev);
=======
	err = dt3155_init_board(pdev);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	if (err) {
		printk(KERN_ERR "dt3155: dt3155_init_board failed\n");
		goto err_init_board;
	}
	err = video_register_device(pd->vdev, VFL_TYPE_GRABBER, -1);
	if (err) {
		printk(KERN_ERR "dt3155: Cannot register video device\n");
		goto err_init_board;
	}
<<<<<<< HEAD
	err = dt3155_alloc_coherent(&dev->dev, DT3155_CHUNK_SIZE,
=======
	err = dt3155_alloc_coherent(&pdev->dev, DT3155_CHUNK_SIZE,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
							DMA_MEMORY_MAP);
	if (err)
		printk(KERN_INFO "dt3155: preallocated 8 buffers\n");
	printk(KERN_INFO "dt3155: /dev/video%i is ready\n", pd->vdev->minor);
	return 0;  /*   success   */

err_init_board:
<<<<<<< HEAD
	pci_iounmap(pd->pdev, pd->regs);
err_pci_iomap:
	pci_release_region(pd->pdev, 0);
err_req_region:
	pci_disable_device(pd->pdev);
=======
	pci_iounmap(pdev, pd->regs);
err_pci_iomap:
	pci_release_region(pdev, 0);
err_req_region:
	pci_disable_device(pdev);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
err_enable_dev:
	video_device_release(pd->vdev);
err_video_device_alloc:
	kfree(pd);
	return err;
}

static void __devexit
<<<<<<< HEAD
dt3155_remove(struct pci_dev *dev)
{
	struct dt3155_priv *pd = pci_get_drvdata(dev);

	printk(KERN_INFO "dt3155: remove()\n");
	dt3155_free_coherent(&dev->dev);
	video_unregister_device(pd->vdev);
	pci_iounmap(dev, pd->regs);
	pci_release_region(pd->pdev, 0);
	pci_disable_device(pd->pdev);
=======
dt3155_remove(struct pci_dev *pdev)
{
	struct dt3155_priv *pd = pci_get_drvdata(pdev);

	printk(KERN_INFO "dt3155: remove()\n");
	dt3155_free_coherent(&pdev->dev);
	video_unregister_device(pd->vdev);
	pci_iounmap(pdev, pd->regs);
	pci_release_region(pdev, 0);
	pci_disable_device(pdev);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	/*
	 * video_device_release() is invoked automatically
	 * see: struct video_device dt3155_vdev
	 */
	kfree(pd);
}

static DEFINE_PCI_DEVICE_TABLE(pci_ids) = {
	{ PCI_DEVICE(DT3155_VENDOR_ID, DT3155_DEVICE_ID) },
	{ 0, /* zero marks the end */ },
};
MODULE_DEVICE_TABLE(pci, pci_ids);

static struct pci_driver pci_driver = {
	.name = DT3155_NAME,
	.id_table = pci_ids,
	.probe = dt3155_probe,
	.remove = __devexit_p(dt3155_remove),
};

static int __init
dt3155_init_module(void)
{
	int err;

	printk(KERN_INFO "dt3155: ==================\n");
	printk(KERN_INFO "dt3155: init()\n");
	err = pci_register_driver(&pci_driver);
	if (err) {
		printk(KERN_ERR "dt3155: cannot register pci_driver\n");
		return err;
	}
	return 0; /* succes */
}

static void __exit
dt3155_exit_module(void)
{
	pci_unregister_driver(&pci_driver);
	printk(KERN_INFO "dt3155: exit()\n");
	printk(KERN_INFO "dt3155: ==================\n");
}

module_init(dt3155_init_module);
module_exit(dt3155_exit_module);

MODULE_DESCRIPTION("video4linux pci-driver for dt3155 frame grabber");
MODULE_AUTHOR("Marin Mitov <mitov@issp.bas.bg>");
MODULE_VERSION(DT3155_VERSION);
MODULE_LICENSE("GPL");
