// SPDX-License-Identifier: GPL-2.0
/*
 * Nuvoton W55FA93 on-chip USB 2.0 device controller (UDC) driver
 *
 * Ported from the Nuvoton W55FA93 Linux 2.6.35 BSP driver
 * (drivers/usb/gadget/w55fa93_udc.c, Copyright (C) 2010 Nuvoton
 * Technology Corp).  The register semantics, the EP0 state machine,
 * the SRAM buffer allocation scheme and the DMA chunking logic are
 * kept faithful to the BSP implementation; only the gadget framework
 * glue has been modernised (UDC core registration, devm resources,
 * DT probing, endpoint capabilities).
 *
 * The controller has one control endpoint and six configurable
 * endpoints (EPA..EPF) sharing a 2 KiB on-chip SRAM, plus a single
 * AHB DMA channel that is shared by all non-control endpoints.
 */

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/usb/gadget.h>

#define DRIVER_DESC	"Nuvoton W55FA93 USB Device Controller Gadget"

#define W55FA93_ENDPOINTS	7	/* EP0 + EPA..EPF */
#define EP0_FIFO_SIZE		64
#define EP_FIFO_SIZE		512

#define DMA_ADDR_INVALID	(~(dma_addr_t)0)

/* ------------------------------------------------------------------ */
/* Register offsets (from mach/w55fa93_reg.h, USBD section)	    */
/* ------------------------------------------------------------------ */

#define USBD_IRQ_STAT_L		0x00	/* interrupt status low */
#define USBD_IRQ_ENB_L		0x08	/* interrupt enable low */
#define USBD_IRQ_STAT		0x10	/* usb interrupt status */
#define USBD_IRQ_ENB		0x14	/* usb interrupt enable */
#define USBD_OPER		0x18	/* usb operation */
#define USBD_FRAME_CNT		0x1c	/* frame count */
#define USBD_ADDR		0x20	/* device address */
#define USBD_MEM_TEST		0x24	/* test mode */
#define USBD_CEP_DATA_BUF	0x28	/* control-ep data buffer */
#define USBD_CEP_CTRL_STAT	0x2c	/* control-ep control/status */
#define USBD_CEP_IRQ_ENB	0x30	/* control-ep interrupt enable */
#define USBD_CEP_IRQ_STAT	0x34	/* control-ep interrupt status */
#define USBD_IN_TRNSFR_CNT	0x38	/* in-transfer data count */
#define USBD_OUT_TRNSFR_CNT	0x3c	/* out-transfer data count */
#define USBD_CEP_CNT		0x40	/* control-ep data count */
#define USBD_SETUP1_0		0x44	/* setup byte1 & byte0 */
#define USBD_SETUP3_2		0x48	/* setup byte3 & byte2 */
#define USBD_SETUP5_4		0x4c	/* setup byte5 & byte4 */
#define USBD_SETUP7_6		0x50	/* setup byte7 & byte6 */
#define USBD_CEP_START_ADDR	0x54	/* control-ep ram start address */
#define USBD_CEP_END_ADDR	0x58	/* control-ep ram end address */
#define USBD_DMA_CTRL_STS	0x5c	/* dma control and status */
#define USBD_DMA_CNT		0x60	/* dma count */

/* Non-control endpoint register block: EPA at 0x64, stride 0x28 */
#define USBD_EP_BASE(n)		(0x64 + 0x28 * ((n) - 1))	/* n = 1..6 */
#define EP_DATA_BUF		0x00
#define EP_IRQ_STAT		0x04
#define EP_IRQ_ENB		0x08
#define EP_DATA_CNT		0x0c
#define EP_RSP_SC		0x10
#define EP_MPS			0x14
#define EP_TRF_CNT		0x18
#define EP_CFG			0x1c
#define EP_START_ADDR		0x20
#define EP_END_ADDR		0x24

#define USBD_HEAD_WORD0		0x158	/* first head data (ISO) */
#define USBD_EP_HEAD_CNT(n)	(0x164 + 4 * ((n) - 1))	/* n = 1..6 */

#define USBD_AHB_DMA_ADDR	0x700	/* AHB DMA address */
#define USBD_PHY_CTL		0x704	/* USB PHY control */
#define PHY_VBUS_STATUS		BIT(31)	/* VBUS present (ro) */

/* IRQ_STAT_L bits */
#define IRQ_USB_STAT		0x01
#define IRQ_CEP			0x02
#define IRQ_NCEP		0xfc	/* one bit per EPA..EPF, at bits 2..7 */

/* USBD_IRQ_STAT / USBD_IRQ_ENB bits */
#define USB_SOF			0x001
#define USB_RST_STS		0x002
#define USB_RESUME		0x004
#define USB_SUS_REQ		0x008
#define USB_HS_SETTLE		0x010
#define USB_DMA_REQ		0x020
#define USABLE_CLK		0x040
#define USB_VBUS_STS		0x100

/* USBD_OPER bits */
#define USB_GEN_RES		0x1
#define USB_HS			0x2
#define USB_CUR_SPD_HS		0x4

/* CEP_IRQ_STAT / CEP_IRQ_ENB bits */
#define CEP_SUPTOK		0x0001
#define CEP_SUPPKT		0x0002
#define CEP_OUT_TOK		0x0004
#define CEP_IN_TOK		0x0008
#define CEP_PING_TOK		0x0010
#define CEP_DATA_TXD		0x0020
#define CEP_DATA_RXD		0x0040
#define CEP_NAK_SENT		0x0080
#define CEP_STALL_SENT		0x0100
#define CEP_USB_ERR		0x0200
#define CEP_STS_END		0x0400
#define CEP_BUFF_FULL		0x0800
#define CEP_BUFF_EMPTY		0x1000

/* CEP_CTRL_STAT bits (write) */
#define CEP_NAK_CLEAR		0x00	/* writing zero clears the NAK bit */
#define CEP_SEND_STALL		0x02
#define CEP_ZEROLEN		0x04
#define CEP_FLUSH		0x08

/* EP_IRQ_STAT / EP_IRQ_ENB bits */
#define EP_BUFF_FULL		0x0001
#define EP_BUFF_EMPTY		0x0002
#define EP_SHORT_PKT		0x0004
#define EP_DATA_TXD		0x0008
#define EP_DATA_RXD		0x0010
#define EP_OUT_TOK		0x0020
#define EP_IN_TOK		0x0040
#define EP_PING_TOK		0x0080
#define EP_NAK_SENT		0x0100
#define EP_STALL_SENT		0x0200
#define EP_USB_ERR		0x0800
#define EP_BO_SHORT_PKT		0x1000

/* EP_RSP_SC bits */
#define EP_BUFF_FLUSH		0x01
#define EP_MODE_MASK		0x06
#define EP_TOGGLE		0x08
#define EP_HALT			0x10
#define EP_ZERO_IN		0x20
#define EP_PKT_END		0x40

/* EP_CFG field values (note: hardware encoding, not ch9 encoding) */
#define EP_TYPE_BLK		0x01
#define EP_TYPE_INT		0x02
#define EP_TYPE_ISO		0x03

#define EP_MODE_AUTO		0x00
#define EP_MODE_MAN		0x02
#define EP_MODE_FLY		0x04

/* DMA direction for the shared AHB DMA channel */
#define Ep_Out			0x00
#define Ep_In			0x01

#define DMA_READ		1
#define DMA_WRITE		2

#define USBD_DMA_LEN		0x10000
#define USBD_ISO_DMA_LEN	0xF0000

/* ------------------------------------------------------------------ */
/* Driver-private structures (from the BSP w55fa93_udc.h)		    */
/* ------------------------------------------------------------------ */

struct w55fa93_udc;

struct w55fa93_ep {
	struct list_head	queue;
	struct w55fa93_udc	*dev;
	const struct usb_endpoint_descriptor *desc;
	struct usb_ep		ep;
	u8			index;		/* 0 = EP0, 1..6 = EPA..EPF */

	u8			buffer_disabled;
	u8			bEndpointAddress; /* with direction */

	u8			EP_Mode;	/* auto/manual/fly */
	u8			EP_Num;		/* endpoint number, no direction */
	u8			EP_Dir;		/* 0 OUT, 1 IN */
	u8			EP_Type;	/* EP_TYPE_BLK/INT/ISO encoding */
	u32			irq_enb;
};

struct w55fa93_request {
	struct list_head	queue;		/* ep's requests */
	struct usb_request	req;
	u32			dma_mapped;
};

enum ep0_state {
	EP0_IDLE,
	EP0_IN_DATA_PHASE,
	EP0_OUT_DATA_PHASE,
	EP0_END_XFER,
	EP0_STALL,
};

struct w55fa93_udc {
	spinlock_t		lock;

	struct w55fa93_ep	ep[W55FA93_ENDPOINTS];
	struct usb_gadget	gadget;
	struct usb_gadget_driver *driver;
	struct device		*dev;
	struct clk		*clk;
	void __iomem		*base;
	int			irq;

	enum ep0_state		ep0state;

	u8			usb_devstate;	/* 1 default, 2 addressed, 3 configured */
	u8			usb_address;

	u8			usb_dma_dir;
	u8			usb_dma_trigger;	/* bool: DMA in flight */
	u8			usb_dma_trigger_next;	/* bool: more chunks pending */
	u8			usb_less_mps;
	u32			usb_dma_cnt;	/* one DMA transfer count */
	u32			usb_dma_loop;	/* for short packet only; DMA loop, each loop 32 bytes */
	u32			usb_dma_owner;	/* index of EP owning the DMA channel */

	struct usb_ctrlrequest	crq;
	s32			setup_ret;

	u32			irq_enbl;	/* snapshot of USBD_IRQ_ENB_L */

	/* GET_STATUS / SET_FEATURE(endpoint halt) bookkeeping.
	 * These were file-scope globals in the BSP driver. */
	int			halt_ep;	/* endpoint number, -1 = none */
	int			status_word;	/* pending GET_STATUS reply, -1 = none */
	u8			ep_halted[W55FA93_ENDPOINTS - 1];
	u32			iso_frame_count;
	bool			vbus;
};

static const char *const ep_name[] = {
	"ep0",					/* everyone has ep0 */
	"ep1", "ep2", "ep3", "ep4", "ep5", "ep6"
};

/* ------------------------------------------------------------------ */
/* Register access helpers					    */
/* ------------------------------------------------------------------ */

static inline u32 udc_readl(struct w55fa93_udc *udc, u32 offset)
{
	return readl(udc->base + offset);
}

static inline void udc_writel(struct w55fa93_udc *udc, u32 value, u32 offset)
{
	writel(value, udc->base + offset);
}

static inline u32 ep_readl(struct w55fa93_ep *ep, u32 offset)
{
	return udc_readl(ep->dev, USBD_EP_BASE(ep->index) + offset);
}

static inline void ep_writel(struct w55fa93_ep *ep, u32 value, u32 offset)
{
	udc_writel(ep->dev, value, USBD_EP_BASE(ep->index) + offset);
}

/* ------------------------------------------------------------------ */
/* Request completion						    */
/*								    */
/* w55fa93_done() and w55fa93_nuke() are called with udc->lock held;  */
/* they drop and reacquire it around the completion callback, as the  */
/* BSP driver did, so gadget drivers may queue new requests from      */
/* within ->complete().						    */
/* ------------------------------------------------------------------ */

static void w55fa93_nuke(struct w55fa93_udc *udc, struct w55fa93_ep *ep)
{
	while (!list_empty(&ep->queue)) {
		struct w55fa93_request *req;

		req = list_first_entry(&ep->queue, struct w55fa93_request,
				       queue);
		list_del_init(&req->queue);
		req->req.status = -ESHUTDOWN;
		if (req->dma_mapped) {
			dma_unmap_single(udc->dev, req->req.dma,
					 req->req.length,
					 ep->EP_Dir ? DMA_TO_DEVICE
						    : DMA_FROM_DEVICE);
			req->req.dma = DMA_ADDR_INVALID;
			req->dma_mapped = 0;
		}
		spin_unlock(&udc->lock);
		usb_gadget_giveback_request(&ep->ep, &req->req);
		spin_lock(&udc->lock);
	}
}

static void w55fa93_done(struct w55fa93_ep *ep, struct w55fa93_request *req,
			 int status)
{
	struct w55fa93_udc *udc = ep->dev;

	list_del_init(&req->queue);

	if (ep->index) {
		if (list_empty(&ep->queue))
			ep_writel(ep, 0, EP_IRQ_ENB);
		else
			ep_writel(ep, ep->irq_enb, EP_IRQ_ENB);
	}

	if (likely(req->req.status == -EINPROGRESS))
		req->req.status = status;
	else
		status = req->req.status;

	if (req->dma_mapped) {
		dma_unmap_single(udc->dev, req->req.dma, req->req.length,
				 ep->EP_Dir ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
		req->req.dma = DMA_ADDR_INVALID;
		req->dma_mapped = 0;
	}

	spin_unlock(&udc->lock);
	usb_gadget_giveback_request(&ep->ep, &req->req);
	spin_lock(&udc->lock);
}

static inline void clear_ep_state(struct w55fa93_udc *dev)
{
	unsigned int i;

	/* Hardware SET_{CONFIGURATION,INTERFACE} automagic resets endpoint
	 * fifos, and pending transactions mustn't be continued in any case.
	 */
	for (i = 0; i < W55FA93_ENDPOINTS; i++)
		w55fa93_nuke(dev, &dev->ep[i]);
}

/* ------------------------------------------------------------------ */
/* Shared AHB DMA engine						    */
/* ------------------------------------------------------------------ */

static void start_write(struct w55fa93_ep *ep, dma_addr_t addr, u32 length)
{
	struct w55fa93_udc *dev = ep->dev;
	u32 reg;

	if (dev->usb_dma_trigger) {
		dev_dbg(dev->dev, "*** dma trigger ***\n");
		return;
	}
	dev->usb_dma_trigger = 1;
	dev->usb_dma_cnt = length;
	dev->usb_dma_owner = ep->index;

	udc_writel(dev, (USB_DMA_REQ | USB_RST_STS | USB_SUS_REQ |
			 USB_VBUS_STS), USBD_IRQ_ENB);

	/* Tell DMA the memory physical address.  The controller's AHB
	 * DMA takes a plain 32-bit system address; on this SoC the DMA
	 * address returned by dma_map_single() is that address. */
	udc_writel(dev, lower_32_bits(addr), USBD_AHB_DMA_ADDR);
	udc_writel(dev, length, USBD_DMA_CNT);

	reg = udc_readl(dev, USBD_DMA_CTRL_STS);
	if ((reg & 0x40) != 0x40)
		udc_writel(dev, reg | 0x00000020, USBD_DMA_CTRL_STS);
}

static void start_read(struct w55fa93_ep *ep, dma_addr_t addr, u32 length)
{
	struct w55fa93_udc *dev = ep->dev;

	if (dev->usb_dma_trigger) {
		dev_dbg(dev->dev, "*** dma trigger ***\n");
		return;
	}

	udc_writel(dev, (USB_DMA_REQ | USB_RST_STS | USB_SUS_REQ |
			 USB_VBUS_STS), USBD_IRQ_ENB);
	udc_writel(dev, lower_32_bits(addr), USBD_AHB_DMA_ADDR);
	udc_writel(dev, length, USBD_DMA_CNT);

	dev->usb_dma_trigger = 1;
	dev->usb_dma_cnt = length;
	dev->usb_dma_loop = (length + 31) / 32;
	dev->usb_dma_owner = ep->index;

	udc_writel(dev, udc_readl(dev, USBD_DMA_CTRL_STS) | 0x00000020,
		   USBD_DMA_CTRL_STS);
}

/*
 * Kick off a DMA transfer for a non-control endpoint.
 *
 * As in the BSP driver this only arms the DMA engine (in chunks of
 * at most USBD_DMA_LEN, aligned down to whole maxpackets); progress
 * is accounted in the DMA interrupt handler, so the return value is
 * always 0.
 */
static int udc_transfer(struct w55fa93_ep *ep, dma_addr_t addr, size_t size,
			u32 mode)
{
	struct w55fa93_udc *dev = ep->dev;
	unsigned int count = 0;
	int loop;

	loop = size / USBD_DMA_LEN;

	if (mode == DMA_WRITE) {
		/* wait for DMA complete */
		while (udc_readl(dev, USBD_DMA_CTRL_STS) & 0x20) {
			if (!(udc_readl(dev, USBD_PHY_CTL) & PHY_VBUS_STATUS)) {
				dev_dbg(dev->dev, "Un-Plug (6)!!\n");
				return 0;
			}
		}

		dev->usb_dma_dir = Ep_In;
		dev->usb_less_mps = 0;
		udc_writel(dev, 0x03, USBD_IRQ_ENB_L);
		udc_writel(dev, (udc_readl(dev, USBD_DMA_CTRL_STS) & 0xe0) |
				0x10 | ep->EP_Num, USBD_DMA_CTRL_STS);
		ep_writel(ep, 0, EP_IRQ_ENB);

		if (ep->EP_Type == EP_TYPE_ISO &&
		    udc_readl(dev, USBD_HEAD_WORD0) != 0) {
			udc_writel(dev,
				   (udc_readl(dev, USBD_HEAD_WORD0) & ~0x300) |
				   ((dev->iso_frame_count & 0x01) << 8),
				   USBD_HEAD_WORD0);
			loop = size / USBD_ISO_DMA_LEN;
			if (loop > 0) {
				dev->usb_dma_trigger_next = 1;
				start_write(ep, addr, USBD_ISO_DMA_LEN);
			} else {
				dev->usb_less_mps = 1;
				dev->usb_dma_trigger_next = 0;
				start_write(ep, addr, size);
			}
		} else if (loop > 0) {
			dev->usb_dma_trigger_next = 1;
			start_write(ep, addr, USBD_DMA_LEN);
		} else if (size >= ep->ep.maxpacket) {
			count = size / ep->ep.maxpacket;
			count *= ep->ep.maxpacket;

			if (count < size)
				dev->usb_dma_trigger_next = 1;
			start_write(ep, addr, count);
		} else {
			if (ep->EP_Type == EP_TYPE_BLK ||
			    ep->EP_Type == EP_TYPE_ISO)
				dev->usb_less_mps = 1;
			start_write(ep, addr, size);
		}
	} else if (mode == DMA_READ) {
		dev->usb_dma_dir = Ep_Out;
		dev->usb_less_mps = 0;
		udc_writel(dev, 0x03, USBD_IRQ_ENB_L);

		udc_writel(dev, (udc_readl(dev, USBD_DMA_CTRL_STS) & 0xe0) |
				ep->EP_Num, USBD_DMA_CTRL_STS);
		ep_writel(ep, 0x1000, EP_IRQ_ENB);
		/* NOTE: the BSP wrote (ep->index << 2) here, which sets
		 * extra NCEP enable bits for index >= 3; the per-endpoint
		 * bit (index + 1) used in ep_enable is used instead. */
		udc_writel(dev, udc_readl(dev, USBD_IRQ_ENB_L) |
				(1 << (ep->index + 1)), USBD_IRQ_ENB_L);

		if (loop > 0) {
			loop--;
			if (loop > 0)
				dev->usb_dma_trigger_next = 1;
			start_read(ep, addr, USBD_DMA_LEN);
		} else if (size >= ep->ep.maxpacket) {
			count = size / ep->ep.maxpacket;
			count *= ep->ep.maxpacket;
			if (count < size)
				dev->usb_dma_trigger_next = 1;
			start_read(ep, addr, count);
		} else {
			/* using short packet intr to deal with */
			start_read(ep, addr, size);
		}
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/* FIFO / DMA packet movement					    */
/* ------------------------------------------------------------------ */

static int write_packet(struct w55fa93_ep *ep, struct w55fa93_request *req)
{
	struct w55fa93_udc *udc = ep->dev;
	unsigned int len;
	dma_addr_t addr;
	u8 *buf;
	u16 i;
	u32 max;

	buf = req->req.buf + req->req.actual;

	if (ep->EP_Num == 0) {	/* ctrl pipe doesn't use DMA */
		max = ep->ep.maxpacket;
		len = min(req->req.length - req->req.actual, max);
		if (len == 0) {
			if (req->req.zero && !req->req.length)
				udc_writel(udc, CEP_ZEROLEN, USBD_CEP_CTRL_STAT);
		} else {
			for (i = 0; i < len; i++)
				writeb(*buf++, udc->base + USBD_CEP_DATA_BUF);

			udc_writel(udc, len, USBD_IN_TRNSFR_CNT);
		}

		req->req.actual += len;
	} else {
		len = req->req.length - req->req.actual;

		if (req->req.dma == DMA_ADDR_INVALID) {
			req->req.dma = dma_map_single(udc->dev, req->req.buf,
					req->req.length,
					ep->EP_Dir ? DMA_TO_DEVICE
						   : DMA_FROM_DEVICE);
			req->dma_mapped = 1;
		} else {
			dma_sync_single_for_device(udc->dev, req->req.dma,
					req->req.length,
					ep->EP_Dir ? DMA_TO_DEVICE
						   : DMA_FROM_DEVICE);
			req->dma_mapped = 0;
		}
		addr = req->req.dma + req->req.actual;

		if (len == 0) {
			if (ep->EP_Type != EP_TYPE_ISO) {
				ep_writel(ep,
					  (ep_readl(ep, EP_RSP_SC) & 0xF7) |
					  EP_ZERO_IN, EP_RSP_SC);
			}
		} else {
			len = udc_transfer(ep, addr, len, DMA_WRITE);
		}

		req->req.actual += len;
	}

	return len;
}

/*
 * write_fifo - return: 0 = still running, 1 = completed
 */
static int write_fifo(struct w55fa93_ep *ep, struct w55fa93_request *req)
{
	write_packet(ep, req);

	/* last packet is often short (sometimes a zlp) */
	if (req->req.length == req->req.actual) {
		w55fa93_done(ep, req, 0);
		return 1;
	}

	return 0;
}

static int read_packet(struct w55fa93_ep *ep, u8 *buf,
		       struct w55fa93_request *req, u16 cnt)
{
	struct w55fa93_udc *udc = ep->dev;
	unsigned int len, fifo_count;
	u16 i;
	u32 data, tmp;

	if (ep->EP_Num == 0) {	/* ctrl pipe doesn't use DMA */
		fifo_count = udc_readl(udc, USBD_CEP_CNT);
		len = min(req->req.length - req->req.actual, fifo_count);

		tmp = len / 4;
		for (i = 0; i < tmp; i++) {
			data = readl(udc->base + USBD_CEP_DATA_BUF);
			*buf++ = data & 0xFF;
			*buf++ = (data & 0xFF00) >> 8;
			*buf++ = (data & 0xFF0000) >> 16;
			*buf++ = (data & 0xFF000000) >> 24;
		}

		tmp = len % 4;
		for (i = 0; i < tmp; i++) {
			data = readb(udc->base + USBD_CEP_DATA_BUF);
			*buf++ = data & 0xFF;
		}
		req->req.actual += len;
	} else {
		if (req->req.dma == DMA_ADDR_INVALID) {
			req->req.dma = dma_map_single(udc->dev, req->req.buf,
					req->req.length,
					ep->EP_Dir ? DMA_TO_DEVICE
						   : DMA_FROM_DEVICE);
			req->dma_mapped = 1;
		} else {
			dma_sync_single_for_device(udc->dev, req->req.dma,
					req->req.length,
					ep->EP_Dir ? DMA_TO_DEVICE
						   : DMA_FROM_DEVICE);
			req->dma_mapped = 0;
		}

		len = req->req.length - req->req.actual;

		if (cnt && cnt < ep->ep.maxpacket)
			len = udc_transfer(ep, req->req.dma, cnt, DMA_READ);
		else if (len)
			len = udc_transfer(ep, req->req.dma, len, DMA_READ);
		req->req.actual += len;
	}

	return len;
}

/*
 * read_fifo - return: 0 = still running, 1 = completed, negative = errno
 */
static int read_fifo(struct w55fa93_ep *ep, struct w55fa93_request *req,
		     u16 cnt)
{
	u8 *buf;
	unsigned int bufferspace;
	int is_last = 1;
	int fifo_count = 0;

	buf = req->req.buf + req->req.actual;
	bufferspace = req->req.length - req->req.actual;
	if (!bufferspace) {
		dev_dbg(ep->dev->dev, "read_fifo: Buffer full !!\n");
		return -1;
	}

	fifo_count = read_packet(ep, buf, req, cnt);

	if (req->req.length == req->req.actual) {
		w55fa93_done(ep, req, 0);
	} else if (fifo_count && fifo_count < ep->ep.maxpacket) {
		w55fa93_done(ep, req, 0);
		/* overflowed this request?  flush extra data */
		if (req->req.length != req->req.actual) {
			dev_dbg(ep->dev->dev, "%s(): EOVERFLOW set\n",
				__func__);
			if (req->req.short_not_ok)
				req->req.status = -EOVERFLOW;
		}
	} else {
		is_last = 0;
	}

	return is_last;
}

/* ------------------------------------------------------------------ */
/* On-chip SRAM buffer allocation (2 KiB total, EP0 takes 0x00..0x3f) */
/* ------------------------------------------------------------------ */

static s32 sram_data[7][2] = { {0, 0x40} };

/* 0-3F for Ctrl pipe */
static s32 get_sram_base(struct w55fa93_udc *dev, u32 max)
{
	int i, cnt = 1, j;
	s32 start, end;

	for (i = 1; i < W55FA93_ENDPOINTS; i++) {
		struct w55fa93_ep *ep = &dev->ep[i];

		start = ep_readl(ep, EP_START_ADDR);
		end = ep_readl(ep, EP_END_ADDR);
		if (end - start > 0) {
			sram_data[cnt][0] = start;
			sram_data[cnt][1] = end + 1;
			cnt++;
		}
	}

	if (cnt == 1)
		return 0x40;

	/* sorting from small to big */
	j = 1;
	while (j < cnt) {
		for (i = 0; i < cnt - j; i++) {
			if (sram_data[i][0] > sram_data[i + 1][0]) {
				start = sram_data[i][0];
				end = sram_data[i][1];
				sram_data[i][0] = sram_data[i + 1][0];
				sram_data[i][1] = sram_data[i + 1][1];
				sram_data[i + 1][0] = start;
				sram_data[i + 1][1] = end;
			}
		}
		j++;
	}

	for (i = 0; i < cnt - 1; i++) {
		if (sram_data[i + 1][0] - sram_data[i][1] >= (s32)max)
			return sram_data[i][1];
	}

	if (0x800 - sram_data[cnt - 1][1] >= (s32)max)
		return sram_data[cnt - 1][1];

	return -ENOBUFS;
}

/* ------------------------------------------------------------------ */
/* Endpoint operations						    */
/* ------------------------------------------------------------------ */

static int w55fa93_ep_enable(struct usb_ep *_ep,
			     const struct usb_endpoint_descriptor *desc)
{
	struct w55fa93_udc *dev;
	struct w55fa93_ep *ep;
	u32 max;
	unsigned long flags;
	u32 int_en_reg;
	u32 hshb = 0;
	s32 sram_addr;

	ep = container_of(_ep, struct w55fa93_ep, ep);
	if (!_ep || !desc || ep->desc || ep->index == 0
			|| desc->bDescriptorType != USB_DT_ENDPOINT)
		return -EINVAL;
	dev = ep->dev;

	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;

	max = le16_to_cpu(desc->wMaxPacketSize) & 0x1fff;
	hshb = (max & 0x1800) >> 11;
	max &= 0x7FF;

	spin_lock_irqsave(&dev->lock, flags);
	_ep->maxpacket = max & 0x7ff;

	ep->desc = desc;
	_ep->desc = desc;
	ep->bEndpointAddress = desc->bEndpointAddress;

	/* set max packet */
	ep_writel(ep, max, EP_MPS);
	ep->ep.maxpacket = max;
	if (((ep->desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
			USB_ENDPOINT_XFER_ISOC) && hshb)
		sram_addr = get_sram_base(dev, max * (hshb + 1) + 100);
	else
		sram_addr = get_sram_base(dev, max);
	/* NOTE: the BSP driver returned from here without dropping the
	 * spinlock; fixed in this port. */
	if (sram_addr < 0) {
		spin_unlock_irqrestore(&dev->lock, flags);
		return sram_addr;
	}

	ep_writel(ep, sram_addr, EP_START_ADDR);
	if ((ep->desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
			USB_ENDPOINT_XFER_ISOC)
		sram_addr = sram_addr + max * (hshb + 1) + 100;
	else
		sram_addr = sram_addr + max;
	ep_writel(ep, sram_addr - 1, EP_END_ADDR);

	/* set type, direction, address; reset fifo counters */
	ep->EP_Num = desc->bEndpointAddress & USB_ENDPOINT_NUMBER_MASK;
	ep->EP_Dir = (desc->bEndpointAddress & USB_DIR_IN) ? 1 : 0;
	switch (desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) {
	case USB_ENDPOINT_XFER_ISOC:
		ep->EP_Type = EP_TYPE_ISO;
		ep->EP_Mode = EP_MODE_AUTO;
		break;
	case USB_ENDPOINT_XFER_BULK:
		ep->EP_Type = EP_TYPE_BLK;
		ep->EP_Mode = EP_MODE_AUTO;
		break;
	case USB_ENDPOINT_XFER_INT:
		ep->EP_Type = EP_TYPE_INT;
		ep->EP_Mode = EP_MODE_MAN;
		break;
	default:
		ep->EP_Type = EP_TYPE_BLK;
		ep->EP_Mode = EP_MODE_AUTO;
		break;
	}
	ep_writel(ep, 0x9, EP_RSP_SC);	/* DATA0 and flush SRAM */

	ep_writel(ep, (hshb << 8) | ep->EP_Num << 4 | ep->EP_Dir << 3 |
			ep->EP_Type << 1 | 1, EP_CFG);

	ep_writel(ep, ep->EP_Mode, EP_RSP_SC);

	/* enable irqs */
	int_en_reg = udc_readl(dev, USBD_IRQ_ENB_L);
	udc_writel(dev, int_en_reg | (1 << (ep->index + 1)), USBD_IRQ_ENB_L);
	dev->irq_enbl = udc_readl(dev, USBD_IRQ_ENB_L);

	if (ep->EP_Type == EP_TYPE_BLK) {
		if (ep->EP_Dir)	/* IN */
			ep->irq_enb = EP_IN_TOK;
		else {
			ep->irq_enb = EP_DATA_RXD;
			ep_writel(ep, (ep_readl(ep, EP_RSP_SC) & 0xF7) | 0x80,
				  EP_RSP_SC); /* disable buffer on short packet */
			ep->buffer_disabled = 1;
		}
	} else if (ep->EP_Type == EP_TYPE_INT) {
		if (ep->EP_Dir)	/* IN */
			ep->irq_enb = EP_IN_TOK;
		else
			ep->irq_enb = EP_DATA_RXD | EP_SHORT_PKT;
	} else if (ep->EP_Type == EP_TYPE_ISO) {
		if (ep->EP_Dir)	/* IN */
			ep->irq_enb = EP_IN_TOK;
		else
			ep->irq_enb = EP_OUT_TOK;
	}

	spin_unlock_irqrestore(&dev->lock, flags);

	return 0;
}

static int w55fa93_ep_disable(struct usb_ep *_ep)
{
	struct w55fa93_ep *ep = container_of(_ep, struct w55fa93_ep, ep);
	unsigned long flags;

	if (!_ep || !ep->desc)
		return -EINVAL;

	spin_lock_irqsave(&ep->dev->lock, flags);
	ep->desc = NULL;
	_ep->desc = NULL;

	ep_writel(ep, 0, EP_CFG);
	ep_writel(ep, 0, EP_IRQ_ENB);

	w55fa93_nuke(ep->dev, ep);

	ep_writel(ep, 0, EP_START_ADDR);
	ep_writel(ep, 0, EP_END_ADDR);

	spin_unlock_irqrestore(&ep->dev->lock, flags);

	return 0;
}

static struct usb_request *
w55fa93_alloc_request(struct usb_ep *_ep, gfp_t mem_flags)
{
	struct w55fa93_request *req;

	if (!_ep)
		return NULL;

	req = kzalloc(sizeof(*req), mem_flags);
	if (!req)
		return NULL;
	INIT_LIST_HEAD(&req->queue);
	req->req.dma = DMA_ADDR_INVALID;

	return &req->req;
}

static void
w55fa93_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct w55fa93_request *req;

	if (!_ep || !_req)
		return;

	req = container_of(_req, struct w55fa93_request, req);

	WARN_ON(!list_empty(&req->queue));
	kfree(req);
}

static int
w55fa93_queue(struct usb_ep *_ep, struct usb_request *_req, gfp_t gfp_flags)
{
	struct w55fa93_request *req;
	struct w55fa93_ep *ep;
	struct w55fa93_udc *dev;
	unsigned long flags;

	req = container_of(_req, struct w55fa93_request, req);

	if (unlikely(!_req || !_req->complete || !_req->buf
			|| !list_empty(&req->queue)))
		return -EINVAL;

	ep = container_of(_ep, struct w55fa93_ep, ep);
	if (unlikely(!_ep || (!ep->desc && ep->index != 0)))
		return -EINVAL;

	dev = ep->dev;
	if (unlikely(!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN))
		return -ESHUTDOWN;

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	spin_lock_irqsave(&dev->lock, flags);

	/* pio or dma irq handler advances the queue. */
	list_add_tail(&req->queue, &ep->queue);

	if (ep->index == 0) {	/* delayed status */
		if (dev->setup_ret > 1000 ||
		    ((_req->length == 0) &&
		     (dev->ep0state == EP0_OUT_DATA_PHASE))) {
			/* clear nak so that sts stage is complete */
			udc_writel(dev, CEP_NAK_CLEAR, USBD_CEP_CTRL_STAT);
			/* suppkt int, enb sts completion int */
			udc_writel(dev, CEP_SUPPKT | CEP_STS_END,
				   USBD_CEP_IRQ_ENB);
			w55fa93_done(ep, req, 0);
		} else if (_req->length != 0) {
			dev->ep0state = EP0_IN_DATA_PHASE;
			if (udc_readl(dev, USBD_CEP_CNT) != 0) {
				struct w55fa93_request *first;

				first = list_first_entry(&ep->queue,
						struct w55fa93_request, queue);
				read_fifo(ep, first, 0);
			} else {
				udc_writel(dev, CEP_IN_TOK, USBD_CEP_IRQ_ENB);
			}
		}
	} else {
		if (!dev->usb_dma_trigger || (ep->index != dev->usb_dma_owner))
			ep_writel(ep, ep->irq_enb, EP_IRQ_ENB);
	}

	spin_unlock_irqrestore(&dev->lock, flags);

	return 0;
}

static int w55fa93_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct w55fa93_ep *ep;
	struct w55fa93_udc *udc;
	int retval = -EINVAL;
	unsigned long flags;
	struct w55fa93_request *req = NULL, *iter;

	if (!_ep || !_req)
		return -EINVAL;
	ep = container_of(_ep, struct w55fa93_ep, ep);
	udc = ep->dev;

	if (!udc->driver)
		return -ESHUTDOWN;

	spin_lock_irqsave(&udc->lock, flags);
	list_for_each_entry(iter, &ep->queue, queue) {
		if (&iter->req == _req) {
			req = iter;
			list_del_init(&req->queue);
			retval = 0;
			break;
		}
	}
	spin_unlock_irqrestore(&udc->lock, flags);

	if (retval == 0) {
		/* NOTE: the BSP driver completed the request twice here
		 * (once directly, once via done()); completed once. */
		_req->status = -ECONNRESET;
		if (req->dma_mapped) {
			dma_unmap_single(udc->dev, req->req.dma,
					 req->req.length,
					 ep->EP_Dir ? DMA_TO_DEVICE
						    : DMA_FROM_DEVICE);
			req->req.dma = DMA_ADDR_INVALID;
			req->dma_mapped = 0;
		}
		usb_gadget_giveback_request(_ep, _req);
	}

	return retval;
}

static int w55fa93_set_halt(struct usb_ep *_ep, int value)
{
	struct w55fa93_ep *ep = container_of(_ep, struct w55fa93_ep, ep);
	struct w55fa93_udc *udc = ep->dev;
	unsigned long flags;
	u32 rsp;

	spin_lock_irqsave(&udc->lock, flags);
	if (ep->index == 0) {
		if (value) {
			udc_writel(udc, CEP_SEND_STALL, USBD_CEP_CTRL_STAT);
			udc->ep0state = EP0_STALL;
		} else {
			udc_writel(udc, CEP_NAK_CLEAR, USBD_CEP_CTRL_STAT);
		}
	} else {
		/* Same RSP_SC manipulation the BSP performs when the
		 * SET_FEATURE / CLEAR_FEATURE(ENDPOINT_HALT) control
		 * requests are completed in udc_isr_update_dev(); the
		 * mode bits (EP_MODE_MASK) are preserved. */
		rsp = ep_readl(ep, EP_RSP_SC) & EP_MODE_MASK;
		if (value) {
			ep_writel(ep, EP_HALT | rsp, EP_RSP_SC);
		} else {
			ep_writel(ep, rsp, EP_RSP_SC);
			ep_writel(ep, EP_TOGGLE | rsp, EP_RSP_SC);
		}
		if (ep->EP_Num >= 1 && ep->EP_Num < W55FA93_ENDPOINTS)
			udc->ep_halted[ep->EP_Num - 1] = !!value;
	}
	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;
}

static const struct usb_ep_ops w55fa93_ep_ops = {
	.enable		= w55fa93_ep_enable,
	.disable	= w55fa93_ep_disable,

	.alloc_request	= w55fa93_alloc_request,
	.free_request	= w55fa93_free_request,

	.queue		= w55fa93_queue,
	.dequeue	= w55fa93_dequeue,

	.set_halt	= w55fa93_set_halt,
};

/* ------------------------------------------------------------------ */
/* Gadget operations						    */
/* ------------------------------------------------------------------ */

static int w55fa93_g_get_frame(struct usb_gadget *_gadget)
{
	struct w55fa93_udc *udc = container_of(_gadget, struct w55fa93_udc,
					       gadget);

	return udc_readl(udc, USBD_FRAME_CNT) & 0xffff;
}

static int w55fa93_wakeup(struct usb_gadget *_gadget)
{
	/* TODO(api-check): the BSP driver treated this as a no-op too;
	 * remote wakeup would toggle USB_GEN_RES in USBD_OPER. */
	return 0;
}

static int w55fa93_set_selfpowered(struct usb_gadget *_gadget, int value)
{
	return 0;
}

static int w55fa93_pullup(struct usb_gadget *_gadget, int is_on)
{
	struct w55fa93_udc *dev;
	unsigned long flags;

	if (!_gadget)
		return -ENODEV;

	dev = container_of(_gadget, struct w55fa93_udc, gadget);

	spin_lock_irqsave(&dev->lock, flags);

	/* Magic values taken verbatim from the BSP driver: 0x320 turns
	 * the PHY on with the D+ pull-up attached, 0x220 detaches it.
	 * (USBD_PHY_CTL fields are only partially documented in the BSP
	 * register header: bit7 = clk48, bit9 = Phy_suspend.) */
	if (is_on)
		udc_writel(dev, 0x320, USBD_PHY_CTL);
	else
		udc_writel(dev, 0x220, USBD_PHY_CTL);

	spin_unlock_irqrestore(&dev->lock, flags);

	return 0;
}

static int w55fa93_udc_start(struct usb_gadget *gadget,
			     struct usb_gadget_driver *driver)
{
	struct w55fa93_udc *udc = container_of(gadget, struct w55fa93_udc,
					       gadget);

	/* hook up the driver; the actual speed is only known after the
	 * first bus reset, see udc_isr_rst(). */
	udc->driver = driver;
	udc->gadget.speed = USB_SPEED_UNKNOWN;
	udc->ep0state = EP0_IDLE;

	return 0;
}

static int w55fa93_udc_stop(struct usb_gadget *gadget)
{
	struct w55fa93_udc *udc = container_of(gadget, struct w55fa93_udc,
					       gadget);
	unsigned long flags;

	spin_lock_irqsave(&udc->lock, flags);

	/* power off usb D+ pull-up, as the BSP unregister path did */
	udc_writel(udc, udc_readl(udc, USBD_PHY_CTL) & ~0x100, USBD_PHY_CTL);

	clear_ep_state(udc);

	udc->driver = NULL;
	udc->gadget.speed = USB_SPEED_UNKNOWN;

	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;
}

static const struct usb_gadget_ops w55fa93_gadget_ops = {
	.get_frame		= w55fa93_g_get_frame,
	.wakeup			= w55fa93_wakeup,
	.set_selfpowered	= w55fa93_set_selfpowered,
	.pullup			= w55fa93_pullup,
	.udc_start		= w55fa93_udc_start,
	.udc_stop		= w55fa93_udc_stop,
};

/* ------------------------------------------------------------------ */
/* Interrupt service helpers (called with udc->lock held)		    */
/* ------------------------------------------------------------------ */

static void udc_isr_rst(struct w55fa93_udc *dev)
{
	int i;

	clear_ep_state(dev);

	dev->usb_devstate = 0;
	dev->usb_address = 0;
	dev->usb_less_mps = 0;
	dev->usb_dma_trigger = 0;
	dev->usb_dma_trigger_next = 0;

	/* reset DMA */
	udc_writel(dev, 0x80, USBD_DMA_CTRL_STS);
	udc_writel(dev, 0x00, USBD_DMA_CTRL_STS);
	udc_writel(dev, 0, USBD_HEAD_WORD0);
	udc_writel(dev, 0, USBD_EP_HEAD_CNT(1));

	dev->usb_devstate = 1;	/* default state */

	if ((udc_readl(dev, USBD_OPER) & USB_CUR_SPD_HS) == 0)
		dev->gadget.speed = USB_SPEED_FULL;
	else
		dev->gadget.speed = USB_SPEED_HIGH;
	usb_gadget_set_state(&dev->gadget, USB_STATE_DEFAULT);

	/* flush fifo */
	udc_writel(dev, udc_readl(dev, USBD_CEP_CTRL_STAT) | CEP_FLUSH,
		   USBD_CEP_CTRL_STAT);
	for (i = 1; i < W55FA93_ENDPOINTS; i++)
		ep_writel(&dev->ep[i], 0x09, EP_RSP_SC);	/* flush fifo */

	udc_writel(dev, 0, USBD_ADDR);
	udc_writel(dev, CEP_SUPPKT, USBD_CEP_IRQ_ENB);
}

static void udc_isr_dma(struct w55fa93_udc *dev)
{
	struct w55fa93_request *req;
	struct w55fa93_ep *ep;

	if (!dev->usb_dma_trigger) {
		dev_dbg(dev->dev, "DMA not trigger, intr?\n");
		return;
	}

	ep = &dev->ep[dev->usb_dma_owner];

	if (dev->usb_dma_dir == Ep_In)
		ep_writel(ep, EP_IN_TOK, EP_IRQ_STAT);

	dev->usb_dma_trigger = 0;

	if (list_empty(&ep->queue)) {
		dev_dbg(dev->dev, "DMA ep->queue is empty\n");
		udc_writel(dev, dev->irq_enbl, USBD_IRQ_ENB_L);
		return;
	}

	req = list_first_entry(&ep->queue, struct w55fa93_request, queue);

	if (ep->EP_Type == EP_TYPE_BLK) {
		if (dev->usb_less_mps == 1) {
			/* packet end */
			ep_writel(ep, (ep_readl(ep, EP_RSP_SC) & 0xF7) |
				  EP_PKT_END, EP_RSP_SC);
			dev->usb_less_mps = 0;
		}
	} else if (ep->EP_Type == EP_TYPE_INT) {
		ep_writel(ep, dev->usb_dma_cnt, EP_TRF_CNT);
	} else if (ep->EP_Type == EP_TYPE_ISO) {
		if (udc_readl(dev, USBD_HEAD_WORD0) != 0) {
			ep_writel(ep, EP_DATA_TXD, EP_IRQ_STAT);
			ep_writel(ep, EP_DATA_TXD | ep_readl(ep, EP_IRQ_ENB),
				  EP_IRQ_ENB);

			if ((ep_readl(ep, EP_DATA_CNT) & 0xFFFF) <
					(ep_readl(ep, EP_MPS) + 2)) {
				if (dev->usb_dma_trigger_next == 0) {
					udc_writel(dev,
						udc_readl(dev, USBD_HEAD_WORD0) |
						0x200, USBD_HEAD_WORD0);
					udc_writel(dev, 2,
						   USBD_EP_HEAD_CNT(ep->index));
					/* packet end */
					ep_writel(ep,
						(ep_readl(ep, EP_RSP_SC) & 0xF7) |
						EP_PKT_END, EP_RSP_SC);
				}
			}
		} else {
			if ((ep_readl(ep, EP_DATA_CNT) & 0xFFFF) >=
					ep_readl(ep, EP_MPS)) {
				ep_writel(ep, EP_DATA_TXD, EP_IRQ_STAT);
				ep_writel(ep,
					EP_DATA_TXD | ep_readl(ep, EP_IRQ_ENB),
					EP_IRQ_ENB);
			} else {
				/* packet end */
				ep_writel(ep, (ep_readl(ep, EP_RSP_SC) & 0xF7) |
					  EP_PKT_END, EP_RSP_SC);
			}
		}
	}

	/* The BSP accounts progress by adding the (remaining) DMA_CNT
	 * register value here; kept as-is. */
	req->req.actual += udc_readl(dev, USBD_DMA_CNT);

	if ((req->req.length == req->req.actual) ||
			dev->usb_dma_cnt < ep->ep.maxpacket) {
		udc_writel(dev, dev->irq_enbl, USBD_IRQ_ENB_L);
		if ((ep->EP_Type == EP_TYPE_BLK) && (ep->EP_Dir == 0) &&
				dev->usb_dma_cnt < ep->ep.maxpacket) {
			if (ep->buffer_disabled) {
				/* enable buffer */
				ep_writel(ep, ep_readl(ep, EP_RSP_SC) & 0x77,
					  EP_RSP_SC);
				/* disable buffer when short packet */
				ep_writel(ep, (ep_readl(ep, EP_RSP_SC) & 0xF7) |
					  0x80, EP_RSP_SC);
			}
		}
		if (ep->EP_Type == EP_TYPE_ISO) {
			if (udc_readl(dev, USBD_HEAD_WORD0) == 0)
				w55fa93_done(ep, req, 0);
		} else {
			w55fa93_done(ep, req, 0);
		}
		return;
	}

	if (dev->usb_dma_dir == Ep_Out) {
		if (dev->usb_dma_trigger_next) {
			dev->usb_dma_trigger_next = 0;
			read_fifo(ep, req, 0);
		}
	} else if (dev->usb_dma_dir == Ep_In) {
		if (dev->usb_less_mps == 1)
			dev->usb_less_mps = 0;

		if (dev->usb_dma_trigger_next) {
			dev->usb_dma_trigger_next = 0;
			write_fifo(ep, req);
		}
	}
}

static void get_setup_packet(struct w55fa93_udc *dev,
			     struct usb_ctrlrequest *pcrq, u32 temp)
{
	pcrq->bRequestType = (u8)temp & 0xff;
	pcrq->bRequest = (u8)(temp >> 8) & 0xff;
	pcrq->wValue = cpu_to_le16(udc_readl(dev, USBD_SETUP3_2) & 0xffff);
	pcrq->wIndex = cpu_to_le16(udc_readl(dev, USBD_SETUP5_4) & 0xffff);
	pcrq->wLength = cpu_to_le16(udc_readl(dev, USBD_SETUP7_6) & 0xffff);
}

static void udc_isr_update_dev(struct w55fa93_udc *dev)
{
	struct usb_ctrlrequest *pcrq = &dev->crq;
	int i, index = 0;

	/* update this device for set requests */
	switch (pcrq->bRequest) {
	case USB_REQ_SET_ADDRESS:
		udc_writel(dev, dev->usb_address, USBD_ADDR);
		break;

	case USB_REQ_SET_CONFIGURATION:
		break;

	case USB_REQ_SET_INTERFACE:
		break;

	case USB_REQ_SET_FEATURE:
		if ((pcrq->bRequestType & 0x3) == 0x0) { /* recipient is device */
			if ((le16_to_cpu(pcrq->wValue) & 0x3) == 0x2) {
				udc_writel(dev, le16_to_cpu(pcrq->wIndex) >> 8,
					   USBD_MEM_TEST);
				dev_dbg(dev->dev, "TEST PACKET\n");
			}
		}
		for (i = 1; i < W55FA93_ENDPOINTS; i++) {
			if (dev->ep[i].EP_Num == dev->halt_ep) {
				index = i;
				break;
			}
		}
		if (dev->halt_ep == 0) {
			udc_writel(dev, CEP_SEND_STALL, USBD_CEP_CTRL_STAT);
		} else if (index) {
			ep_writel(&dev->ep[index],
				  EP_HALT | (ep_readl(&dev->ep[index],
						      EP_RSP_SC) & 0x06),
				  EP_RSP_SC);
		}
		break;

	case USB_REQ_CLEAR_FEATURE:
		for (i = 1; i < W55FA93_ENDPOINTS; i++) {
			if (dev->ep[i].EP_Num == dev->halt_ep) {
				index = i;
				break;
			}
		}
		if (dev->halt_ep != -1 && index) {
			ep_writel(&dev->ep[index],
				  ep_readl(&dev->ep[index], EP_RSP_SC) & 0x06,
				  EP_RSP_SC);
			ep_writel(&dev->ep[index],
				  EP_TOGGLE | (ep_readl(&dev->ep[index],
							EP_RSP_SC) & 0x06),
				  EP_RSP_SC);
			dev->halt_ep = -1;	/* just for changing the value */
		}
		break;

	default:
		break;
	}
}

/*
 * TODO(api-check): like the BSP driver, this UDC intercepts
 * SET_ADDRESS / SET_CONFIGURATION for its own device-state tracking
 * and answers GET_STATUS / SET_FEATURE / CLEAR_FEATURE for endpoint
 * halt itself without forwarding them to the gadget driver.  That
 * matched the 2.6.35-era gadget drivers this controller shipped
 * with.  Modern composite/configfs gadget drivers also implement
 * parts of these standard requests in their ->setup() (e.g.
 * GET_STATUS for interfaces); if enumeration with g_ether behaves
 * oddly, this interception is the first place to re-check.
 */
static void udc_isr_ctrl_pkt(struct w55fa93_udc *dev)
{
	u32 temp;
	u32 ReqErr = 0;
	struct w55fa93_ep *ep = &dev->ep[0];
	struct usb_ctrlrequest crq;
	u16 wValue, wIndex, wLength;
	int ret;

	temp = udc_readl(dev, USBD_SETUP1_0);

	get_setup_packet(dev, &crq, temp);
	wValue = le16_to_cpu(crq.wValue);
	wIndex = le16_to_cpu(crq.wIndex);
	wLength = le16_to_cpu(crq.wLength);

	dev->crq = crq;

	switch (dev->ep0state) {
	case EP0_IDLE:
		switch (crq.bRequest) {
		case USB_REQ_SET_ADDRESS:
			ReqErr = ((crq.bRequestType == 0) &&
				  ((wValue & 0xff00) == 0) &&
				  (wIndex == 0) && (wLength == 0)) ? 0 : 1;

			if ((wValue & 0xffff) > 0x7f)	/* within 7f */
				ReqErr = 1;	/* Devaddr > 127 */

			if (dev->usb_devstate == 3)	/* Dev is configured */
				ReqErr = 1;

			if (ReqErr == 1)
				break;

			if (dev->usb_devstate == 2) {
				if (wValue == 0)
					dev->usb_devstate = 1;	/* default state */
				dev->usb_address = wValue;
			}

			if (dev->usb_devstate == 1) {
				if (wValue != 0) {
					dev->usb_address = wValue;
					dev->usb_devstate = 2;
				}
			}
			break;

		case USB_REQ_SET_CONFIGURATION:
			if (crq.bRequestType & 0x20) {
				/* For Class Set Configuration */
				ReqErr = (((wValue & 0xf000) == 0) &&
					  ((wValue & 0x80) == 0) &&
					  (wIndex == 0)) ? 0 : 1;
			} else {
				ReqErr = ((crq.bRequestType == 0) &&
					  ((wValue & 0xff00) == 0) &&
					  ((wValue & 0x80) == 0) &&
					  (wIndex == 0) &&
					  (wLength == 0)) ? 0 : 1;
			}

			if (dev->usb_devstate == 1)
				ReqErr = 1;

			if (ReqErr == 1)
				break;

			if (wValue == 0)
				dev->usb_devstate = 2;
			else
				dev->usb_devstate = 3;
			break;

		case USB_REQ_SET_INTERFACE:
			break;

		case USB_REQ_SET_FEATURE:
			if ((crq.bRequestType & 0x3) == 0x2) {
				if ((wValue & 0x3) == 0) {
					dev->halt_ep = wIndex & 0xF;
					if (dev->halt_ep >= 1 &&
					    dev->halt_ep < W55FA93_ENDPOINTS)
						dev->ep_halted[dev->halt_ep - 1] = 1;
				}
			} else {
				break;
			}

			if (ReqErr == 1)
				break;

			udc_writel(dev, CEP_STS_END, USBD_CEP_IRQ_STAT);
			/* Enable IN/RxED/status complete interrupt */
			udc_writel(dev, CEP_STS_END, USBD_CEP_IRQ_ENB);
			/* Clear nak so that sts stage is complete */
			udc_writel(dev, CEP_NAK_CLEAR, USBD_CEP_CTRL_STAT);
			return;

		case USB_REQ_CLEAR_FEATURE:
			if ((crq.bRequestType & 0x3) == 0x2) {
				if ((wValue & 0x3) == 0) {
					dev->halt_ep = wIndex & 0xF;
					if (dev->halt_ep >= 1 &&
					    dev->halt_ep < W55FA93_ENDPOINTS)
						dev->ep_halted[dev->halt_ep - 1] = 0;
				}
			} else {
				break;
			}

			if (ReqErr == 1)
				break;

			udc_writel(dev, CEP_STS_END, USBD_CEP_IRQ_STAT);
			udc_writel(dev, CEP_STS_END, USBD_CEP_IRQ_ENB);
			udc_writel(dev, CEP_NAK_CLEAR, USBD_CEP_CTRL_STAT);
			return;

		case USB_REQ_GET_STATUS:
			if ((crq.bRequestType & 0x3) == 0x0) {		/* USB */
				dev->status_word = 0;
			} else if ((crq.bRequestType & 0x3) == 0x2) {	/* Endpoint */
				if ((wIndex & 0x3) == 0)	/* Control Endpoint */
					dev->status_word = 0;
				else if ((wIndex & 0xF) >= 1 &&
					 (wIndex & 0xF) < W55FA93_ENDPOINTS)
					dev->status_word =
						dev->ep_halted[(wIndex & 0xF) - 1];
				else
					dev->status_word = 0;
			} else {
				break;
			}

			if (ReqErr == 1)
				break;

			if (crq.bRequestType & USB_DIR_IN) {
				dev->ep0state = EP0_IN_DATA_PHASE;
				udc_writel(dev, CEP_IN_TOK, USBD_CEP_IRQ_ENB);
			} else {
				dev->ep0state = EP0_OUT_DATA_PHASE;
				udc_writel(dev, CEP_DATA_RXD, USBD_CEP_IRQ_ENB);
			}
			return;

		default:
			break;
		} /* switch end */

		if (crq.bRequestType & USB_DIR_IN) {
			dev->ep0state = EP0_IN_DATA_PHASE;
			udc_writel(dev, CEP_IN_TOK, USBD_CEP_IRQ_ENB);
		} else {
			dev->ep0state = EP0_OUT_DATA_PHASE;
			udc_writel(dev, CEP_DATA_RXD, USBD_CEP_IRQ_ENB);
		}

		/* The gadget driver's ->setup() may queue requests and
		 * enable endpoints, which take udc->lock; drop it here.
		 * (The BSP driver called ->setup() with no lock held.) */
		spin_unlock(&dev->lock);
		ret = dev->driver->setup(&dev->gadget, &crq);
		spin_lock(&dev->lock);
		dev->setup_ret = ret;
		if (ret < 0) {
			udc_writel(dev, CEP_STS_END, USBD_CEP_IRQ_STAT);
			/* enable in/RxED/status complete interrupt */
			udc_writel(dev, CEP_DATA_RXD | CEP_IN_TOK |
				   CEP_STS_END, USBD_CEP_IRQ_ENB);
			/* clear nak so that sts stage is complete */
			udc_writel(dev, CEP_NAK_CLEAR, USBD_CEP_CTRL_STAT);

			if (ret == -EOPNOTSUPP) {
				if (crq.bRequest != USB_REQ_SET_ADDRESS)
					dev_dbg(dev->dev,
						"Request 0x%x is not supported\n",
						crq.bRequest);
			} else {
				dev_dbg(dev->dev,
					"driver->setup failed. (%d)\n", ret);
			}
		} else if (ret > 1000) {	/* DELAYED_STATUS */
			dev->ep0state = EP0_END_XFER;
			udc_writel(dev, 0, USBD_CEP_IRQ_ENB);
		}
		break;

	case EP0_STALL:
		break;
	default:
		break;
	}

	if (ReqErr == 1) {
		udc_writel(dev, CEP_SEND_STALL, USBD_CEP_CTRL_STAT);
		dev->ep0state = EP0_STALL;
	}
}

/* ------------------------------------------------------------------ */
/* Interrupt dispatchers (called with udc->lock held)		    */
/* ------------------------------------------------------------------ */

static void handle_usb_stat(u32 irq, struct w55fa93_udc *dev)
{
	udc_writel(dev, irq, USBD_IRQ_STAT);	/* clear irq bit */
	switch (irq) {
	case USB_VBUS_STS:
		/* The BSP driver reported plug/unplug to a system
		 * manager and ran an eject-detection timer; here we
		 * only track the gadget state. */
		if (udc_readl(dev, USBD_PHY_CTL) & PHY_VBUS_STATUS) {
			dev->vbus = true;
			usb_gadget_set_state(&dev->gadget, USB_STATE_ATTACHED);
			dev_dbg(dev->dev, "<USBD - USBD plug>\n");
		} else {
			dev->vbus = false;
			usb_gadget_set_state(&dev->gadget,
					     USB_STATE_NOTATTACHED);
			dev_dbg(dev->dev, "<USBD - USBD Un-plug>\n");
		}
		break;

	case USB_SOF:
		break;

	case USB_RST_STS:	/* reset */
		udc_isr_rst(dev);
		break;

	case USB_RESUME:
		udc_writel(dev, (USB_RST_STS | USB_SUS_REQ | USB_VBUS_STS),
			   USBD_IRQ_ENB);
		if (dev->usb_devstate == 3)
			usb_gadget_set_state(&dev->gadget, USB_STATE_CONFIGURED);
		else if (dev->usb_devstate == 2)
			usb_gadget_set_state(&dev->gadget, USB_STATE_ADDRESSED);
		else
			usb_gadget_set_state(&dev->gadget, USB_STATE_DEFAULT);
		break;

	case USB_SUS_REQ:
		udc_writel(dev, (USB_RST_STS | USB_RESUME | USB_VBUS_STS),
			   USBD_IRQ_ENB);
		usb_gadget_set_state(&dev->gadget, USB_STATE_SUSPENDED);
		break;

	case USB_HS_SETTLE:
		dev->usb_devstate = 1;	/* default state */
		dev->usb_address = 0;
		udc_writel(dev, CEP_SUPPKT, USBD_CEP_IRQ_ENB);
		break;

	case USB_DMA_REQ:
		udc_isr_dma(dev);
		break;

	case USABLE_CLK:
		break;

	default:
		break;
	}
}

static void handle_cep(u32 irq, struct w55fa93_udc *dev)
{
	struct w55fa93_ep *ep = &dev->ep[0];
	struct w55fa93_request *req;
	int is_last = 1;

	if (list_empty(&ep->queue))
		req = NULL;
	else
		req = list_first_entry(&ep->queue, struct w55fa93_request,
				       queue);

	switch (irq) {
	case CEP_SUPPKT:	/* receive setup packet */
		dev->ep0state = EP0_IDLE;
		dev->setup_ret = 0;
		udc_isr_ctrl_pkt(dev);
		break;

	case CEP_DATA_RXD:
		if (req)
			is_last = read_fifo(ep, req, 0);

		udc_writel(dev, CEP_STS_END, USBD_CEP_IRQ_STAT);

		if (!is_last) {
			/* enable out token and status complete int */
			udc_writel(dev, CEP_DATA_RXD | CEP_STS_END,
				   USBD_CEP_IRQ_ENB);
		} else {	/* transfer finished */
			/* clear nak so that sts stage is complete */
			udc_writel(dev, CEP_NAK_CLEAR, USBD_CEP_CTRL_STAT);
			udc_writel(dev, CEP_SUPPKT | CEP_OUT_TOK | CEP_IN_TOK |
				   CEP_PING_TOK | CEP_DATA_TXD | CEP_STS_END,
				   USBD_CEP_IRQ_ENB);
			dev->ep0state = EP0_END_XFER;
		}
		return;

	case CEP_IN_TOK:
		if (dev->ep0state == EP0_IN_DATA_PHASE) {
			if (req) {
				is_last = write_fifo(ep, req);
			} else if (dev->status_word != -1) {
				/* pending GET_STATUS reply */
				writeb(dev->status_word,
				       dev->base + USBD_CEP_DATA_BUF);
				writeb(0, dev->base + USBD_CEP_DATA_BUF);
				udc_writel(dev, 2, USBD_IN_TRNSFR_CNT);
				is_last = 1;
				dev->status_word = -1;
			}
			if (!is_last) {
				udc_writel(dev, CEP_IN_TOK, USBD_CEP_IRQ_ENB);
			} else {
				if (dev->setup_ret >= 0)
					/* clear nak so sts stage completes */
					udc_writel(dev, CEP_NAK_CLEAR,
						   USBD_CEP_CTRL_STAT);
				/* suppkt int, enb sts completion int */
				udc_writel(dev, CEP_SUPPKT | CEP_IN_TOK |
					   CEP_STS_END, USBD_CEP_IRQ_ENB);

				if (dev->setup_ret < 0)
					dev->ep0state = EP0_IDLE;
				else if (dev->ep0state != EP0_IDLE)
					dev->ep0state = EP0_END_XFER;
			}
		}
		return;

	case CEP_PING_TOK:
		/* suppkt int, enb sts completion int */
		udc_writel(dev, CEP_SUPPKT | CEP_STS_END, USBD_CEP_IRQ_ENB);
		return;

	case CEP_DATA_TXD:
		return;

	case CEP_STS_END:
		udc_writel(dev, CEP_SUPPKT, USBD_CEP_IRQ_ENB);
		udc_isr_update_dev(dev);
		dev->ep0state = EP0_IDLE;
		dev->setup_ret = 0;
		break;

	default:
		break;
	}
}

/* Bulk / ISO non-control endpoint interrupts (BSP paser_irq_nep) */
static void handle_ep(u32 irq, struct w55fa93_ep *ep)
{
	struct w55fa93_udc *dev = ep->dev;
	struct w55fa93_request *req;
	int i;
	u16 fifo_count, loop;
	u8 *buf;
	u8 data;

	if (list_empty(&ep->queue)) {
		dev_dbg(dev->dev, "nep->queue is empty\n");
		req = NULL;
	} else {
		ep_writel(ep, ep_readl(ep, EP_IRQ_STAT), EP_IRQ_STAT);
		req = list_first_entry(&ep->queue, struct w55fa93_request,
				       queue);
	}

	switch (irq) {
	case EP_IN_TOK:
		ep_writel(ep, irq, EP_IRQ_STAT);

		if (ep->EP_Type == EP_TYPE_BLK) {
			if (ep_readl(ep, EP_RSP_SC) & EP_PKT_END) {
				/* send last packet */
				break;
			}
		}
		if (!req) {
			ep_writel(ep, 0, EP_IRQ_ENB);
			break;
		}

		/* wait DMA complete */
		while (udc_readl(dev, USBD_DMA_CTRL_STS) & 0x20) {
			if (!(udc_readl(dev, USBD_PHY_CTL) & PHY_VBUS_STATUS)) {
				dev_dbg(dev->dev, "Un-Plug (0)!!\n");
				break;
			}
		}
		if (dev->usb_dma_trigger) {
			while ((udc_readl(dev, USBD_IRQ_STAT) & USB_DMA_REQ) == 0) {
				if (!(udc_readl(dev, USBD_PHY_CTL) &
						PHY_VBUS_STATUS)) {
					dev_dbg(dev->dev, "Un-Plug (1)!!\n");
					break;
				}
			}
			udc_writel(dev, USB_DMA_REQ, USBD_IRQ_STAT);
			udc_isr_dma(dev);
		}

		write_fifo(ep, req);
		break;

	case EP_BO_SHORT_PKT:
		if (!req) {
			ep_writel(ep, 0, EP_IRQ_ENB);
			break;
		}

		if (dev->usb_dma_trigger) {
			loop = ep_readl(ep, EP_DATA_CNT) >> 16;
			loop = dev->usb_dma_loop - loop;

			if (loop)
				req->req.actual += loop * 32;	/* each loop 32 bytes */
			dev->usb_dma_trigger = 0;
			/* reset DMA */
			udc_writel(dev, 0x80, USBD_DMA_CTRL_STS);
			udc_writel(dev, 0x00, USBD_DMA_CTRL_STS);

			udc_writel(dev, dev->irq_enbl, USBD_IRQ_ENB_L);
		}

		/* NOTE: the BSP used the raw DATA_CNT value here; the
		 * byte count lives in the low 16 bits (as everywhere
		 * else in this driver), so mask it. */
		fifo_count = ep_readl(ep, EP_DATA_CNT) & 0xffff;

		buf = req->req.buf + req->req.actual;

		for (i = 0; i < fifo_count; i++) {
			data = readb(dev->base + USBD_EP_BASE(ep->index) +
				     EP_DATA_BUF);
			*buf++ = data & 0xFF;
		}
		if (ep->buffer_disabled) {
			/* enable buffer */
			ep_writel(ep, ep_readl(ep, EP_RSP_SC) & 0x77, EP_RSP_SC);
			/* disable buffer when short packet */
			ep_writel(ep, (ep_readl(ep, EP_RSP_SC) & 0xF7) | 0x80,
				  EP_RSP_SC);
		}

		req->req.actual += fifo_count;

		w55fa93_done(ep, req, 0);
		break;

	case EP_DATA_RXD:
		if (!req) {
			ep_writel(ep, 0, EP_IRQ_ENB);
			break;
		}
		if (ep_readl(ep, EP_DATA_CNT) == 0)
			break;

		/* wait DMA complete */
		while (udc_readl(dev, USBD_DMA_CTRL_STS) & 0x20) {
			if (!(udc_readl(dev, USBD_PHY_CTL) & PHY_VBUS_STATUS)) {
				dev_dbg(dev->dev, "Un-Plug (2)!!\n");
				break;
			}
		}

		if (dev->usb_dma_trigger) {
			while ((udc_readl(dev, USBD_IRQ_STAT) & USB_DMA_REQ) == 0) {
				if (!(udc_readl(dev, USBD_PHY_CTL) &
						PHY_VBUS_STATUS)) {
					dev_dbg(dev->dev, "Un-Plug (3)!!\n");
					break;
				}
			}
			udc_writel(dev, USB_RST_STS, USBD_IRQ_STAT);
			udc_isr_dma(dev);
		}

		read_fifo(ep, req, ep_readl(ep, EP_DATA_CNT));
		break;

	case EP_DATA_TXD:
		if (!req) {
			ep_writel(ep, 0, EP_IRQ_ENB);
			break;
		}

		if (udc_readl(dev, USBD_HEAD_WORD0) != 0) {
			if ((ep_readl(ep, EP_DATA_CNT) & 0xFFFF) >=
					(ep_readl(ep, EP_MPS) + 2)) {
				ep_writel(ep, EP_DATA_TXD |
					  ep_readl(ep, EP_IRQ_ENB), EP_IRQ_ENB);
			} else if ((ep_readl(ep, EP_DATA_CNT) & 0xFFFF) == 0) {
				ep_writel(ep, ~EP_DATA_TXD &
					  ep_readl(ep, EP_IRQ_ENB), EP_IRQ_ENB);
				dev->iso_frame_count++;
				w55fa93_done(ep, req, 0);
			} else {
				udc_writel(dev,
					udc_readl(dev, USBD_HEAD_WORD0) | 0x200,
					USBD_HEAD_WORD0);
				udc_writel(dev, 2, USBD_EP_HEAD_CNT(ep->index));
				/* packet end */
				ep_writel(ep, (ep_readl(ep, EP_RSP_SC) & 0xF7) |
					  EP_PKT_END, EP_RSP_SC);
			}
		} else {
			if ((ep_readl(ep, EP_DATA_CNT) & 0xFFFF) >=
					ep_readl(ep, EP_MPS)) {
				ep_writel(ep, EP_DATA_TXD |
					  ep_readl(ep, EP_IRQ_ENB), EP_IRQ_ENB);
			} else {
				/* packet end */
				ep_writel(ep, (ep_readl(ep, EP_RSP_SC) & 0xF7) |
					  EP_PKT_END, EP_RSP_SC);
				ep_writel(ep, ~EP_DATA_TXD &
					  ep_readl(ep, EP_IRQ_ENB), EP_IRQ_ENB);
				w55fa93_done(ep, req, 0);
			}
		}
		break;

	default:
		dev_dbg(dev->dev, "irq: %d not handled !\n", irq);
		ep_writel(ep, irq, EP_IRQ_STAT);
		break;
	}
}

/* Interrupt-type non-control endpoint interrupts (BSP paser_irq_nepint) */
static void handle_ep_int(u32 irq, struct w55fa93_ep *ep)
{
	struct w55fa93_udc *dev = ep->dev;
	struct w55fa93_request *req;

	ep_writel(ep, irq, EP_IRQ_STAT);

	if (list_empty(&ep->queue)) {
		dev_dbg(dev->dev, "nepirq->queue is empty\n");
		return;
	}

	req = list_first_entry(&ep->queue, struct w55fa93_request, queue);

	switch (irq) {
	case EP_IN_TOK:
		/* wait DMA complete */
		while (udc_readl(dev, USBD_DMA_CTRL_STS) & 0x20) {
			if (!(udc_readl(dev, USBD_PHY_CTL) & PHY_VBUS_STATUS)) {
				dev_dbg(dev->dev, "Un-Plug (4)!!\n");
				break;
			}
		}
		if (dev->usb_dma_trigger) {
			while ((udc_readl(dev, USBD_IRQ_STAT) & USB_DMA_REQ) == 0) {
				if (!(udc_readl(dev, USBD_PHY_CTL) &
						PHY_VBUS_STATUS)) {
					dev_dbg(dev->dev, "Un-Plug (5)!!\n");
					break;
				}
			}
			udc_writel(dev, USB_DMA_REQ, USBD_IRQ_STAT);
			udc_isr_dma(dev);
		}
		write_fifo(ep, req);
		break;

	case EP_DATA_RXD:
	case EP_BO_SHORT_PKT:
		if (ep_readl(ep, EP_DATA_CNT) == 0)
			break;

		/* wait DMA complete */
		while (udc_readl(dev, USBD_DMA_CTRL_STS) & 0x20) {
			if (!(udc_readl(dev, USBD_PHY_CTL) & PHY_VBUS_STATUS)) {
				dev_dbg(dev->dev, "Un-Plug (2)!!\n");
				break;
			}
		}

		if (dev->usb_dma_trigger) {
			while ((udc_readl(dev, USBD_IRQ_STAT) & USB_DMA_REQ) == 0) {
				if (!(udc_readl(dev, USBD_PHY_CTL) &
						PHY_VBUS_STATUS)) {
					dev_dbg(dev->dev, "Un-Plug (3)!!\n");
					break;
				}
			}
			udc_writel(dev, USB_RST_STS, USBD_IRQ_STAT);
			udc_isr_dma(dev);
		}

		read_fifo(ep, req, ep_readl(ep, EP_DATA_CNT));
		break;

	default:
		dev_dbg(dev->dev, "irq: %d not handled !\n", irq);
		ep_writel(ep, irq, EP_IRQ_STAT);
		break;
	}
}

static irqreturn_t w55fa93_udc_irq(int irq, void *_dev)
{
	struct w55fa93_udc *dev = _dev;
	struct w55fa93_ep *ep;
	u32 IrqStL, IrqEnL;
	u32 IrqSt, IrqEn;
	int i, j;

	spin_lock(&dev->lock);

	IrqStL = udc_readl(dev, USBD_IRQ_STAT_L);	/* get interrupt status */
	IrqEnL = udc_readl(dev, USBD_IRQ_ENB_L);

	IrqStL = IrqStL & IrqEnL;
	if (!IrqStL) {
		spin_unlock(&dev->lock);
		return IRQ_NONE;
	}

	if (IrqStL & IRQ_USB_STAT) {
		IrqSt = udc_readl(dev, USBD_IRQ_STAT);
		IrqEn = udc_readl(dev, USBD_IRQ_ENB);

		IrqSt = IrqSt & IrqEn;

		if (IrqSt && (dev->driver || (IrqSt & USB_VBUS_STS))) {
			for (i = 0; i < 9; i++) {
				if (IrqSt & (1 << i)) {
					handle_usb_stat(1 << i, dev);
					break;
				}
			}
		}
		udc_writel(dev, IrqSt, USBD_IRQ_STAT);
	} /* end IRQ_USB_STAT */

	if (IrqStL & IRQ_CEP) {
		IrqSt = udc_readl(dev, USBD_CEP_IRQ_STAT);
		IrqEn = udc_readl(dev, USBD_CEP_IRQ_ENB);

		IrqSt = IrqSt & IrqEn;

		if (IrqSt && dev->driver) {
			if (IrqSt & CEP_STS_END) {	/* deal with STS END */
				if (dev->ep0state == EP0_OUT_DATA_PHASE)
					IrqSt &= 0x1BF7;
				handle_cep(CEP_STS_END, dev);
			}
			for (i = 0; i < 13; i++) {
				if (i == 10)
					continue;
				if (IrqSt & (1 << i))
					handle_cep(1 << i, dev);
			}
		}
		udc_writel(dev, IrqSt, USBD_CEP_IRQ_STAT);
	}

	if (IrqStL & IRQ_NCEP) {
		IrqStL >>= 2;

		for (j = 0; j < 6; j++) {	/* 6 endpoints */
			if (IrqStL & (1 << j)) {
				/* in-token and out token interrupt can
				 * deal with one only */
				ep = &dev->ep[j + 1];
				IrqSt = ep_readl(ep, EP_IRQ_STAT);
				IrqEn = ep_readl(ep, EP_IRQ_ENB);

				IrqSt = IrqSt & IrqEn;
				if (IrqSt && dev->driver) {
					for (i = 12; i >= 0; i--) {
						if (IrqSt & (1 << i)) {
							if ((1 << i) == EP_BO_SHORT_PKT)
								/* clear out token/RxED intr */
								IrqSt &= 0x1FCF;
							if ((ep->EP_Type ==
								EP_TYPE_BLK) ||
							    (ep->EP_Type ==
								EP_TYPE_ISO))
								handle_ep(1 << i, ep);
							else if (ep->EP_Type ==
								EP_TYPE_INT)
								handle_ep_int(1 << i, ep);
							break;
						}
					}
				}
			}
		}
	} /* if end */

	spin_unlock(&dev->lock);

	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/* Platform driver						    */
/* ------------------------------------------------------------------ */

static void w55fa93_usb_init(struct w55fa93_udc *dev)
{
	int j;

	dev->usb_devstate = 0;
	dev->usb_address = 0;

	/* configure USB controller */
	udc_writel(dev, 0x03, USBD_IRQ_ENB_L);	/* enable usb, cep interrupt */
	udc_writel(dev, (USB_RESUME | USB_RST_STS | USB_VBUS_STS),
		   USBD_IRQ_ENB);
	udc_writel(dev, USB_HS, USBD_OPER);	/* USB 2.0 */
	udc_writel(dev, 0, USBD_ADDR);
	udc_writel(dev, (CEP_SUPPKT | CEP_STS_END), USBD_CEP_IRQ_ENB);

	for (j = 0; j < W55FA93_ENDPOINTS; j++) {
		dev->ep[j].EP_Num = 0xff;
		dev->ep[j].EP_Dir = 0xff;
		dev->ep[j].EP_Type = 0xff;
	}
}

static int w55fa93_udc_probe(struct platform_device *pdev)
{
	struct w55fa93_udc *udc;
	struct device *dev = &pdev->dev;
	int error, i;

	udc = devm_kzalloc(dev, sizeof(*udc), GFP_KERNEL);
	if (!udc)
		return -ENOMEM;

	udc->dev = dev;
	udc->halt_ep = -1;
	udc->status_word = -1;
	spin_lock_init(&udc->lock);

	udc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(udc->base))
		return PTR_ERR(udc->base);

	/* The clock is the APBCLK USBD gate (with the 48 MHz USB engine
	 * clock behind it), provided by the SoC clock driver. */
	udc->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(udc->clk))
		return dev_err_probe(dev, PTR_ERR(udc->clk),
				     "failed to get udc clock\n");

	error = clk_prepare_enable(udc->clk);
	if (error)
		return dev_err_probe(dev, error, "failed to enable udc clock\n");

	platform_set_drvdata(pdev, udc);

	for (i = 0; i < W55FA93_ENDPOINTS; i++) {
		udc->ep[i].index = i;
		udc->ep[i].dev = udc;
	}

	udc_writel(udc, 0x220, USBD_PHY_CTL);

	/* FIXME: is it possible to loop forever? (kept from the BSP) */
	while (1) {
		udc_writel(udc, 0x20, USBD_EP_BASE(1) + EP_MPS);
		if (udc_readl(udc, USBD_EP_BASE(1) + EP_MPS) == 0x20)
			break;
	}
	w55fa93_usb_init(udc);

	/* setup endpoint information */
	udc->gadget.ops = &w55fa93_gadget_ops;
	udc->gadget.max_speed = USB_SPEED_HIGH;
	udc->gadget.speed = USB_SPEED_UNKNOWN;
	udc->gadget.name = "w55fa93-udc";
	INIT_LIST_HEAD(&udc->gadget.ep_list);
	for (i = 0; i < W55FA93_ENDPOINTS; i++) {
		struct w55fa93_ep *ep = &udc->ep[i];

		ep->ep.name = ep_name[i];
		ep->ep.ops = &w55fa93_ep_ops;
		ep->desc = NULL;
		INIT_LIST_HEAD(&ep->queue);

		/* maxpacket differs between ep0 and others */
		if (i == 0) {
			ep->EP_Num = 0;
			ep->ep.maxpacket = EP0_FIFO_SIZE;
			usb_ep_set_maxpacket_limit(&ep->ep, EP0_FIFO_SIZE);
			ep->ep.caps.type_control = true;
			ep->ep.caps.dir_in = true;
			ep->ep.caps.dir_out = true;
			udc_writel(udc, 0x00000000, USBD_CEP_START_ADDR);
			udc_writel(udc, 0x0000003f, USBD_CEP_END_ADDR);
		} else {
			ep->ep.maxpacket = EP_FIFO_SIZE;
			usb_ep_set_maxpacket_limit(&ep->ep, EP_FIFO_SIZE);
			/* EPA..EPF are fully configurable by EP_CFG */
			ep->ep.caps.type_iso = true;
			ep->ep.caps.type_bulk = true;
			ep->ep.caps.type_int = true;
			ep->ep.caps.dir_in = true;
			ep->ep.caps.dir_out = true;
			ep_writel(ep, 0, EP_START_ADDR);
			ep_writel(ep, 0, EP_END_ADDR);
			list_add_tail(&ep->ep.ep_list, &udc->gadget.ep_list);
		}
	}

	udc->gadget.ep0 = &udc->ep[0].ep;

	udc->irq = platform_get_irq(pdev, 0);
	if (udc->irq < 0) {
		error = udc->irq;
		goto err_clk;
	}
	error = devm_request_irq(dev, udc->irq, w55fa93_udc_irq, 0,
				 "w55fa93-udc", udc);
	if (error) {
		dev_err(dev, "request_irq() failed\n");
		goto err_clk;
	}

	error = usb_add_gadget_udc(dev, &udc->gadget);
	if (error) {
		dev_err(dev, "usb_add_gadget_udc() failed\n");
		goto err_clk;
	}

	return 0;

err_clk:
	clk_disable_unprepare(udc->clk);
	return error;
}

static void w55fa93_udc_remove(struct platform_device *pdev)
{
	struct w55fa93_udc *udc = platform_get_drvdata(pdev);

	usb_del_gadget_udc(&udc->gadget);

	udc_writel(udc, 0x20, USBD_PHY_CTL);	/* phy suspend */
	clk_disable_unprepare(udc->clk);
}

static const struct of_device_id w55fa93_udc_of_match[] = {
	{ .compatible = "nuvoton,w55fa93-usbd" },
	{ }
};
MODULE_DEVICE_TABLE(of, w55fa93_udc_of_match);

static struct platform_driver w55fa93_udc_driver = {
	.probe		= w55fa93_udc_probe,
	.remove		= w55fa93_udc_remove,
	.driver		= {
		.name	= "w55fa93-udc",
		.of_match_table = w55fa93_udc_of_match,
	},
};

module_platform_driver(w55fa93_udc_driver);

MODULE_AUTHOR("Nuvoton Technology Corp.");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
