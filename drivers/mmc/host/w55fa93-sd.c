// SPDX-License-Identifier: GPL-2.0
/*
 * Nuvoton W55FA93 SIC (SD Host Interface Controller) MMC/SD host driver.
 *
 * Ported to the modern MMC host API from the vendor BSP driver for
 * Linux 2.6.35 (drivers/mmc/host/w55fa93_sd.c in the W55FA93 BSP).
 * The register semantics follow the BSP driver; where the BSP relied
 * on kernel threads, board Kconfig options or global semaphores, the
 * equivalent modern mechanism (synchronous request execution,
 * completions, DT properties, a per-controller mutex) is used instead.
 * Such places are commented individually.
 *
 * Hardware summary (see arch/arm/mach-w55fa93/include/mach/w55fa93_reg.h
 * in the BSP):
 *   - One SIC block (0xB1006000, IRQ 17) contains a shared DMAC, the
 *     FMI engine and the SD engine.
 *   - The SD engine serves up to three physical ports (SD0/SD1/SD2),
 *     selected one at a time through SDCR.SDPORT.  A board wires two
 *     of them; the BSP exposed SD0 plus either SD1 or SD2 depending on
 *     Kconfig.  This driver registers SD0 and SD1 (SD2 selectable via
 *     the DT property described in the probe function).
 *   - Data is moved by the built-in FMI DMAC from/to a single bounce
 *     buffer; the hardware has no scatter-gather.
 *
 * Copyright (C) 2026
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>

#include <linux/mmc/core.h>
#include <linux/mmc/host.h>
#include <linux/mmc/slot-gpio.h>

#define DRIVER_NAME	"w55fa93-sd"

/* ------------------------------------------------------------------ */
/* Register offsets from the SIC base (BSP: DMAC_BA / FMI_BA)          */
/* ------------------------------------------------------------------ */

/* Shared buffer (FIFO) window: R2 responses are read from here */
#define W55_SD_FB		0x000

/* DMAC (shared by FMI) */
#define W55_DMACCSR		0x400	/* DMAC control and status */
#define   DMAC_FMI_BUSY		BIT(9)
#define   DMAC_SG_EN		BIT(3)
#define   DMAC_SWRST		BIT(1)
#define   DMAC_EN		BIT(0)
#define W55_DMACSAR		0x408	/* DMAC transfer start address */
#define W55_DMACBCR		0x40c	/* DMAC transfer byte count (unused, see below) */
#define W55_DMACIER		0x410
#define W55_DMACISR		0x414
#define   DMAC_WEOT_IF		BIT(1)
#define   DMAC_TABORT_IF	BIT(0)

/* FMI engine */
#define W55_FMICR		0x800	/* FMI control */
#define   FMI_SM_EN		BIT(3)
#define   FMI_SD_EN		BIT(1)
#define   FMI_SWRST		BIT(0)
#define W55_FMIIER		0x804
#define   FMI_DAT_IE		BIT(0)
#define W55_FMIISR		0x808
#define   FMI_DAT_IF		BIT(0)

/* SD engine */
#define W55_SDCR		0x820	/* SD control */
#define   SDCR_CLK_KEEP1	BIT(31)
#define   SDCR_SDPORT_MASK	GENMASK(30, 29)
#define   SDCR_CLK_KEEP2	BIT(28)
#define   SDCR_SDNWR_MASK	GENMASK(27, 24)
#define   SDCR_BLKCNT_MASK	GENMASK(23, 16)
#define   SDCR_DBW		BIT(15)
#define   SDCR_SWRST		BIT(14)
#define   SDCR_CMD_MASK		GENMASK(13, 8)
#define   SDCR_CLK_KEEP		BIT(7)	/* port 0 */
#define   SDCR_8CLK_OE		BIT(6)
#define   SDCR_74CLK_OE		BIT(5)
#define   SDCR_R2_EN		BIT(4)
#define   SDCR_DO_EN		BIT(3)
#define   SDCR_DI_EN		BIT(2)
#define   SDCR_RI_EN		BIT(1)
#define   SDCR_CO_EN		BIT(0)
/* SDCR bits preserved across a command (BSP: csr & 0xff00c080) */
#define   SDCR_PRESERVE		0xff00c080
#define W55_SDARG		0x824	/* command argument */
#define W55_SDIER		0x828	/* interrupt enable */
#define   SDIER_CDSRC		BIT(30)
#define   SDIER_R1B_IEN		BIT(24)
#define   SDIER_WKUP_EN		BIT(14)
#define   SDIER_DITO_IEN	BIT(13)
#define   SDIER_RITO_IEN	BIT(12)
#define   SDIER_SDIO1_IEN	BIT(11)
#define   SDIER_SDIO_IEN	BIT(10)	/* port 0 */
#define   SDIER_CD_IEN		BIT(8)
#define   SDIER_SDIO2_IEN	BIT(2)
#define   SDIER_CRC_IEN		BIT(1)
#define   SDIER_BLKD_IEN	BIT(0)
#define W55_SDISR		0x82c	/* interrupt status (write 1 to clear) */
#define   SDISR_R1B_IF		BIT(24)
#define   SDISR_SDIO2_IF	BIT(22)
#define   SDISR_CD_CARD		BIT(16)
#define   SDISR_DITO_IF		BIT(13)
#define   SDISR_RITO_IF		BIT(12)
#define   SDISR_SDIO1_IF	BIT(11)
#define   SDISR_SDIO_IF		BIT(10)	/* port 0 */
#define   SDISR_CD_IF		BIT(8)
#define   SDISR_SD_DATA0	BIT(7)
#define   SDISR_CRC_MASK	GENMASK(6, 4)
#define   SDISR_CRC_16		BIT(3)
#define   SDISR_CRC_7		BIT(2)
#define   SDISR_CRC_IF		BIT(1)
#define   SDISR_BLKD_IF		BIT(0)
#define W55_SDRSP0		0x830
#define W55_SDRSP1		0x834
#define W55_SDBLEN		0x838	/* block length - 1 */
#define W55_SDTMOUT		0x83c	/* response / data timeout */

/*
 * BSP wrote its Kconfig SD_DISK_MOUNT_DELAY (default 0x1fff, range
 * 0x1fff..0xffffff) into SDTMOUT before every command with a response,
 * noting that a longer value is slower to fail but more stable.  Use
 * the maximum: the value only matters for cards that never respond.
 */
#define W55_RESP_TIMEOUT	0xffffff
#define W55_DATA_TIMEOUT	0xffffff

/* Hardware limits: the SIC transfers at most 255 blocks of <= 512
 * bytes in one command (BSP: MCI_BLKATONCE / MCI_BLKSIZE).  The BSP
 * only printed an error when the core asked for more; enforcing the
 * limits through the mmc_host fields is the modern equivalent.
 */
#define W55_MAX_BLKSIZE		512
#define W55_MAX_BLKCNT		255
#define W55_BUFSIZE		(W55_MAX_BLKSIZE * W55_MAX_BLKCNT)

#define W55_NUM_PORTS		2	/* slots registered per SIC (see probe) */

struct w55fa93_sic;

/* One SD port of the SIC; this is the mmc_host private data. */
struct w55fa93_sd_slot {
	struct mmc_host *mmc;
	struct w55fa93_sic *sic;
	unsigned int port;		/* SDCR.SDPORT value: 0, 1 or 2 */

	u32 bus_width;			/* MMC_BUS_WIDTH_1 / _4 */
	bool sdio_irq_enabled;

	/* Bounce buffer for the FMI DMAC (no scatter-gather in HW). */
	void *bounce;
	dma_addr_t bounce_dma;

	/* Data transfer completion, signalled from the IRQ handler. */
	struct completion xfer_done;
	bool xfer_active;
	int data_error;
	struct mmc_command *cur_cmd;	/* for CRC checks in the IRQ */
};

/* The whole SIC block: register window, clocks and the slots. */
struct w55fa93_sic {
	struct device *dev;
	void __iomem *base;
	int irq;
	struct clk *sic_clk;		/* "sic" gate clock */
	struct clk *sd_clk;		/* "sd" engine clock (rate = SDCLK) */

	/*
	 * Serialises requests across slots.  The BSP used two global
	 * semaphores shared with the NAND (SM) driver (fmi_sem for the
	 * whole command, dmac_sem around DMA buffer use); the SIC can
	 * only run one transfer at a time anyway, so one mutex covering
	 * the whole request is equivalent for SD-only use.
	 */
	struct mutex lock;

	struct w55fa93_sd_slot *slots[3];	/* indexed by port number */
};

static inline u32 w55_read(struct w55fa93_sic *sic, u32 reg)
{
	return readl(sic->base + reg);
}

static inline void w55_write(struct w55fa93_sic *sic, u32 reg, u32 val)
{
	writel(val, sic->base + reg);
}

/* Per-port SDCR clock-keep (free run) bit. */
static u32 w55_clk_keep_bit(unsigned int port)
{
	switch (port) {
	case 0: return SDCR_CLK_KEEP;
	case 1: return SDCR_CLK_KEEP1;
	default: return SDCR_CLK_KEEP2;
	}
}

/* Per-port SDIO interrupt enable / flag bits. */
static u32 w55_sdio_ien_bit(unsigned int port)
{
	switch (port) {
	case 0: return SDIER_SDIO_IEN;
	case 1: return SDIER_SDIO1_IEN;
	default: return SDIER_SDIO2_IEN;
	}
}

static u32 w55_sdio_if_bit(unsigned int port)
{
	switch (port) {
	case 0: return SDISR_SDIO_IF;
	case 1: return SDISR_SDIO1_IF;
	default: return SDISR_SDIO2_IF;
	}
}

/* ------------------------------------------------------------------ */
/* Low level helpers (BSP equivalents in comments)                     */
/* ------------------------------------------------------------------ */

/* BSP: nvt_sd_reset_host() */
static void w55fa93_sd_reset_host(struct w55fa93_sic *sic)
{
	w55_write(sic, W55_DMACCSR,
		  w55_read(sic, W55_DMACCSR) | DMAC_EN | DMAC_SWRST);
	w55_write(sic, W55_FMICR, FMI_SD_EN);
}

/*
 * BSP: nvt_sd_select_port()
 * Select the SD port and reset DMAC/FMI/SD engines so the next access
 * starts from a clean state.  The BSP spun unboundedly on the reset
 * bits; readl_poll_timeout() keeps the same semantics with a bound.
 */
static int w55fa93_sd_select_port(struct w55fa93_sic *sic, unsigned int port)
{
	u32 val;
	int ret;

	w55_write(sic, W55_SDCR,
		  (w55_read(sic, W55_SDCR) & ~SDCR_SDPORT_MASK) |
		  FIELD_PREP(SDCR_SDPORT_MASK, port));

	/* Reset DMAC engine and interrupt status */
	w55_write(sic, W55_DMACCSR,
		  w55_read(sic, W55_DMACCSR) | DMAC_SWRST | DMAC_EN);
	ret = readl_poll_timeout(sic->base + W55_DMACCSR, val,
				 !(val & DMAC_SWRST), 1, 10000);
	if (ret)
		return ret;
	w55_write(sic, W55_DMACCSR, w55_read(sic, W55_DMACCSR) | DMAC_EN);
	w55_write(sic, W55_DMACISR, DMAC_WEOT_IF | DMAC_TABORT_IF);

	/* Reset FMI engine and interrupt status */
	w55_write(sic, W55_FMICR, FMI_SWRST);
	ret = readl_poll_timeout(sic->base + W55_FMICR, val,
				 !(val & FMI_SWRST), 1, 10000);
	if (ret)
		return ret;
	w55_write(sic, W55_FMIISR, FMI_DAT_IF);

	/* Reset SD engine and interrupt status */
	w55_write(sic, W55_FMICR, FMI_SD_EN);
	w55_write(sic, W55_SDCR, w55_read(sic, W55_SDCR) | SDCR_SWRST);
	ret = readl_poll_timeout(sic->base + W55_SDCR, val,
				 !(val & SDCR_SWRST), 1, 10000);
	if (ret)
		return ret;
	w55_write(sic, W55_SDISR, 0xffffffff);

	return 0;
}

/*
 * Clock programming.  The BSP poked the CLKDIV2 dividers of the clock
 * controller directly (nvt_sd_set_clock(), SDCLK = UPLL / div).  Here
 * the dividers belong to the clock provider: this driver only asks
 * for the rate on the "sd" engine clock.  A provider that cannot set
 * rates (fixed clock) is tolerated -- the BSP divider search also
 * silently kept the old rate on impossible requests.
 */
static void w55fa93_sd_set_clock(struct w55fa93_sd_slot *slot,
				 unsigned int clock)
{
	struct w55fa93_sic *sic = slot->sic;
	int ret;

	if (!clock)
		return;

	ret = clk_set_rate(sic->sd_clk, clock);
	if (ret)
		dev_dbg(sic->dev, "clk_set_rate(%u) failed: %d\n", clock, ret);
}

/*
 * BSP: nvt_sd_wait_card_ready()
 * After a write, wait until the card releases DAT0 (busy -> ready),
 * feeding it 8-clock bursts meanwhile.  BSP looped forever; the wait
 * is bounded here and a timeout does not fail the (already written)
 * request.
 */
static void w55fa93_sd_wait_card_ready(struct w55fa93_sd_slot *slot)
{
	struct w55fa93_sic *sic = slot->sic;
	unsigned long deadline = jiffies + msecs_to_jiffies(1000);
	u32 val;
	int ret;

	while (!(w55_read(sic, W55_SDISR) & SDISR_SD_DATA0)) {
		if (time_after(jiffies, deadline)) {
			dev_dbg(sic->dev, "card still busy after write\n");
			break;
		}
		/* Generate 8 clocks */
		w55_write(sic, W55_SDCR,
			  w55_read(sic, W55_SDCR) | SDCR_8CLK_OE);
		ret = readl_poll_timeout(sic->base + W55_SDCR, val,
					 !(val & SDCR_8CLK_OE), 1, 1000);
		if (ret)
			break;
		usleep_range(50, 100);
	}
}

/* ------------------------------------------------------------------ */
/* Command execution                                                   */
/* ------------------------------------------------------------------ */

/*
 * Wait for the command/response phase to finish.  The hardware has
 * no command-complete interrupt: the BSP polled SDCR from per-port
 * kernel threads until CO_EN (and RI_EN/R2_EN) cleared by hardware.
 * .request() runs in sleepable context, so poll here directly.
 */
static int w55fa93_sd_poll_cmd(struct w55fa93_sd_slot *slot, u32 expect)
{
	struct w55fa93_sic *sic = slot->sic;
	unsigned long timeout = jiffies + msecs_to_jiffies(1000);

	while (w55_read(sic, W55_SDCR) & expect) {
		if (expect & SDCR_RI_EN) {
			if (w55_read(sic, W55_SDISR) & SDISR_RITO_IF) {
				w55_write(sic, W55_SDTMOUT, 0);
				w55_write(sic, W55_SDISR, SDISR_RITO_IF);
				return -ETIMEDOUT;
			}
		}
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
		usleep_range(20, 100);
	}
	return 0;
}

/* Collect the response, BSP: nvt_sd_completed_command() */
static int w55fa93_sd_collect_resp(struct w55fa93_sd_slot *slot,
				   struct mmc_command *cmd, u32 expect)
{
	struct w55fa93_sic *sic = slot->sic;
	u32 status;

	if (mmc_resp_type(cmd) == MMC_RSP_NONE)
		return 0;

	if (w55fa93_sd_poll_cmd(slot, expect)) {
		cmd->resp[0] = cmd->resp[1] = cmd->resp[2] = cmd->resp[3] = 0;
		return -ETIMEDOUT;
	}

	/* SDISR is only meaningful once the response has arrived */
	status = w55_read(sic, W55_SDISR);

	if (mmc_resp_type(cmd) == MMC_RSP_R2) {
		/*
		 * R2 (136 bit) responses land in the shared buffer
		 * window at the start of the SIC address space; the BSP
		 * reassembled them byte-wise from REG_FB_0.
		 * TODO(api-check): the BSP dereferenced a plain memory
		 * pointer here.  Verify on hardware that the FIFO window
		 * tolerates readb() accesses (vs. requiring 32-bit
		 * readl()); the byte assembly itself is BSP-faithful.
		 */
		u8 buf[20];
		u32 tmp[5];
		int i, j;

		for (i = 0; i < 20; i++)
			buf[i] = readb(sic->base + W55_SD_FB + i);
		for (i = 0, j = 0; j < 5; i += 4, j++)
			tmp[j] = (buf[i] << 24) | (buf[i + 1] << 16) |
				 (buf[i + 2] << 8) | buf[i + 3];
		for (i = 0; i < 4; i++)
			cmd->resp[i] = ((tmp[i] & 0x00ffffff) << 8) |
				       ((tmp[i + 1] & 0xff000000) >> 24);
	} else {
		cmd->resp[0] = (w55_read(sic, W55_SDRSP0) << 8) |
			       (w55_read(sic, W55_SDRSP1) & 0xff);
		cmd->resp[1] = cmd->resp[2] = cmd->resp[3] = 0;
	}

	/*
	 * CRC-7 check, BSP semantics: SDISR_CRC_7 set means the check
	 * passed.  Responses without a CRC (e.g. R3) must not be failed.
	 */
	if (!(status & SDISR_CRC_7) &&
	    (mmc_resp_type(cmd) & MMC_RSP_CRC)) {
		u32 val;

		w55_write(sic, W55_SDISR, SDISR_CRC_IF);
		/* CRC error: reset the SD engine (BSP) */
		w55_write(sic, W55_SDCR,
			  w55_read(sic, W55_SDCR) | SDCR_SWRST);
		readl_poll_timeout(sic->base + W55_SDCR, val,
				   !(val & SDCR_SWRST), 1, 10000);
		return -EIO;
	}
	w55_write(sic, W55_SDISR, SDISR_CRC_IF);

	return 0;
}

/*
 * Execute one command, including its data phase if any.
 * BSP: nvt_sd_send_command() + the transfer wait in
 * nvt_sd_completed_command().
 *
 * Must be called with sic->lock held.
 */
static int w55fa93_sd_exec(struct w55fa93_sd_slot *slot,
			   struct mmc_command *cmd)
{
	struct w55fa93_sic *sic = slot->sic;
	struct mmc_data *data = cmd->data;
	u32 csr, expect = SDCR_CO_EN;
	unsigned int blocks = 0;
	bool keep_clock;
	int ret;

	slot->cur_cmd = cmd;

	if (w55_read(sic, W55_FMICR) != FMI_SD_EN)
		w55_write(sic, W55_FMICR, FMI_SD_EN);

	ret = w55fa93_sd_select_port(sic, slot->port);
	if (ret)
		return ret;

	/* Prepare SDCR: keep port select/divider bits, clear the rest */
	csr = w55_read(sic, W55_SDCR) & SDCR_PRESERVE;

	/*
	 * The BSP turned SD clock free-run off for the duration of a
	 * command (to support SDIO interrupt mode) and restored it
	 * afterwards.  Do the same: remember the state, clear it for
	 * the command, restore it on the way out (see "out" below).
	 */
	keep_clock = !!(csr & w55_clk_keep_bit(slot->port));
	csr &= ~w55_clk_keep_bit(slot->port);

	csr |= FIELD_PREP(SDCR_CMD_MASK, cmd->opcode) | SDCR_CO_EN;

	if (slot->bus_width == MMC_BUS_WIDTH_4)
		csr |= SDCR_DBW;

	if (mmc_resp_type(cmd) != MMC_RSP_NONE) {
		if (mmc_resp_type(cmd) == MMC_RSP_R2) {
			csr |= SDCR_R2_EN;
			expect |= SDCR_R2_EN;
		} else {
			csr |= SDCR_RI_EN;
			expect |= SDCR_RI_EN;
		}
		w55_write(sic, W55_SDISR, SDISR_RITO_IF);
		w55_write(sic, W55_SDTMOUT, W55_RESP_TIMEOUT);
	}

	if (data) {
		unsigned int len = data->blksz * data->blocks;

		blocks = data->blocks;
		if (data->blksz > W55_MAX_BLKSIZE || blocks > W55_MAX_BLKCNT)
			return -EINVAL;	/* cannot happen: host limits */

		w55_write(sic, W55_SDIER,
			  w55_read(sic, W55_SDIER) | SDIER_BLKD_IEN);
		w55_write(sic, W55_SDBLEN, data->blksz - 1);
		csr = (csr & ~SDCR_BLKCNT_MASK) |
		      FIELD_PREP(SDCR_BLKCNT_MASK, blocks);

		data->bytes_xfered = 0;
		slot->data_error = 0;
		reinit_completion(&slot->xfer_done);
		slot->xfer_active = true;

		/* The DMAC only needs the start address; the SD engine
		 * derives the byte count from SDBLEN/BLKCNT (BSP never
		 * programmed DMACBCR either).
		 */
		w55_write(sic, W55_DMACSAR, slot->bounce_dma);

		if (data->flags & MMC_DATA_WRITE) {
			sg_copy_to_buffer(data->sg, data->sg_len,
					  slot->bounce, len);
			csr |= SDCR_DO_EN;
		}
	} else {
		w55_write(sic, W55_SDIER,
			  w55_read(sic, W55_SDIER) & ~SDIER_BLKD_IEN);
	}

	w55fa93_sd_set_clock(slot, slot->mmc->ios.clock);

	w55_write(sic, W55_SDARG, cmd->arg);
	w55_write(sic, W55_SDCR, csr);		/* go */

	ret = w55fa93_sd_collect_resp(slot, cmd, expect);
	if (ret) {
		slot->xfer_active = false;
		goto out;
	}

	if (!data) {
		ret = 0;
		goto out;
	}

	/* Data phase */
	if (data->flags & MMC_DATA_READ) {
		w55_write(sic, W55_SDTMOUT, W55_DATA_TIMEOUT);
		w55_write(sic, W55_SDCR,
			  w55_read(sic, W55_SDCR) | SDCR_DI_EN);
	}

	{
		unsigned long timeout;

		if (data->timeout_ns)
			timeout = usecs_to_jiffies(
					(unsigned int)(data->timeout_ns / 1000));
		else
			timeout = msecs_to_jiffies(5000);
		/* Add slack like other host drivers do */
		timeout += msecs_to_jiffies(500);

		if (!wait_for_completion_timeout(&slot->xfer_done, timeout)) {
			slot->xfer_active = false;
			w55fa93_sd_reset_host(sic);
			ret = -ETIMEDOUT;
			goto out;
		}
	}
	slot->xfer_active = false;

	if (slot->data_error) {
		ret = slot->data_error;
		goto out;
	}

	if (data->flags & MMC_DATA_READ) {
		sg_copy_from_buffer(data->sg, data->sg_len, slot->bounce,
				    data->blksz * data->blocks);
		data->bytes_xfered = data->blksz * data->blocks;
	} else {
		data->bytes_xfered = data->blksz * data->blocks;
		w55fa93_sd_wait_card_ready(slot);
	}
	ret = 0;

out:
	/* Restore SD clock free-run if it was on (SDIO interrupt mode) */
	if (keep_clock)
		w55_write(sic, W55_SDCR, w55_read(sic, W55_SDCR) |
			  w55_clk_keep_bit(slot->port));
	return ret;
}

/* ------------------------------------------------------------------ */
/* mmc_host_ops (BSP: nvt_sd_ops)                                      */
/* ------------------------------------------------------------------ */

static void w55fa93_sd_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct w55fa93_sd_slot *slot = mmc_priv(mmc);
	struct w55fa93_sic *sic = slot->sic;
	int ret;

	mutex_lock(&sic->lock);

	/*
	 * BSP checked card presence up front for the internally
	 * detected port; with a cd GPIO the core does this for us.
	 */
	if (slot->port == 0 && mmc_gpio_get_cd(mmc) < 0 &&
	    !(mmc->caps & MMC_CAP_NONREMOVABLE)) {
		if (w55_read(sic, W55_SDISR) & SDISR_CD_CARD) {
			mrq->cmd->error = -ENOMEDIUM;
			goto out;
		}
	}

	ret = w55fa93_sd_exec(slot, mrq->cmd);
	mrq->cmd->error = ret;
	if (mrq->cmd->data && ret)
		mrq->cmd->data->error = ret;

	/* Send the stop command, as the BSP did via nvt_sd_send_stop() */
	if (mrq->stop && mrq->data) {
		int stop_ret = w55fa93_sd_exec(slot, mrq->stop);

		mrq->stop->error = stop_ret;
	}

out:
	mutex_unlock(&sic->lock);
	mmc_request_done(mmc, mrq);
}

static void w55fa93_sd_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct w55fa93_sd_slot *slot = mmc_priv(mmc);
	struct w55fa93_sic *sic = slot->sic;
	u32 val;
	int ret;

	slot->bus_width = ios->bus_width;

	mutex_lock(&sic->lock);

	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		/*
		 * The FMI engine is shared by all ports, so power is
		 * effectively controller-wide (the BSP did the same).
		 */
		w55_write(sic, W55_FMICR,
			  w55_read(sic, W55_FMICR) & ~FMI_SD_EN);
		break;
	case MMC_POWER_UP:
	case MMC_POWER_ON:
		w55_write(sic, W55_FMICR, FMI_SD_EN);
		w55fa93_sd_set_clock(slot, ios->clock ? ios->clock : 300000);

		/* Emit the 74 clocks required before the first command */
		w55_write(sic, W55_SDCR,
			  w55_read(sic, W55_SDCR) | SDCR_74CLK_OE);
		ret = readl_poll_timeout(sic->base + W55_SDCR, val,
					 !(val & SDCR_74CLK_OE), 1, 10000);
		if (ret)
			dev_dbg(sic->dev, "74-clock output timed out\n");
		break;
	default:
		break;
	}

	if (ios->bus_width == MMC_BUS_WIDTH_4)
		w55_write(sic, W55_SDCR, w55_read(sic, W55_SDCR) | SDCR_DBW);
	else
		w55_write(sic, W55_SDCR, w55_read(sic, W55_SDCR) & ~SDCR_DBW);

	mutex_unlock(&sic->lock);
}

/*
 * BSP: nvt_sd_card_detect() / nvt_sd_get_cd()
 * A cd GPIO (from DT) wins; port 0 falls back to the SIC's own CD pin
 * status (SDISR.CD_CARD set means "removed" in the BSP wiring, GPA1
 * pulled high); other ports without a GPIO report "present", which
 * was also the BSP default when no CD pin was configured.
 */
static int w55fa93_sd_get_cd(struct mmc_host *mmc)
{
	struct w55fa93_sd_slot *slot = mmc_priv(mmc);
	int ret = mmc_gpio_get_cd(mmc);

	if (ret >= 0)
		return ret;

	if (slot->port == 0)
		return (w55_read(slot->sic, W55_SDISR) & SDISR_CD_CARD) ?
			0 : 1;

	return 1;
}

/* BSP: nvt_sd_get_ro() -- GPIO if DT provides one, else writable */
static int w55fa93_sd_get_ro(struct mmc_host *mmc)
{
	int ret = mmc_gpio_get_ro(mmc);

	if (ret >= 0)
		return ret;
	return 0;
}

/* BSP: nvt_sd_enable_sdio_irq() */
static void w55fa93_sd_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct w55fa93_sd_slot *slot = mmc_priv(mmc);
	struct w55fa93_sic *sic = slot->sic;
	u32 ier;

	slot->sdio_irq_enabled = !!enable;

	ier = w55_read(sic, W55_SDIER);
	ier &= ~w55_sdio_ien_bit(slot->port);
	if (slot->port == 2)
		ier &= ~SDIER_SDIO1_IEN;	/* BSP quirk, see below */
	w55_write(sic, W55_SDIER, ier);
	w55_write(sic, W55_SDISR, w55_sdio_if_bit(slot->port));

	if (enable) {
		w55_write(sic, W55_SDIER,
			  w55_read(sic, W55_SDIER) | SDIER_WKUP_EN);
		/*
		 * BSP: for port 2 both SDIO1_IEN and SDIO2_IEN must be
		 * set for the SDIO2 interrupt to work.
		 */
		if (slot->port == 2)
			w55_write(sic, W55_SDIER,
				  w55_read(sic, W55_SDIER) | SDIER_SDIO1_IEN);
		w55_write(sic, W55_SDIER,
			  w55_read(sic, W55_SDIER) |
			  w55_sdio_ien_bit(slot->port));
		/* Clock must free-run to latch the SDIO interrupt */
		w55_write(sic, W55_SDCR, w55_read(sic, W55_SDCR) |
			  w55_clk_keep_bit(slot->port));
	} else {
		w55_write(sic, W55_SDIER,
			  w55_read(sic, W55_SDIER) & ~SDIER_WKUP_EN);
		w55_write(sic, W55_SDCR, w55_read(sic, W55_SDCR) &
			  ~w55_clk_keep_bit(slot->port));
	}
}

static const struct mmc_host_ops w55fa93_sd_ops = {
	.request	= w55fa93_sd_request,
	.set_ios	= w55fa93_sd_set_ios,
	.get_ro		= w55fa93_sd_get_ro,
	.get_cd		= w55fa93_sd_get_cd,
	.enable_sdio_irq = w55fa93_sd_enable_sdio_irq,
};

/* ------------------------------------------------------------------ */
/* Interrupt handler (BSP: nvt_sd_irq(), shared by all ports)          */
/* ------------------------------------------------------------------ */

static irqreturn_t w55fa93_sd_irq(int irq, void *devid)
{
	struct w55fa93_sic *sic = devid;
	struct w55fa93_sd_slot *slot;
	u32 status = w55_read(sic, W55_SDISR);
	irqreturn_t ret = IRQ_NONE;
	unsigned int i;

	/* SDIO interrupts are flagged per port */
	for (i = 0; i < 3; i++) {
		slot = sic->slots[i];
		if (!slot)
			continue;
		if (status & w55_sdio_if_bit(slot->port)) {
			/* Disable before clearing, as the BSP did */
			w55_write(sic, W55_SDIER, w55_read(sic, W55_SDIER) &
				  ~w55_sdio_ien_bit(slot->port));
			w55_write(sic, W55_SDISR, w55_sdio_if_bit(slot->port));
			mmc_signal_sdio_irq(slot->mmc);
			ret = IRQ_HANDLED;
		}
	}

	/* Card detect on the SIC CD pin (port 0, BSP default wiring) */
	if (status & SDISR_CD_IF) {
		w55_write(sic, W55_SDISR, SDISR_CD_IF);
		if (sic->slots[0])
			mmc_detect_change(sic->slots[0]->mmc,
					  msecs_to_jiffies(500));
		ret = IRQ_HANDLED;
	}

	/* Block transfer done */
	if (status & SDISR_BLKD_IF) {
		unsigned int port = FIELD_GET(SDCR_SDPORT_MASK,
					      w55_read(sic, W55_SDCR));

		slot = (port < 3) ? sic->slots[port] : NULL;
		if (slot && slot->xfer_active) {
			struct mmc_command *cmd = slot->cur_cmd;

			/*
			 * CRC-16 check for data-in, BSP semantics.
			 * ACMD51 (SCR, 8 bytes) and CMD6 (SSR, 64 bytes)
			 * can raise false CRC-16 errors and are exempt.
			 */
			if (cmd && cmd->data &&
			    (cmd->data->flags & MMC_DATA_READ) &&
			    (status & SDISR_CRC_IF) &&
			    !(status & SDISR_CRC_16) &&
			    cmd->opcode != 51 && cmd->opcode != 6) {
				u32 val;

				slot->data_error = -EIO;
				w55_write(sic, W55_SDISR, SDISR_CRC_IF);
				w55_write(sic, W55_SDCR,
					  w55_read(sic, W55_SDCR) |
					  SDCR_SWRST);
				readl_poll_timeout_atomic(
					sic->base + W55_SDCR, val,
					!(val & SDCR_SWRST), 1, 1000);
			}
			w55_write(sic, W55_SDISR, SDISR_BLKD_IF);
			complete(&slot->xfer_done);
			ret = IRQ_HANDLED;
		} else if (slot) {
			/* Stale flag for an idle slot: clear and move on */
			w55_write(sic, W55_SDISR, SDISR_BLKD_IF);
			ret = IRQ_HANDLED;
		}
	}

	return ret;
}

/* ------------------------------------------------------------------ */
/* Probe / remove                                                      */
/* ------------------------------------------------------------------ */

/*
 * BSP: nvt_sd_enable()
 * The GPIO multi-function pin setup the BSP did here (GPEFUN/GPAFUN
 * writes) is the pinctrl driver's job in a DT system and is not
 * replicated; only the SIC-internal setup remains.
 */
static void w55fa93_sd_hw_init(struct w55fa93_sic *sic)
{
	/* Enable DMAC for FMI and the SD function */
	w55_write(sic, W55_DMACCSR, w55_read(sic, W55_DMACCSR) | DMAC_EN);
	w55_write(sic, W55_FMICR, FMI_SD_EN);
	w55_write(sic, W55_SDISR, 0xffffffff);

	/* Port 0 card detect comes from the SIC CD pin (BSP: CDSRC) */
	w55_write(sic, W55_SDIER, w55_read(sic, W55_SDIER) | SDIER_CDSRC);

	w55fa93_sd_select_port(sic, 0);

	/* SDNWR = 9 + 1 clocks (BSP) */
	w55_write(sic, W55_SDCR,
		  (w55_read(sic, W55_SDCR) & ~SDCR_SDNWR_MASK) |
		  FIELD_PREP(SDCR_SDNWR_MASK, 9));
	/* SDCR_BLKCNT = 1 */
	w55_write(sic, W55_SDCR,
		  (w55_read(sic, W55_SDCR) & ~SDCR_BLKCNT_MASK) |
		  FIELD_PREP(SDCR_BLKCNT_MASK, 1));

	/* Card detect interrupt (the BSP enabled it after probing) */
	w55_write(sic, W55_SDIER, w55_read(sic, W55_SDIER) | SDIER_CD_IEN);
}

static int w55fa93_sd_add_slot(struct w55fa93_sic *sic, unsigned int port)
{
	struct device *dev = sic->dev;
	struct mmc_host *mmc;
	struct w55fa93_sd_slot *slot;
	int ret;

	mmc = devm_mmc_alloc_host(dev, sizeof(*slot));
	if (!mmc)
		return -ENOMEM;

	mmc->ops = &w55fa93_sd_ops;
	mmc->f_min = 300000;
	mmc->f_max = 24000000;	/* BSP default (a 48 MHz variant existed) */
	mmc->ocr_avail = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30 |
			 MMC_VDD_30_31 | MMC_VDD_31_32 | MMC_VDD_32_33 |
			 MMC_VDD_33_34;
	mmc->caps = MMC_CAP_4_BIT_DATA | MMC_CAP_SD_HIGHSPEED |
		    MMC_CAP_SDIO_IRQ;
	mmc->max_blk_size = W55_MAX_BLKSIZE;
	mmc->max_blk_count = W55_MAX_BLKCNT;
	mmc->max_req_size = W55_BUFSIZE;
	mmc->max_segs = W55_MAX_BLKCNT;
	mmc->max_seg_size = W55_BUFSIZE;

	slot = mmc_priv(mmc);
	slot->mmc = mmc;
	slot->sic = sic;
	slot->port = port;
	slot->bus_width = MMC_BUS_WIDTH_1;
	init_completion(&slot->xfer_done);

	slot->bounce = dmam_alloc_coherent(dev, W55_BUFSIZE,
					   &slot->bounce_dma, GFP_KERNEL);
	if (!slot->bounce)
		return -ENOMEM;

	sic->slots[port] = slot;

	ret = mmc_of_parse(mmc);
	if (ret)
		goto fail;

	/*
	 * Optional cd/wp GPIOs from DT.  -ENOENT simply means the
	 * board does not wire them; port 0 then uses the SIC CD pin
	 * and other ports report "always present" (both BSP defaults).
	 */
	ret = mmc_gpiod_request_cd(mmc, "cd", 0, false, 0);
	if (ret && ret != -ENOENT)
		goto fail;
	if (!ret)
		mmc_gpiod_request_cd_irq(mmc);

	ret = mmc_gpiod_request_ro(mmc, "wp", 0, 0);
	if (ret && ret != -ENOENT)
		goto fail;

	ret = mmc_add_host(mmc);
	if (ret)
		goto fail;

	dev_info(dev, "SD port %u registered (%s)\n", port, mmc_hostname(mmc));
	return 0;

fail:
	sic->slots[port] = NULL;
	return ret;
}

/* Boot-milestone breadcrumbs (see clk-w55fa93.c). Diagnostic only. */
extern void ja720_paint(u32 color);

static int __init ja720_crumb_late(void)
{
	ja720_paint(0xffff00ff);	/* MAGENTA: all initcalls finished */
	return 0;
}
late_initcall_sync(ja720_crumb_late);

static int w55fa93_sd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	ja720_paint(0xff00ffff);	/* CYAN: MMC host probing */
	struct w55fa93_sic *sic;
	unsigned int port2 = 0;
	int ret;

	sic = devm_kzalloc(dev, sizeof(*sic), GFP_KERNEL);
	if (!sic)
		return -ENOMEM;
	sic->dev = dev;
	mutex_init(&sic->lock);

	sic->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(sic->base))
		return PTR_ERR(sic->base);

	sic->irq = platform_get_irq(pdev, 0);
	if (sic->irq < 0)
		return sic->irq;

	/*
	 * Clocks, as agreed for this platform: "sic" is the gate for
	 * the SIC block, "sd" is the engine clock whose rate is the
	 * SDCLK requested through clk_set_rate() in set_ios.
	 * TODO(api-check): clock names/rates are defined by the JA720
	 * clock driver + DT; if "sd" turns out to be a gate-only clock,
	 * clk_set_rate() fails and is merely logged (see set_clock).
	 */
	sic->sic_clk = devm_clk_get(dev, "sic");
	if (IS_ERR(sic->sic_clk))
		return dev_err_probe(dev, PTR_ERR(sic->sic_clk),
				     "failed to get sic clock\n");
	sic->sd_clk = devm_clk_get(dev, "sd");
	if (IS_ERR(sic->sd_clk))
		return dev_err_probe(dev, PTR_ERR(sic->sd_clk),
				     "failed to get sd clock\n");

	ret = clk_prepare_enable(sic->sic_clk);
	if (ret)
		return ret;
	ret = clk_prepare_enable(sic->sd_clk);
	if (ret)
		goto err_clk_sic;

	ret = devm_request_irq(dev, sic->irq, w55fa93_sd_irq, IRQF_SHARED,
			       DRIVER_NAME, sic);
	if (ret)
		goto err_clk_sd;

	platform_set_drvdata(pdev, sic);

	w55fa93_sd_hw_init(sic);

	/* SD0 is always present (it is the boot port on JA720). */
	ret = w55fa93_sd_add_slot(sic, 0);
	if (ret)
		goto err_clk_sd;

	/*
	 * Second slot: the BSP compiled in either SD1
	 * (CONFIG_FA93_SD_SD0_SD1) or SD2 (CONFIG_FA93_SD_SD0_SD2) as
	 * the second port, depending on board wiring.  In DT terms:
	 * omit the property for SD1 (default), set
	 * "nuvoton,second-port" to 2 for SD2 wiring, or set
	 * "nuvoton,no-second-port" for single-slot boards.
	 * TODO(api-check): these two property names are invented here
	 * and must be fixed in the DT binding the parent writes; the
	 * JA720 only needs the default (SD0 + SD1).
	 */
	if (!of_property_read_bool(dev->of_node, "nuvoton,no-second-port")) {
		of_property_read_u32(dev->of_node, "nuvoton,second-port",
				     &port2);
		ret = w55fa93_sd_add_slot(sic, port2 ? 2 : 1);
		if (ret)
			dev_warn(dev, "second SD port disabled: %d\n", ret);
	}

	return 0;

err_clk_sd:
	clk_disable_unprepare(sic->sd_clk);
err_clk_sic:
	clk_disable_unprepare(sic->sic_clk);
	return ret;
}

static void w55fa93_sd_remove(struct platform_device *pdev)
{
	struct w55fa93_sic *sic = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < 3; i++) {
		if (sic->slots[i]) {
			mmc_remove_host(sic->slots[i]->mmc);
			sic->slots[i] = NULL;
		}
	}

	/* BSP: nvt_sd_disable() */
	w55_write(sic, W55_SDISR, 0xffffffff);
	w55_write(sic, W55_FMICR,
		  w55_read(sic, W55_FMICR) & ~FMI_SD_EN);

	clk_disable_unprepare(sic->sd_clk);
	clk_disable_unprepare(sic->sic_clk);
}

static const struct of_device_id w55fa93_sd_of_match[] = {
	{ .compatible = "nuvoton,w55fa93-mmc" },
	{ }
};
MODULE_DEVICE_TABLE(of, w55fa93_sd_of_match);

static struct platform_driver w55fa93_sd_driver = {
	.probe = w55fa93_sd_probe,
	.remove = w55fa93_sd_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = w55fa93_sd_of_match,
	},
};
module_platform_driver(w55fa93_sd_driver);

MODULE_DESCRIPTION("Nuvoton W55FA93 SIC SD/MMC host driver");
MODULE_LICENSE("GPL-2.0");
