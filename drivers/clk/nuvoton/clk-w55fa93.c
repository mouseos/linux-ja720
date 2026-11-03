// SPDX-License-Identifier: GPL-2.0-only
/*
 * Clock provider for the Nuvoton W55FA93 (N32905) SoC.
 *
 * Bring-up oriented provider:
 *  - APLL/UPLL rates are decoded from the PLLCON registers (FB_DV [8:0],
 *    IN_DV [13:9], OUT_DV [15:14]; Fout = Fin * (FB+2) / ((IN+2) * OD),
 *    OD = 1/2/2/4 for OUT_DV = 0/1/2/3, per the vendor BSP's NOMap).
 *  - AHBCLK/APBCLK gates are registered with CLK_IGNORE_UNUSED, and the
 *    clocks the port depends on (VPOST, SIC, SD engine, USBD, USBH,
 *    timers) are explicitly switched on at probe time, because the
 *    state left by the boot loader is not guaranteed for APB gates.
 *  - The timer gate parents are the external crystal clock, matching
 *    the BSP assumption CLOCK_TICK_RATE == EXTCLK rate.
 *  - The SD engine clock is UPLL divided by the CLKDIV2 two-stage
 *    divider (SD_S = UPLL, SDCLK = UPLL / ((SD_N0+1) * (SD_N1+1))),
 *    programmed exactly like the BSP's nvt_sd_set_clock(). Until this
 *    was implemented, "sd-src" was a fictitious fixed 48 MHz clock and
 *    clk_set_rate() from the MMC driver changed no hardware, leaving
 *    the SD engine at whatever clock the boot loader happened to set.
 *
 * Copyright (C) 2026 JA720 Linux port contributors
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>

#include <dt-bindings/clock/nuvoton-w55fa93.h>

#define AHBCLK		0x204
#define APBCLK		0x208
#define CLKDIV2		0x214
#define APLLCON		0x220
#define UPLLCON		0x224

/* CLKDIV2 SD engine clock fields (vendor BSP w55fa93_reg.h) */
#define SD_N0_MASK	(0x7 << 16)	/* pre-divider SD_N0, stored value = div - 1 */
#define SD_S_MASK	(0x3 << 19)	/* source select: 0=xin 1=x32k 2=apll 3=upll */
#define SD_S_UPLL	(0x3 << 19)
#define SD_N1_MASK	(0xff << 24)	/* divider SD_N1, stored value = div - 1 */

#define PLL_PD		BIT(16)
#define PLL_OUT_DV(x)	(((x) >> 14) & 0x3)
#define PLL_IN_DV(x)	(((x) >> 9) & 0x1f)
#define PLL_FB_DV(x)	((x) & 0x1ff)

/* Output divider indexed by OUT_DV: the BSP's NOMap is {1, 2, 2, 4}
 * (arch/arm/mach-w55fa93/w55fa93.c, w55fa93_get_pll), not the more
 * common {1, 2, 4, 8} progression. Getting this wrong halves the
 * decoded PLL rate for OUT_DV 2 and 3, which in turn makes the SD
 * divider computation run the card at twice the requested clock. */
static const unsigned int pll_outdiv[] = { 1, 2, 2, 4 };

static unsigned long w55fa93_pll_rate(void __iomem *gcr, u32 off,
				      unsigned long fin)
{
	u32 con = readl(gcr + off);

	if (con & PLL_PD)
		return 0;
	return fin * (PLL_FB_DV(con) + 2) /
	       ((PLL_IN_DV(con) + 2) * pll_outdiv[PLL_OUT_DV(con)]);
}

struct w55fa93_clk_data {
	struct clk_hw_onecell_data *hw_data;
};

static DEFINE_SPINLOCK(w55fa93_clk_lock);

/*
 * SD engine clock: UPLL through the CLKDIV2 two-stage divider.
 * Divider search mirrors the vendor BSP (nvt_sd_set_clock): the total
 * division is rounded up so the resulting clock never exceeds the
 * requested rate, div0 (1..8) is the largest factor of the total that
 * fits, and div1 (1..256) takes the remainder.
 */
struct w55fa93_sdclk {
	struct clk_hw hw;
	void __iomem *reg;	/* CLKDIV2 */
	spinlock_t *lock;
};

#define to_w55fa93_sdclk(_hw) container_of(_hw, struct w55fa93_sdclk, hw)

static void w55fa93_sdclk_choose(unsigned long parent, unsigned long rate,
				 unsigned int *div0, unsigned int *div1)
{
	unsigned long total;
	unsigned int d0;

	if (!rate)
		rate = 1;
	total = DIV_ROUND_UP(parent, rate);
	if (total > 8UL * 256)
		total = 8UL * 256;
	if (!total)
		total = 1;
	for (d0 = 8; d0 > 0; d0--) {
		if (total % d0 == 0)
			break;
	}
	if (d0 == 0)
		d0 = (total <= 256) ? 1 : 8;
	*div0 = d0;
	*div1 = (unsigned int)(total / d0);
}

static unsigned long w55fa93_sdclk_recalc_rate(struct clk_hw *hw,
					       unsigned long parent_rate)
{
	struct w55fa93_sdclk *sdclk = to_w55fa93_sdclk(hw);
	u32 val = readl(sdclk->reg);
	unsigned int div0 = ((val & SD_N0_MASK) >> 16) + 1;
	unsigned int div1 = ((val & SD_N1_MASK) >> 24) + 1;

	return parent_rate / (div0 * div1);
}

static int w55fa93_sdclk_determine_rate(struct clk_hw *hw,
					struct clk_rate_request *req)
{
	unsigned int div0, div1;

	w55fa93_sdclk_choose(req->best_parent_rate, req->rate, &div0, &div1);
	req->rate = req->best_parent_rate / (div0 * div1);
	return 0;
}

static int w55fa93_sdclk_set_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long parent_rate)
{
	struct w55fa93_sdclk *sdclk = to_w55fa93_sdclk(hw);
	unsigned int div0, div1;
	unsigned long flags;
	u32 val;

	w55fa93_sdclk_choose(parent_rate, rate, &div0, &div1);

	spin_lock_irqsave(sdclk->lock, flags);
	val = readl(sdclk->reg);
	val &= ~(SD_N0_MASK | SD_N1_MASK | SD_S_MASK);
	val |= ((div0 - 1) << 16) | ((div1 - 1) << 24) | SD_S_UPLL;
	writel(val, sdclk->reg);
	spin_unlock_irqrestore(sdclk->lock, flags);
	return 0;
}

static const struct clk_ops w55fa93_sdclk_ops = {
	.recalc_rate = w55fa93_sdclk_recalc_rate,
	.determine_rate = w55fa93_sdclk_determine_rate,
	.set_rate = w55fa93_sdclk_set_rate,
};

/*
 * Boot-milestone breadcrumb for the JA720 hardware bring-up: fill the
 * framebuffer the boot loader set up (0x81c00000, 320x240 a8r8g8b8)
 * with a solid color. Diagnostic only; each boot stage paints its own
 * color so the last visible color names the stage that was reached.
 */
void ja720_paint(u32 color)
{
	void __iomem *fb;
	int i;

	fb = ioremap(0x81c00000, 320 * 240 * 4);
	if (!fb)
		return;
	for (i = 0; i < 320 * 240; i++)
		writel(color, fb + i * 4);
	iounmap(fb);
}
EXPORT_SYMBOL_GPL(ja720_paint);

static void __init w55fa93_clk_init(struct device_node *np)
{
	struct clk_hw_onecell_data *hw_data;
	struct clk_hw *hw, *sd_src_hw;
	void __iomem *gcr;
	struct clk *xin_clk;
	unsigned long xin_rate, apll_rate, upll_rate;
	u32 val;
	int i, ret;

	gcr = of_iomap(np, 0);
	if (!gcr) {
		pr_err("w55fa93-clk: failed to map GCR registers\n");
		return;
	}

	xin_clk = of_clk_get(np, 0);
	if (IS_ERR(xin_clk)) {
		pr_err("w55fa93-clk: failed to get xin clock\n");
		return;
	}
	xin_rate = clk_get_rate(xin_clk);
	clk_put(xin_clk);

	apll_rate = w55fa93_pll_rate(gcr, APLLCON, xin_rate);
	upll_rate = w55fa93_pll_rate(gcr, UPLLCON, xin_rate);
	pr_info("w55fa93-clk: xin %lu Hz, apll %lu Hz, upll %lu Hz\n",
		xin_rate, apll_rate, upll_rate);

	/*
	 * Make sure the clocks this port relies on are running, no matter
	 * what state the boot loader left behind.
	 * AHBCLK: VPOST(27) SIC(21) SD(23) USBD(18) USBH(17)
	 * APBCLK: TMR0(8) TMR1(9)
	 */
	val = readl(gcr + AHBCLK);
	val |= BIT(27) | BIT(21) | BIT(23) | BIT(18) | BIT(17);
	writel(val, gcr + AHBCLK);
	val = readl(gcr + APBCLK);
	val |= BIT(8) | BIT(9);
	writel(val, gcr + APBCLK);

	hw_data = kzalloc(struct_size(hw_data, hws, W55FA93_NUM_CLKS),
			  GFP_KERNEL);
	if (!hw_data)
		return;
	hw_data->num = W55FA93_NUM_CLKS;

	/* Fixed PLL clocks */
	hw = clk_hw_register_fixed_rate(NULL, "apll", "xin", 0, apll_rate);
	if (!IS_ERR(hw))
		hw_data->hws[W55FA93_CLK_APLL] = hw;
	hw = clk_hw_register_fixed_rate(NULL, "upll", "xin", 0, upll_rate);
	if (!IS_ERR(hw))
		hw_data->hws[W55FA93_CLK_UPLL] = hw;

	/* SD engine clock: UPLL through the CLKDIV2 divider */
	{
		struct w55fa93_sdclk *sdclk;
		struct clk_init_data init = { };

		sdclk = kzalloc(sizeof(*sdclk), GFP_KERNEL);
		if (!sdclk)
			goto err;
		sdclk->reg = gcr + CLKDIV2;
		sdclk->lock = &w55fa93_clk_lock;
		init.name = "sd-src";
		init.ops = &w55fa93_sdclk_ops;
		init.parent_hws = (const struct clk_hw *[]) {
			hw_data->hws[W55FA93_CLK_UPLL]
		};
		init.num_parents = 1;
		sdclk->hw.init = &init;
		ret = clk_hw_register(NULL, &sdclk->hw);
		if (ret) {
			kfree(sdclk);
			goto err;
		}
		sd_src_hw = &sdclk->hw;
	}

#define GATE(id, name, parent_hw, reg, bit)				\
	do {								\
		hw = clk_hw_register_gate_parent_hw(NULL, name,		\
			parent_hw, CLK_IGNORE_UNUSED, gcr + reg, bit,	\
			0, &w55fa93_clk_lock);				\
		if (!IS_ERR(hw))					\
			hw_data->hws[id] = hw;				\
	} while (0)

	GATE(W55FA93_CLK_VPOST, "vpost", hw_data->hws[W55FA93_CLK_UPLL],
	     AHBCLK, 27);
	GATE(W55FA93_CLK_SIC, "sic", sd_src_hw, AHBCLK, 21);
	GATE(W55FA93_CLK_SD, "sd", sd_src_hw, AHBCLK, 23);
	GATE(W55FA93_CLK_USBD, "usbd", hw_data->hws[W55FA93_CLK_UPLL],
	     AHBCLK, 18);
	GATE(W55FA93_CLK_USBH, "usbh", hw_data->hws[W55FA93_CLK_UPLL],
	     AHBCLK, 17);
#undef GATE

	/* APB gates: parent is xin (registered by name through the DT) */
#define GATE_XIN(id, name, reg, bit)					\
	do {								\
		hw = clk_hw_register_gate(NULL, name, "xin",		\
			CLK_IGNORE_UNUSED, gcr + reg, bit,		\
			0, &w55fa93_clk_lock);				\
		if (!IS_ERR(hw))					\
			hw_data->hws[id] = hw;				\
	} while (0)

	GATE_XIN(W55FA93_CLK_TIMER0, "timer0", APBCLK, 8);
	GATE_XIN(W55FA93_CLK_TIMER1, "timer1", APBCLK, 9);
	GATE_XIN(W55FA93_CLK_PWM, "pwm", APBCLK, 5);
	GATE_XIN(W55FA93_CLK_KPI, "kpi", APBCLK, 25);
	GATE_XIN(W55FA93_CLK_RTC, "rtc", APBCLK, 12);
	GATE_XIN(W55FA93_CLK_ADC, "adc", APBCLK, 11);
	GATE_XIN(W55FA93_CLK_UART0, "uart0", APBCLK, 16);
#undef GATE_XIN

	for (i = 0; i < W55FA93_NUM_CLKS; i++) {
		if (!hw_data->hws[i]) {
			pr_err("w55fa93-clk: clock %d failed to register\n", i);
			goto err;
		}
	}

	ret = of_clk_add_hw_provider(np, of_clk_hw_onecell_get, hw_data);
	if (ret) {
		pr_err("w55fa93-clk: failed to add provider: %d\n", ret);
		goto err;
	}
	ja720_paint(0xff00ff00);	/* GREEN: clk provider is up */
	return;

err:
	kfree(hw_data);
}

CLK_OF_DECLARE(w55fa93_clk, "nuvoton,w55fa93-clk", w55fa93_clk_init);
