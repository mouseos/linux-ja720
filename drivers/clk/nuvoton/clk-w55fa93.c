// SPDX-License-Identifier: GPL-2.0-only
/*
 * Clock provider for the Nuvoton W55FA93 (N32905) SoC.
 *
 * Bring-up oriented provider:
 *  - APLL/UPLL rates are decoded from the PLLCON registers (FB_DV [8:0],
 *    IN_DV [13:9], OUT_DV [15:14]; Fout = Fin * (FB+2) / ((IN+2) * OD),
 *    OD = 1/2/4/8 for OUT_DV = 0/1/2/3), as in the vendor BSP.
 *  - AHBCLK/APBCLK gates are registered with CLK_IGNORE_UNUSED, and the
 *    clocks the port depends on (VPOST, SIC, SD engine, USBD, USBH,
 *    timers) are explicitly switched on at probe time, because the
 *    state left by the boot loader is not guaranteed for APB gates.
 *  - The timer gate parents are the external crystal clock, matching
 *    the BSP assumption CLOCK_TICK_RATE == EXTCLK rate.
 *  - The SIC (SD host) gate is fed from a nominal 48 MHz source clock;
 *    the exact CLKDIV1 SD divider programming is a TODO to be measured
 *    on hardware. The MMC driver derives its bus divider from this
 *    rate, so the bus stays at or below the requested speed.
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
#define APLLCON		0x220
#define UPLLCON		0x224

#define PLL_PD		BIT(16)
#define PLL_OUT_DV(x)	(((x) >> 14) & 0x3)
#define PLL_IN_DV(x)	(((x) >> 9) & 0x1f)
#define PLL_FB_DV(x)	((x) & 0x1ff)

static const unsigned int pll_outdiv[] = { 1, 2, 4, 8 };

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

	/* Nominal 48 MHz feed for the SIC/SD engine (TODO: measure) */
	hw = clk_hw_register_fixed_rate(NULL, "sd-src", "upll", 0, 48000000);
	if (IS_ERR(hw))
		goto err;
	sd_src_hw = hw;

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
	return;

err:
	kfree(hw_data);
}

CLK_OF_DECLARE(w55fa93_clk, "nuvoton,w55fa93-clk", w55fa93_clk_init);
