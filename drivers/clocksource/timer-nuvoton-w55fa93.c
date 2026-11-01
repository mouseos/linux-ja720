// SPDX-License-Identifier: GPL-2.0-only
/*
 * Clocksource/clockevent driver for the Nuvoton W55FA93 (N32905) timers.
 *
 * The timer block has two up-counting channels sharing one register
 * block. Channel 0 is used as clockevent device, channel 1 as a
 * free-running clocksource, exactly as in the vendor BSP
 * (arch/arm/mach-w55fa93/time.c of Linux 2.6.35):
 *
 *   TCSR:  [7:0] prescale, [16] TDR enable, [28:27] mode
 *          (0 = one-shot, 1 = periodic), [29] interrupt enable,
 *          [30] count enable
 *   TICR:  compare value; the counter counts up from 0 to TICR
 *   TDR:   current counter value
 *   TISR:  interrupt flags, write 1 to clear (bit 0 / bit 1)
 *
 * The counter input clock is the external crystal clock (12 MHz on
 * JA720), as assumed by the BSP (CLOCK_TICK_RATE == EXTCLK rate). The
 * clock is obtained from the DT; the prescaler is left at 0.
 *
 * Copyright (C) 2026 JA720 Linux port contributors
 */

#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/sched_clock.h>

#define TCSR0		0x00
#define TCSR1		0x04
#define TICR0		0x08
#define TICR1		0x0c
#define TDR0		0x10
#define TDR1		0x14
#define TISR		0x18

#define TCSR_TDREN	BIT(16)
#define TCSR_MODE_ONESHOT	(0 << 27)
#define TCSR_MODE_PERIODIC	(1 << 27)
#define TCSR_INTEN	BIT(29)
#define TCSR_COUNTEN	BIT(30)

struct w55fa93_timer {
	void __iomem *regs;
	unsigned long rate;
	struct clock_event_device evt;
};

static struct w55fa93_timer w55_timer;

/* ---- clocksource (channel 1) ---- */

static u64 w55fa93_clocksource_read(struct clocksource *cs)
{
	return readl(w55_timer.regs + TDR1);
}

static struct clocksource w55fa93_clocksource = {
	.name	= "w55fa93-timer1",
	.rating	= 200,
	.read	= w55fa93_clocksource_read,
	.mask	= CLOCKSOURCE_MASK(32),
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};

/* ---- clockevent (channel 0) ---- */

static int w55fa93_evt_shutdown(struct clock_event_device *evt)
{
	u32 val = readl(w55_timer.regs + TCSR0);

	val &= ~(TCSR_COUNTEN | TCSR_INTEN);
	writel(val, w55_timer.regs + TCSR0);
	return 0;
}

static int w55fa93_evt_set_oneshot(struct clock_event_device *evt)
{
	u32 val = readl(w55_timer.regs + TCSR0);

	val &= ~(0x3 << 27);
	val |= TCSR_MODE_ONESHOT | TCSR_INTEN;
	writel(val, w55_timer.regs + TCSR0);
	return 0;
}

static int w55fa93_evt_set_periodic(struct clock_event_device *evt)
{
	u32 ticks = w55_timer.rate / HZ;
	u32 val;

	writel(1, w55_timer.regs + TISR);	/* clear TIF0 */
	writel(ticks, w55_timer.regs + TICR0);
	val = readl(w55_timer.regs + TCSR0);
	val &= ~(0x3 << 27);
	val |= TCSR_MODE_PERIODIC | TCSR_COUNTEN | TCSR_INTEN;
	writel(val, w55_timer.regs + TCSR0);
	return 0;
}

static int w55fa93_evt_set_next_event(unsigned long delta,
				      struct clock_event_device *evt)
{
	u32 now = readl(w55_timer.regs + TDR0);
	u32 val;

	/* Up-counter: fire when TDR reaches TICR. */
	writel(now + delta, w55_timer.regs + TICR0);
	val = readl(w55_timer.regs + TCSR0);
	val |= TCSR_COUNTEN | TCSR_INTEN;
	writel(val, w55_timer.regs + TCSR0);
	return 0;
}

static irqreturn_t w55fa93_timer_irq(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;

	writel(1, w55_timer.regs + TISR);	/* clear TIF0 */
	evt->event_handler(evt);
	return IRQ_HANDLED;
}

static int __init w55fa93_timer_of_init(struct device_node *node)
{
	struct clock_event_device *evt = &w55_timer.evt;
	struct clk *clk;
	int irq, ret;

	w55_timer.regs = of_iomap(node, 0);
	if (!w55_timer.regs) {
		pr_err("w55fa93-timer: failed to map registers\n");
		return -ENOMEM;
	}

	clk = of_clk_get(node, 0);
	if (IS_ERR(clk)) {
		pr_err("w55fa93-timer: failed to get clock\n");
		return PTR_ERR(clk);
	}
	ret = clk_prepare_enable(clk);
	if (ret) {
		pr_err("w55fa93-timer: failed to enable clock\n");
		return ret;
	}
	w55_timer.rate = clk_get_rate(clk);
	if (!w55_timer.rate) {
		pr_err("w55fa93-timer: clock rate is zero\n");
		return -EINVAL;
	}

	/* Channel 0: clockevent */
	writel(0, w55_timer.regs + TCSR0);
	writel(1, w55_timer.regs + TISR);

	irq = irq_of_parse_and_map(node, 0);
	if (!irq) {
		pr_err("w55fa93-timer: failed to get IRQ\n");
		return -EINVAL;
	}

	evt->name = "w55fa93-timer0";
	evt->features = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT;
	evt->set_state_shutdown = w55fa93_evt_shutdown;
	evt->set_state_periodic = w55fa93_evt_set_periodic;
	evt->set_state_oneshot = w55fa93_evt_set_oneshot;
	evt->set_next_event = w55fa93_evt_set_next_event;
	evt->rating = 300;
	evt->cpumask = cpumask_of(0);

	ret = request_irq(irq, w55fa93_timer_irq, IRQF_TIMER, "w55fa93-timer0",
			  evt);
	if (ret) {
		pr_err("w55fa93-timer: failed to request IRQ %d\n", irq);
		return ret;
	}

	clockevents_config_and_register(evt, w55_timer.rate, 0xf, 0xffffffff);

	/* Channel 1: free-running clocksource */
	writel(0, w55_timer.regs + TCSR1);
	writel(2, w55_timer.regs + TISR);
	writel(0xffffffff, w55_timer.regs + TICR1);
	writel(TCSR_COUNTEN | TCSR_MODE_PERIODIC | TCSR_TDREN,
	       w55_timer.regs + TCSR1);

	ret = clocksource_register_hz(&w55fa93_clocksource, w55_timer.rate);
	if (ret)
		pr_warn("w55fa93-timer: clocksource registration failed\n");

	pr_info("w55fa93-timer: rate %lu Hz, clockevent IRQ %d\n",
		w55_timer.rate, irq);
	return 0;
}

TIMER_OF_DECLARE(w55fa93_timer, "nuvoton,w55fa93-timer",
		 w55fa93_timer_of_init);
