/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Clock indices for the Nuvoton W55FA93 clock controller.
 * The provider (drivers/clk/nuvoton/clk-w55fa93.c) registers clocks in
 * exactly this order.
 */
#ifndef _DT_BINDINGS_CLOCK_NUVOTON_W55FA93_H
#define _DT_BINDINGS_CLOCK_NUVOTON_W55FA93_H

#define W55FA93_CLK_APLL	0	/* APLL output (decoded from APLLCON) */
#define W55FA93_CLK_UPLL	1	/* UPLL output (decoded from UPLLCON) */
#define W55FA93_CLK_VPOST	2	/* AHBCLK bit 27 */
#define W55FA93_CLK_SIC		3	/* AHBCLK bit 21 */
#define W55FA93_CLK_SD		4	/* AHBCLK bit 23 (SD engine) */
#define W55FA93_CLK_USBD	5	/* AHBCLK bit 18 */
#define W55FA93_CLK_USBH	6	/* AHBCLK bit 17 */
#define W55FA93_CLK_TIMER0	7	/* APBCLK bit 8 */
#define W55FA93_CLK_TIMER1	8	/* APBCLK bit 9 */
#define W55FA93_CLK_PWM		9	/* APBCLK bit 5 */
#define W55FA93_CLK_KPI		10	/* APBCLK bit 25 */
#define W55FA93_CLK_RTC		11	/* APBCLK bit 12 */
#define W55FA93_CLK_ADC		12	/* APBCLK bit 11 */
#define W55FA93_CLK_UART0	13	/* APBCLK bit 16 (high-speed UART) */

#define W55FA93_NUM_CLKS	14

#endif
