# linux-ja720

Linux kernel port for the **Nuvoton W55FA93** (N32905) SoC, targeting the
**Benesse Pocket Challenge 4 (model JA720)**.

Based on mainline Linux **6.18.60** (stable). This repository contains the
kernel source plus the W55FA93/JA720 platform support. The bootable SD
update image (kernel + initramfs with SSH) is built by the separate
[ja720-sd-image](https://github.com/mouseos/ja720-sd-image) repository from
the releases published here.

## Status

Bring-up in progress. Implemented:

- **AIC interrupt controller** (`drivers/irqchip/irq-nuvoton-w55fa93-aic.c`)
  — the W55FA93 AIC is the same block as the WPCM450 AIC.
- **Timer clocksource/clockevent** (`drivers/clocksource/timer-nuvoton-w55fa93.c`)
  — timer0 as clockevent, timer1 as clocksource, as in the vendor BSP.
- **Clock provider** (`drivers/clk/nuvoton/clk-w55fa93.c`)
  — APLL/UPLL rate decode, AHB/APB gates.
- **Device tree** (`arch/arm/boot/dts/nuvoton/nuvoton-w55fa93{,-ja720}.dts`)
  — 32 MiB SDRAM, loader-initialized 320x240 framebuffer via
  `simple-framebuffer` (fbcon works without a UART).
- **SIC SD host (MMC)** and **USB device (gadget)** drivers ported from
  the vendor BSP — under compile validation.

The board boots through the stock loader (no signature checks): the
image builder wraps the zImage with a 32-byte Besta header and a small
register shim, and appends the DTB. The DTB's `/chosen` node carries
the initrd location for the image built by ja720-sd-image.

## Build

CI builds every push to `main` (see `.github/workflows/build.yml`) and
publishes a `dev-<n>` release with `zImage`, `ja720.dtb`, `config` and
`System.map`. ccache is cached between runs, so rebuilds are
incremental.

Local build:

```sh
sudo apt install gcc-arm-linux-gnueabi bc bison flex libssl-dev libelf-dev
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabi- ja720_defconfig
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabi- -j$(nproc) zImage dtbs
# outputs: arch/arm/boot/zImage
#          arch/arm/boot/dts/nuvoton/nuvoton-w55fa93-ja720.dtb
```

## Hardware notes

Register maps, interrupt numbers, LCD timings and boot-chain details
were reverse engineered from the stock firmware and cross-checked
against the vendor Linux 2.6.35 BSP (OpenNuvoton `N32905_Linux_BSP`).
SDRAM is 32 MiB at `0x80000000`; the loader sets up VPOST with a
320x240, 32 bpp framebuffer at `0x81c00000`.

## License

GPL-2.0, as the Linux kernel.
