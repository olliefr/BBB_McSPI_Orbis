# Orbis Rotary Encoder (SPI bus) on BeagleBone Black

This project implements a bare-metal driver for [*Orbis Rotary Encoder*](https://www.rls.si/en/orbis-true-absolute-rotary-encoder) on a [*BeagleBone Black*](https://beagleboard.org/black) platform. The connecting interface is SPI.

The *BeagleBone Black* platform is based on the [TI *AM3358* processor](https://www.ti.com/processors/sitara-arm/am335x-cortex-a8/overview.html), which comes with a detailed [*Technical Reference Manual*](https://www.ti.com/lit/pdf/spruh73). However, some information in the chapter describing its SPI module is ambiguous and I had found at least one difference between what the manual says and what the hardware does. So I've been horsing around with the TI *XDS110 Debug Probe* and oscilloscope, working out the correct sequence of operations and learning the TI *StarterWare* no-OS platform support package API.

The operation modes that I had tested each have a tag whose name and the commit message explain the configuration parameters used.

&mdash; Oliver Frolovs, 2019
