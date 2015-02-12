RFM69-STM32
===========

Introduction
------------
This is a protocol agnostic driver library for handling HopeRF's RFM69 433/868/915 MHz RF modules.
This library is written for the STM32 family of controllers, but can easily be ported to other devices.

Support is also available for the +20 dBm high power modules called RFM69HW/RFM69HCW.

A CSMA/CA (carrier sense multiple access) algorithm can be enabled to avoid collisions.
If you want to enable CSMA, you should initialize the random number generator before.

Usage
-----
You have to provide your own functions for `delay_ms` and `mstimer_get`.
Use the SysTick timer (for example) with a 1 ms resolution which is present on all ARM controllers.

If you want to port this library to other devices, you have to provide an SPI instance
derived from the `SPIBase` (see `spibase.hpp`) class.

Examples
--------
TODO
