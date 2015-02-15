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

Example
--------
```cpp
...
// setup SPI
SPI spiRF(SPI1);
spiRF.setPrescaler(SPI_BaudRatePrescaler_2);
spiRF.init();

// setup RFM69 and optional reset
RFM69 rfm69(&spiRF, GPIOA, GPIO_Pin_1, true); // false = RFM69W, true = RFM69HW
rfm69.setResetPin(GPIOA, GPIO_Pin_2);
rfm69.reset();

// init RF module and put it to sleep
rfm69.init();
rfm69.sleep();

// set output power
rfm69.setPowerDBm(10); // +10 dBm

// enable CSMA/CA algorithm
rfm69.setCSMA(true);

...

// send a packet and let RF module sleep
char testdata[] = {'H', 'e', 'l', 'l', 'o'};
rfm69.send(testdata, sizeof(testdata));
rfm69.sleep();

...

// check if a packet has been received
char rx[64];
int bytesReceived = rfm69.receive(rx, sizeof(rx));

if (bytesReceived > 0)
{
  printf("%d bytes received.", bytesReceived);
}

```

Power settings
--------------
The power settings of +20 dBm RFM69HW modules are a bit confusing. Look [here](http://blog.andrehessling.de/2015/02/07/figuring-out-the-power-level-settings-of-hoperfs-rfm69-hwhcw-modules/) for some explanations and experiments regarding power consumption, power output and RSSI values at the receiver.
