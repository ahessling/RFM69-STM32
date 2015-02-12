/**
 * @file spiL1.cpp
 * @brief STM32L1 SPI class.
 * @date January 2015
 * @author André Heßling
 *
 * Class for communicating with other SPI devices using the STM32L1 controller.
 */

/** @addtogroup SPI
 * @{
 */
#include "spiL1.hpp"

/**
 * Initialize the peripheral clocks needed for SPI transfers.
 *
 * @return true if OK, else false.
 */
bool SPI::initClock()
{
  bool clockEnabled = true;

  // init clocks
  if (_spi == SPI1)
  {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  }
  else if (_spi == SPI2)
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  }
  else if (_spi == SPI3)
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
  }
  else
  {
    clockEnabled = false;
  }

  return clockEnabled;
}

/**
 * Init SPI peripheral.
 *
 * @return true if OK, else false
 */
bool SPI::init()
{
  // deinit before init
  if (true == _init)
    deinit();

  // enable peripheral clock
  if (false == initClock())
    return false;

  // configure SPI
  SPI_InitTypeDef SPI_InitStructure;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = (_bits == 8) ? SPI_DataSize_8b : SPI_DataSize_16b;
  SPI_InitStructure.SPI_CPOL = _cpol ? SPI_CPOL_High : SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = _cpha ? SPI_CPHA_2Edge : SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = _prescaler;
  SPI_InitStructure.SPI_FirstBit = _firstBitLSB ? SPI_FirstBit_LSB : SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(_spi, &SPI_InitStructure);

  // enable SPI
  SPI_Cmd(_spi, ENABLE);

  _init = true;

  return _init;
}

/**
 * Deinit the SPI peripheral.
 */
void SPI::deinit()
{
  // fixme: check if transfer is pending
  SPI_I2S_DeInit(_spi);

  _init = false;
}

/**
 * SPI read/write.
 *
 * Depending on the number of bits per transfer unit,
 * 8 or 16 bits are read/written.
 *
 * @param data Data to be sent
 * @return Data received
 */
uint16_t SPI::transfer(uint16_t data)
{
  // truncate data if necessary
  if (8 == _bits)
    data &= 0xff;

  // SPI must be init'd
  if (false == _init)
    return 0;

  // wait for TX buffer empty (transfer finished)
  while ((_spi->SR & SPI_I2S_FLAG_TXE) == RESET);

  // transfer
  _spi->DR = data;

  // wait for RX buffer full (transfer finished)
  while ((_spi->SR & SPI_I2S_FLAG_RXNE) == RESET);

  // get received data
  data = _spi->DR;

  // truncate data if necessary
  if (8 == _bits)
    return data & 0xff;
  else
    return data;
}

/**
 * Select the device.
 *
 * Can be used if only one device shall be addressed.
 *
 * @note Can only be used if configureCS() has been called before.
 */
void SPI::select()
{
  if (_csGPIO != 0)
    _csGPIO->BSRRH = _csPin;
}

/**
 * Release the device.
 *
 * Can be used if only one device shall be addressed.
 *
 * @note Can only be used if configureCS() has been called before.
 */
void SPI::unselect()
{
  if (_csGPIO != 0)
    _csGPIO->BSRRL = _csPin;
}

/** @}
 *
 */
