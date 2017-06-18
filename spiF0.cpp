/**
 * @file spiF0.cpp
 *
 * @date 12.12.2015
 * @author Andre
 * @description
 */

#include "spiF0.hpp"

bool SPI::initClock()
{
  bool clockEnabled = true;

  // init clocks
  if (_spi == SPI1)
  {
    RCC->APB2ENR |= RCC_APB2Periph_SPI1;
  }
  else if (_spi == SPI2)
  {
    RCC->APB1ENR |= RCC_APB1Periph_SPI2;
  }
  else
  {
    clockEnabled = false;
  }

  return clockEnabled;
}

bool SPI::init()
{
  // deinit before init
  if (true == _init)
    deinit();

  // enable peripheral clock
  if (false == initClock())
    return false;

  // configure SPI
  reconfigure();

  _init = true;

  return _init;
}

void SPI::reconfigure()
{
  // Reset peripheral

  if (_spi == SPI1)
  {
    RCC->APB2RSTR |= RCC_APB2Periph_SPI1;
    RCC->APB2RSTR &= ~RCC_APB2Periph_SPI1;
  }
  else if (_spi == SPI2)
  {
    RCC->APB1RSTR |= RCC_APB1Periph_SPI2;
    RCC->APB1RSTR &= ~RCC_APB1Periph_SPI2;
  }

  // apply new configuration
  uint16_t cr1 = SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM | ((_prescaler & 0x07) << 3);

  // MSB or LSB first
  if (true == _firstBitLSB)
  {
    cr1 |= SPI_CR1_LSBFIRST;
  }

  // CPOL/CPHA
  if (true == _cpol)
  {
    cr1 |= SPI_CR1_CPOL;
  }

  if (true == _cpha)
  {
    cr1 |= SPI_CR1_CPHA;
  }

  // control 1 register
  _spi->CR1 = cr1;

  // RXNE flag after 1 byte (quarter FIFO)
  uint16_t cr2 = SPI_CR2_FRXTH;

  // set frame length
  if (8 == _bits)
  {
    cr2 |= SPI_DataSize_8b;
  }
  else if (16 == _bits)
  {
    cr2 |= SPI_DataSize_16b;
  }

  // control 2 register
  _spi->CR2 = cr2;

  // enable SPI
  _spi->CR1 |= SPI_CR1_SPE;
}

void SPI::deinit()
{
  // fixme: check if transfer is pending

  if (_spi == SPI1)
  {
    RCC->APB2RSTR |= RCC_APB2Periph_SPI1;
    RCC->APB2RSTR &= ~RCC_APB2Periph_SPI1;
    RCC->APB2ENR &= ~RCC_APB2Periph_SPI1;

  }
  else if (_spi == SPI2)
  {
    RCC->APB1RSTR |= RCC_APB1Periph_SPI2;
    RCC->APB1RSTR &= ~RCC_APB1Periph_SPI2;
    RCC->APB1ENR &= ~RCC_APB1Periph_SPI2;
  }

  _init = false;
}

uint16_t SPI::transfer(uint16_t data)
{
  // SPI must be init'd
  if (false == _init)
    return 0;

  // wait for TX buffer empty (transfer finished)
  while ((_spi->SR & SPI_I2S_FLAG_TXE) == RESET);

  // transfer
  if (8 == _bits)
  {
    // 8 bit write operation!
    *(uint8_t*)&(_spi->DR) = data & 0xFF;
  }
  else
  {
    // 16 bit write operation
    _spi->DR = data;
  }
  // wait for RX buffer full (transfer finished)
  while ((_spi->SR & SPI_I2S_FLAG_RXNE) == RESET);

  // get received data
  if (8 == _bits)
  {
    // 8 bit read operation!
    data = *(uint8_t*)&(_spi->DR) & 0xFF;
  }
  else
  {
    // 16 bit read operation
    data = _spi->DR;
  }

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
		_csGPIO->BRR = _csPin;
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
		_csGPIO->BSRR = _csPin;
}
