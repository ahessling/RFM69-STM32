/**
 * @file spibase.hpp
 *
 * @brief Abstract base class for SPI communication.
 * @date January 2015
 * @author André Heßling
 *
 * This base class provides the basic interface for communicating
 * with other SPI devices using the STM32 controller.
 *
 * Derive a concrete implementation from this class.
 */

#ifndef SPIBASE_HPP_
#define SPIBASE_HPP_

#include "hw_config.h"

/** STM32 SPI abstract base class. */
class SPIBase
{
public:
  /** @addtogroup SPI
   * @{
   */

  /**
   * SPIBase default constructor.
   *
   * @param spi SPI interface (SPI1, SPI2, ...) which should be used
   */
  SPIBase(SPI_TypeDef *spi)
  {
    _spi = spi;
    _init = false;
    _csGPIO = 0;
    _bits = 8;
    _prescaler = 0;
    _cpol = false;
    _cpha = false;
    _firstBitLSB = false;
    _csPin = 0;
  }

  virtual ~SPIBase()
  {
  }

  /**
   * Set /CS pin, useful if only one device shall be addressed.
   *
   * @param gpio GPIO port of /CS pin (GPIOA, GPIOB, ...)
   * @param csPin Pin of /CS
   */
  void configureCS(GPIO_TypeDef *gpio, uint16_t csPin)
  {
    _csGPIO = gpio;
    _csPin = csPin;
  }

  /**
   * Change the clock polarity.
   *
   * Default is Low.
   *
   * @param cpol low = false, high = true
   */
  void setCPOL(bool cpol)
  {
    _cpol = cpol;
  }

  /**
   * Change the clock phase.
   *
   * Default is first edge.
   *
   * @param cpol first edge = false, second edge = true
   */
  void setCPHA(bool cpol)
  {
    _cpha = cpol;
  }

  /**
   * Enable transmission of LSB as first bit.
   *
   * @param firstBitLSB true or false
   */
  void setFirstBitLSB(bool firstBitLSB)
  {
    _firstBitLSB = firstBitLSB;
  }

  /**
   * Change the number of bits per SPI transfer.
   *
   * Default is 8 bits (1 byte).
   *
   * @param bits 8 or 16 bits
   */
  virtual void setBits(unsigned char bits)
  {
    // support 8 and 16 bits mode
    if (8 == bits || 16 == bits)
      _bits = bits;
  }

  /**
   * Set the SPI prescaler and therefore change the SPI transfer speed.
   *
   * @param prescaler Prescaler
   */
  virtual void setPrescaler(uint16_t prescaler)
  {
    _prescaler = prescaler;
  }

  /**
   * Init SPI peripheral.
   *
   * @return true if OK, else false
   */
  virtual bool init() = 0;

  /**
   * Deinit the SPI peripheral.
   */
  virtual void deinit() = 0;

  /**
   * Select the device.
   *
   * Can be used if only one device shall be addressed.
   *
   * @note Can only be used if configureCS() has been called before.
   */
  virtual void select() = 0;

  /**
   * Release the device.
   *
   * Can be used if only one device shall be addressed.
   *
   * @note Can only be used if configureCS() has been called before.
   */
  virtual void unselect() = 0;

  /**
   * SPI read/write.
   *
   * Depending on the number of bits per transfer unit,
   * 8 or 16 bits are read/written.
   *
   * @param data Data to be sent
   * @return Data received
   */
  virtual uint16_t transfer(uint16_t data) = 0;

  /**
   * Check if SPI peripheral has been initialized before.
   *
   * @return true or false
   */
  bool isInit() { return _init; };

protected:
  /**
   * Initialize the peripheral clocks needed for SPI transfers.
   *
   * @return true if OK, else false.
   */
  virtual bool initClock() = 0;

  SPI_TypeDef* _spi; ///< SPI interface
  GPIO_TypeDef* _csGPIO; ///< GPIO port of /CS pin
  uint16_t _csPin; ///< GPIO pin of /CS
  unsigned char _bits; ///< Number of bits per SPI transfer unit
  bool _cpol; ///< Clock polarity
  bool _cpha; ///< Clock phase
  bool _init; ///< Flag: Initialized
  bool _firstBitLSB; ///< If enabled, the LSB is transmitted first
  uint16_t _prescaler; ///< Prescaler

private:
  /** @}
   *
   */
};

#endif /* SPIBASE_HPP_ */
