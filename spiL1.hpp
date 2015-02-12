/**
 * @file spiL1.hpp
 * @brief STM32L1 SPI class.
 * @date January 2015
 * @author André Heßling
 *
 * Class for communicating with other SPI devices using the STM32L1 controller.
 */

#ifndef SPIL1_HPP_
#define SPIL1_HPP_

#include "spibase.hpp"

/** STM32L1 SPI class */
class SPI : public SPIBase
{
  /** @addtogroup SPI
   * @{
   */

public:
  /**
   * SPI constructor.
   *
   * @param spi SPI interface (SPI1, SPI2, ...) which should be used
   */
  SPI(SPI_TypeDef *spi) : SPIBase(spi) { };

  virtual ~SPI() { };

  virtual bool init();

  virtual void deinit();

  virtual uint16_t transfer(uint16_t data);

  virtual void select();

  virtual void unselect();

protected:
  virtual bool initClock();

  /** @}
   *
   */
};

#endif /* SPIL1_HPP_ */
