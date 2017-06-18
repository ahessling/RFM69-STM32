/**
 * @file spiF0.hpp
 *
 * @date 12.12.2015
 * @author Andre
 * @description
 */
#ifndef SPIF0_HPP_
#define SPIF0_HPP_

#include "spibase.hpp"

class SPI : public SPIBase
{
public:
	SPI(SPI_TypeDef *spi) : SPIBase(spi) { };
	virtual ~SPI() { };

	virtual bool init();

	virtual void deinit();

	virtual uint16_t transfer(uint16_t data);

	virtual void reconfigure();
  
  virtual void select();

  virtual void unselect();

protected:
	virtual bool initClock();
};





#endif /* SPIF0_HPP_ */
