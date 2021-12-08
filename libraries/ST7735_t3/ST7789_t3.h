/***************************************************
  This is a library for the Adafruit 1.8" SPI display.
  This library works with the Adafruit 1.8" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/358
  as well as Adafruit raw 1.8" TFT display
  ----> http://www.adafruit.com/products/618

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#ifndef __ST7789_t3_H_
#define __ST7789_t3_H_
#include "ST7735_t3.h"

class ST7789_t3 : public ST7735_t3 {

 public:

  ST7789_t3(uint8_t CS, uint8_t RS, uint8_t SID, uint8_t SCLK, uint8_t RST = -1);
  ST7789_t3(uint8_t CS, uint8_t RS, uint8_t RST = -1);

  virtual void  setRotation(uint8_t m);

  void  init(uint16_t width=240, uint16_t height=240, uint8_t mode=SPI_MODE0);
};


#endif
