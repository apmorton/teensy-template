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

#include "ST7789_t3.h"
#include <limits.h>
#include "pins_arduino.h"
#include "wiring_private.h"
#include <SPI.h>

 
#define ST77XX_MADCTL_MY  0x80
#define ST77XX_MADCTL_MX  0x40
#define ST77XX_MADCTL_MV  0x20
#define ST77XX_MADCTL_ML  0x10
#define ST77XX_MADCTL_RGB 0x00

ST7789_t3::ST7789_t3(uint8_t CS, uint8_t RS, uint8_t SID, uint8_t SCLK, uint8_t RST) : ST7735_t3(CS, RS, SID, SCLK, RST) 
{
  // Assume the majority of ones.
  tabcolor = INIT_ST7789_TABCOLOR;
  _screenHeight = 240;
  _screenWidth = 240;   
  
	cursor_y  = cursor_x    = 0;
	textsize_x  = 1;
  textsize_y  = 1;
	textcolor = textbgcolor = 0xFFFF;
	wrap      = true;
	font      = NULL;
	setClipRect();
	setOrigin();
}

ST7789_t3::ST7789_t3(uint8_t CS, uint8_t RS, uint8_t RST) : 
      ST7735_t3(CS, RS, RST) 
{
  tabcolor = INIT_ST7789_TABCOLOR;
  _screenHeight = 240;
  _screenWidth = 240; 

	cursor_y  = cursor_x    = 0;
	textsize_x  = 1;
  textsize_y  = 1;
	textcolor = textbgcolor = 0xFFFF;
	wrap      = true;
	font      = NULL;
	setClipRect();
	setOrigin();
}


void  ST7789_t3::setRotation(uint8_t m) 
{
  beginSPITransaction();
  writecommand(ST7735_MADCTL);
  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
   case 0:
     writedata_last(ST77XX_MADCTL_MX | ST77XX_MADCTL_MY | ST77XX_MADCTL_RGB);

     _xstart = _colstart;
     _ystart = _rowstart;
     _width = _screenWidth;
     _height = _screenHeight;
     break;
   case 1:
     writedata_last(ST77XX_MADCTL_MY | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB);

     _xstart = _rowstart;
     _ystart = _colstart;
     _height = _screenWidth;
     _width = _screenHeight;
     break;
  case 2:
     writedata_last(ST77XX_MADCTL_RGB); 
    if ((_screenWidth == 135) && (_screenHeight == 240)) {
      _xstart = _colstart - 1;
      _ystart = _rowstart;
    } else {
      _xstart = 0;
      _ystart = 0;
    }
     _width = _screenWidth;
     _height = _screenHeight;
     break;

   case 3:
     writedata_last(ST77XX_MADCTL_MX | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB);
    if ((_screenWidth == 135) && (_screenHeight == 240)) {
      _xstart = _rowstart;
      _ystart = _colstart;
    } else {
      _xstart = 0;
      _ystart = 0;
    }
     _height = _screenWidth;
     _width = _screenHeight;
     break;
  }

  _rot = m;  
  endSPITransaction();
//  Serial.printf("Set rotation %d start(%d %d) row: %d, col: %d\n", m, _xstart, _ystart, _rowstart, _colstart);
  setClipRect();
  setOrigin();
	
	cursor_x = 0;
	cursor_y = 0;
}

#define ST7789_240x240_XSTART 0
#define ST7789_240x240_YSTART 80

// Probably should use generic names like Adafruit..
#define DELAY 0x80
static const uint8_t PROGMEM
  cmd_st7789[] = {                  // Initialization commands for 7735B screens
    9,                       // 9 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, no args, w/delay
      150,                     //    150 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, no args, w/delay
      255,                    //     255 = 500 ms delay
    ST7735_COLMOD , 1+DELAY,  //  3: Set color mode, 1 arg + delay:
      0x55,                   //     16-bit color
      10,                     //     10 ms delay
    ST7735_MADCTL , 1      ,  //  4: Memory access ctrl (directions), 1 arg:
      0x08,                   //     Row addr/col addr, bottom to top refresh
    ST7735_CASET  , 4      ,  //  5: Column addr set, 4 args, no delay:
      0x00, 
      0x00,                   //     XSTART = 0
      0x00, 
      240,                    //      XEND = 240
    ST7735_RASET  , 4      ,  // 6: Row addr set, 4 args, no delay:
      0x00, 
      0x00,                   //     YSTART = 0
      320>>8, 
      320 & 0xFF,             //      YEND = 320
    ST7735_INVON ,   DELAY,   // 7: hack
      10,
    ST7735_NORON  ,   DELAY,  // 8: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,   DELAY,  // 9: Main screen turn on, no args, w/delay
    255 };                  //     255 = 500 ms delay


void  ST7789_t3::init(uint16_t width, uint16_t height, uint8_t mode)
{
  Serial.printf("ST7789_t3::init mode: %x\n", mode);
	commonInit(NULL, mode);

  if ((width == 240) && (height == 240)) {
    _colstart = 0;
    _rowstart = 80;
  } else if ((width == 135) && (height == 240)) { // 1.13" display Their smaller display
    _colstart = 53;
    _rowstart = 40;
  } else {
    _colstart = 0;
    _rowstart = 0;
  }
  
  _height = height;
  _width = width;
  _screenHeight = height;
  _screenWidth = width;   

  commandList(cmd_st7789);
  setRotation(0);
  cursor_y  = cursor_x    = 0;
  textsize_x = textsize_y = 1;
  textcolor = textbgcolor = 0xFFFF;
  wrap      = true;
  font      = NULL;
  setClipRect();
  setOrigin();
  
}

