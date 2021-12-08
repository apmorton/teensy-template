/**************************************************************************
  This is a library for several Adafruit displays based on ST77* drivers.

  Works with the Adafruit 1.8" TFT Breakout w/SD card
    ----> http://www.adafruit.com/products/358
  The 1.8" TFT shield
    ----> https://www.adafruit.com/product/802
  The 1.44" TFT breakout
    ----> https://www.adafruit.com/product/2088
  The 1.54" TFT breakout
    ----> https://www.adafruit.com/product/3787
  The 2.0" TFT breakout
    ----> https://www.adafruit.com/product/4311
  as well as Adafruit raw 1.8" TFT display
    ----> http://www.adafruit.com/products/618

  Check out the links above for our tutorials and wiring diagrams.
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional).

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 **************************************************************************/
// This Teensy3 and 4 native optimized and extended version
// requires specific pins. 
// If you use the short version of the constructor and the DC
// pin is hardware CS pin, then it will be slower.

// This example sketch shows some of the extensions to the library.
// that for those Teensy devices that have enough memory allow you
// to tell the library to create a logical frame buffer for the device
// and you can choose when to transfer the data from the frame buffer
// to the device.  You can choose to do it, synchronous tft.updateScreen(),
// or you can do it Asynch (using DMA) tft.updateScreenAsync() which does it
// once or you can set to continuous updates: tft.updateScreenAsync().  This
// example uses the updateScreenAsync() call and allows you to define multiple
// displays with one on each buss to have them update at the same time. 

#include <Adafruit_GFX.h>    // Core graphics library
#include <ST7735_t3.h> // Hardware-specific library
#include <ST7789_t3.h> // Hardware-specific library
#include <SPI.h>

// undefine this if you wish to try sync updates, note:
// some processors will undefine this.
#define USE_FRAME_BUFFER 

#if defined(KINETISK)
//----------------------------------------------------------
// Define pins for Teensy 3.x 
//----------------------------------------------------------
// Teensy 3.x
#define TFT_SCLK 13  // SCLK can also use pin 14
#define TFT_MOSI 11  // MOSI can also use pin 7
#define TFT_CS   10  // CS & DC can use pins 2, 6, 9, 10, 15, 20, 21, 22, 23
#define TFT_DC    9  //  but certain pairs must NOT be used: 2+10, 6+9, 20+23, 21+22
#define TFT_RST   8  // RST can use any pin

#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
// SPI1 pins
#define TFT_SCLK1 32  // T3.5/T3.6
#define TFT_MOSI1 0   // 
#define TFT_CS1   30  // random digital pin
#define TFT_DC1   31   // Only Hardware CS pin on SPI1
#define TFT_RST1  29  // RST can use any pin

// SPI2 pins requires SDCard adapter
#define TFT_SCLK2 53  //
#define TFT_MOSI2 52  // 
//#define TFT_MISO2 51 // 0xe3
#define TFT_CS2   54  // CS pins (43, 54, 55)  
#define TFT_DC2   55  // Note: don't use 43 with 55
#define TFT_RESET 56

#define SPI_INTERFACES_COUNT 2  // only do 1 to start testing
#else   // T3.2
#define SPI_INTERFACES_COUNT 1  // only do 1 to start testing
#endif
#endif

//----------------------------------------------------------
// Define pins for Teensy 4
//----------------------------------------------------------
#if defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x

#define SPI_INTERFACES_COUNT 2  // only do 1 to start testing

#define TFT_CS   10  // 
#define TFT_DC    9  // Note pin 10 is only Hardware CS - slightly faster
#define TFT_RST   20  // Any digital pin or -1 if not used.

// Try defining 2nd on on SPI1
#define TFT_CS1     0   // Only hardware CS pin - maybe faster if DC 
#define TFT_DC1     2   //  
#define TFT_RST1    3   // Any digital pin or -1 
#define TFT_MOSI1  26
//#define TFT_MISO1   1   // Not used...
#define TFT_SCLK1  27

// Try defining 3rd on SPI2 - SDCard Ribbon connector
#define TFT_CS2    36    // Only hardware CS  
#define TFT_DC2    38    // 
#define TFT_RST2   39    // Any digital pin or -1
#define TFT_MOSI2  35 
//#define TFT_MISO2 34
#define TFT_SCLK2  37

#endif

#if defined(KINETISL)
//----------------------------------------------------------
// Define pins for Teensy LC
//----------------------------------------------------------
#define TFT_SCLK 13  // SCLK can also use pin 14
#define TFT_MOSI 11  // MOSI can also use pin 7
#define TFT_CS   10  // any digial pin or -1 if not used
#define TFT_DC    9  // any digital pin
#define TFT_RST   8  // any digital pin

// SPI1 pins
#define TFT_SCLK1 20  // 
#define TFT_MOSI1 0   // can be 21
#define TFT_CS1   19  // any digital pin
#define TFT_DC1   18  // any digital pin
#define TFT_RST1  17  // any digital pin
#define SPI_INTERFACES_COUNT 2  // only do 1 to start testing
#undef USE_FRAME_BUFFER   // not enough memory for this....
#endif

//----------------------------------------------------------
// Option to do synch or asynch...
//----------------------------------------------------------
#ifdef USE_FRAME_BUFFER
#define UpdateScreen _ptft->updateScreenAsync 
#else
#define UpdateScreen _ptft->updateScreen
#endif

//----------------------------------------------------------
// Define the Objects. 
// Note: See the graphictest example for examples showing
//       the different options for the constructors
//----------------------------------------------------------
// only uncomment one line in this section
ST7735_t3 tft = ST7735_t3(TFT_CS, TFT_DC, TFT_RST);
//ST7789_t3 tft = ST7789_t3(TFT_CS, TFT_DC, TFT_RST1);

#if SPI_INTERFACES_COUNT > 1
// Again only one
//ST7735_t3 tft1 = ST7735_t3(TFT_CS1, TFT_DC1, TFT_MOSI1, TFT_SCLK1, TFT_RST1);
ST7789_t3 tft1 = ST7789_t3(TFT_CS1, TFT_DC1, TFT_MOSI1, TFT_SCLK1, TFT_RST1);

#if SPI_INTERFACES_COUNT > 2
//ST7735_t3 tft2 = ST7735_t3(TFT_CS2, TFT_DC2, TFT_MOSI2, TFT_SCLK2, TFT_RST2);
ST7789_t3 tft2 = ST7789_t3(TFT_CS2, TFT_DC2, TFT_MOSI2, TFT_SCLK2, TFT_RST2);
#endif
#endif



//----------------------------------------------------------
// Table of tft objects.
//----------------------------------------------------------
ST7735_t3 * tfts[] =
{
  & tft

#if SPI_INTERFACES_COUNT > 1
  , & tft1

#if SPI_INTERFACES_COUNT > 2
  , & tft2
#endif
#endif

};

uint8_t tft_which_test[] =
{
  0, 0, 0
};

uint32_t last_test_start_time[] =
{
  0, 0, 0
};

uint32_t next_test_start_time[] =
{
  0, 0, 0
};

//=============================================================================
// Setup 
//=============================================================================
void setup()
{
  while (!Serial && millis() < 3000);
  Serial.begin(38400);

  Serial.println(F_CPU, DEC);

  //----------------------------------------------------------
  // Init the first display, only uncomment one in this
  // section which should coorespond to the ones defined in
  // objects.
  //----------------------------------------------------------
  //tft.initR(INITR_GREENTAB);
  tft.initR(INITR_144GREENTAB);
  //tft.initR(INITR_144GREENTAB_OFFSET);
  //tft.setRowColStart(0,0);
  //tft.init(240,240) ;     // use for ST7789
  //tft.init(240,320) ;     // use for ST7789 (newer 2')
  //tft.init(240,240, SPI_MODE2) ;  // use for ST7789 without CS)

  tft.fillScreen(ST7735_BLACK);
#ifdef USE_FRAME_BUFFER
  if(!tft.useFrameBuffer(true)) { // lets try using a frame buffer.
    Serial.println("TFT failed to create Frame buffer");
  }
#endif  
  Serial.println("tft displayed");

#if SPI_INTERFACES_COUNT > 1
  //----------------------------------------------------------
  // Init the second display, only uncomment one in this
  //----------------------------------------------------------
  //tft1.initR(INITR_144GREENTAB);
  //tft1.initR(INITR_144GREENTAB_OFFSET);
  //tft1.setRowColStart(0,0);
  tft1.init(240,240) ;  // use for ST7789
  //tft1.init(240,320) ;     // use for ST7789 (newer 2')
  //tft1.init(240,240, SPI_MODE2) ;  // use for ST7789 without CS)

  tft1.fillScreen(ST7735_BLACK);
#ifdef USE_FRAME_BUFFER
  if (!tft1.useFrameBuffer(true)) { // lets try using a frame buffer.
    Serial.println("TFT1 failed to create Frame buffer");
  }
#endif
  Serial.println("tft1 displayed");
#endif

#if SPI_INTERFACES_COUNT > 2
  //----------------------------------------------------------
  // Init the third display, only uncomment one in this
  //----------------------------------------------------------
  tft2.initR(INITR_144GREENTAB);
  //tft2.initR(INITR_144GREENTAB_OFFSET);
  //tft2.setRowColStart(0,0);
  //tft2.init(240,240, SPI_MODE3) ;  // use for ST7789
  //tft2.init(240,320) ;     // use for ST7789 (newer 2')
  //tft2.init(240,240, SPI_MODE2) ;  // use for ST7789 without CS)

  tft2.fillScreen(ST7735_BLACK);
#ifdef USE_FRAME_BUFFER
  if (!tft2.useFrameBuffer(true)) { // lets try using a frame buffer.
    Serial.println("TFT2 failed to create Frame buffer");
#endif
  Serial.println("tft2 displayed");
#endif

  delay(1000); // Delay 1000 ms

#if SPI_INTERFACES_COUNT > 1
#endif

#if SPI_INTERFACES_COUNT > 2
#endif
  //  Serial.printf("Hit any key to continue");
  //  while (Serial.read() == -1) ;
  //  while (Serial.read() != -1) ;
#ifdef A1
  randomSeed(analogRead(A0) + analogRead(A1));
#else
  randomSeed(analogRead(A0) + (analogRead(A0) << 1));
#endif
}

//=============================================================================
// Loop
//=============================================================================
void loop()
{
  // Lets see which of our displays is ready to display something different
  for (uint8_t i = 0; i < sizeof(tfts) / sizeof(tfts[0]); i++)
  {
    if ((millis() > next_test_start_time[i]) && !tfts[i] -> asyncUpdateActive())
    {
      //Serial.printf("%d: Start test: %d\n", i, tft_which_test[i]);
      last_test_start_time[i] = millis();
      switch (tft_which_test[i])
      {
        case 0:
          testlines(tfts[i], ST7735_YELLOW);
          break;
        case 1:
          testfastlines(tfts[i], ST7735_RED, ST7735_BLUE);
          break;
        case 2:
            testdrawrects(tfts[i], ST7735_GREEN);
            break;
        case 3:
            testfillrects(tfts[i], ST7735_YELLOW, ST7735_MAGENTA);
            break;
        case 4:
            testcircles(tfts[i], 10, ST7735_BLUE);
            break;
        case 5:
            testroundrects(tfts[i]);
            break;
        case 6:
            testtriangles(tfts[i]);
            break;
      }
      tft_which_test[i] ++;
      if (tft_which_test[i] > 6)tft_which_test[i] = 0;
      next_test_start_time[i] = millis() + 250;
    }
    else if ((millis() - last_test_start_time[i]) > 5000)
    {
      Serial.printf("Oled %d hung test: %d\n ", i, tft_which_test[i]);
      last_test_start_time[i] = millis();
    }
  }
}

//=============================================================================
// Test functions
//=============================================================================
void testlines(ST7735_t3 *_ptft, uint16_t color) {
  _ptft->fillScreen(ST7735_BLACK);
  for (int16_t x=0; x < _ptft->width(); x+=6) {
    _ptft->drawLine(0, 0, x, _ptft->height()-1, color);
  }
  for (int16_t y=0; y < _ptft->height(); y+=6) {
    _ptft->drawLine(0, 0, _ptft->width()-1, y, color);
  }

  _ptft->fillScreen(ST7735_BLACK);
  for (int16_t x=0; x < _ptft->width(); x+=6) {
    _ptft->drawLine(_ptft->width()-1, 0, x, _ptft->height()-1, color);
  }
  for (int16_t y=0; y < _ptft->height(); y+=6) {
    _ptft->drawLine(_ptft->width()-1, 0, 0, y, color);
  }

  _ptft->fillScreen(ST7735_BLACK);
  for (int16_t x=0; x < _ptft->width(); x+=6) {
    _ptft->drawLine(0, _ptft->height()-1, x, 0, color);
  }
  for (int16_t y=0; y < _ptft->height(); y+=6) {
    _ptft->drawLine(0, _ptft->height()-1, _ptft->width()-1, y, color);
  }

  _ptft->fillScreen(ST7735_BLACK);
  for (int16_t x=0; x < _ptft->width(); x+=6) {
    _ptft->drawLine(_ptft->width()-1, _ptft->height()-1, x, 0, color);
  }
  for (int16_t y=0; y < _ptft->height(); y+=6) {
    _ptft->drawLine(_ptft->width()-1, _ptft->height()-1, 0, y, color);
  }
  UpdateScreen();
}

void testdrawtext(ST7735_t3 *_ptft, const char *text, uint16_t color) {
  _ptft->setCursor(0, 0);
  _ptft->setTextColor(color);
  _ptft->setTextWrap(true);
  _ptft->print(text);
  UpdateScreen();
}

void testfastlines(ST7735_t3 *_ptft, uint16_t color1, uint16_t color2) {
  _ptft->fillScreen(ST7735_BLACK);
  for (int16_t y = 0; y < _ptft->height(); y += 5) {
    _ptft->drawFastHLine(0, y, _ptft->width(), color1);
  }
  for (int16_t x = 0; x < _ptft->width(); x += 5) {
    _ptft->drawFastVLine(x, 0, _ptft->height(), color2);
  }
  UpdateScreen();
}

void testdrawrects(ST7735_t3 *_ptft, uint16_t color) {
  _ptft->fillScreen(ST7735_BLACK);
  for (int16_t x = 0; x < _ptft->width(); x += 6) {
    _ptft->drawRect(_ptft->width() / 2 - x / 2, _ptft->height() / 2 - x / 2 , x, x, color);
  }
  UpdateScreen();
}

void testfillrects(ST7735_t3 *_ptft, uint16_t color1, uint16_t color2) {
  _ptft->fillScreen(ST7735_BLACK);
  for (int16_t x = _ptft->width() - 1; x > 6; x -= 6) {
    _ptft->fillRect(_ptft->width() / 2 - x / 2, _ptft->height() / 2 - x / 2 , x, x, color1);
    _ptft->drawRect(_ptft->width() / 2 - x / 2, _ptft->height() / 2 - x / 2 , x, x, color2);
  }
  UpdateScreen();
}

void testcircles(ST7735_t3 *_ptft, uint8_t radius, uint16_t color) {
  _ptft->fillScreen(ST7735_BLACK);
  for (int16_t x = radius; x < _ptft->width(); x += radius * 2) {
    for (int16_t y = radius; y < _ptft->height(); y += radius * 2) {
      _ptft->fillCircle(x, y, radius, color);
    }
  }
  for (int16_t x = 0; x < _ptft->width() + radius; x += radius * 2) {
    for (int16_t y = 0; y < _ptft->height() + radius; y += radius * 2) {
      _ptft->drawCircle(x, y, radius, color);
    }
  }
  UpdateScreen();
}

void testtriangles(ST7735_t3 *_ptft) {
  _ptft->fillScreen(ST7735_BLACK);
  int color = 0xF800;
  int t;
  int w = _ptft->width() / 2;
  int x = _ptft->height() - 1;
  int y = 0;
  int z = _ptft->width();
  for (t = 0 ; t <= 15; t += 1) {
    _ptft->drawTriangle(w, y, y, x, z, x, color);
    x -= 4;
    y += 4;
    z -= 4;
    color += 100;
  }
  UpdateScreen();
}

void testroundrects(ST7735_t3 *_ptft) {
  _ptft->fillScreen(ST7735_BLACK);
  int color = 100;
  int i;
  int t;
  for (t = 0 ; t <= 4; t += 1) {
    int x = 0;
    int y = 0;
    int w = _ptft->width() - 2;
    int h = _ptft->height() - 2;
    for (i = 0 ; i <= 16; i += 1) {
      _ptft->drawRoundRect(x, y, w, h, 5, color);
      x += 2;
      y += 3;
      w -= 4;
      h -= 6;
      color += 1100;
    }
    color += 100;
  }
  UpdateScreen();
}

void tftPrintTest(ST7735_t3 *_ptft) {
  _ptft->setTextWrap(false);
  _ptft->fillScreen(ST7735_BLACK);
  _ptft->setCursor(0, 30);
  _ptft->setTextColor(ST7735_RED);
  _ptft->setTextSize(1);
  _ptft->println("Hello World!");
  _ptft->setTextColor(ST7735_YELLOW);
  _ptft->setTextSize(2);
  _ptft->println("Hello World!");
  _ptft->setTextColor(ST7735_GREEN);
  _ptft->setTextSize(3);
  _ptft->println("Hello World!");
  _ptft->setTextColor(ST7735_BLUE);
  _ptft->setTextSize(4);
  _ptft->print(1234.567);
  UpdateScreen();
}
