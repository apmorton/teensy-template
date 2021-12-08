/***************************************************
  This is our GFX example for the Adafruit ST7735 Breakout and Shield
  ----> http://www.adafruit.com/products/1651

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/


#include <ST7735_t3.h> // Hardware-specific library
#include <ST7789_t3.h> // Hardware-specific library
#include <SPI.h>
#include <st7735_t3_font_ComicSansMS.h>

#define TFT_MISO  12
#define TFT_MOSI  11  //a12
#define TFT_SCK   13  //a13
#define TFT_DC   9
#define TFT_CS   10
#define TFT_RST  8

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
ST7789_t3 tft = ST7789_t3(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCK, TFT_RST);

// If using the breakout, change pins as desired
//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

void setup() {

  Serial.begin(9600);
 
  tft.init(240, 320);           // Init ST7789 320x240
  tft.setRotation(3);
  tft.useFrameBuffer(true);
  tft.fillScreen(ST7735_BLACK);
  while (!Serial) ; 
  tft.setTextColor(ST7735_WHITE);  tft.setTextSize(1);
  tft.enableScroll();
  tft.setScrollTextArea(0,0,120,240);
  tft.setScrollBackgroundColor(ST7735_GREEN);

  tft.setCursor(180, 100);

  tft.setFont(ComicSansMS_12);
  tft.print("Fixed text");

  tft.setCursor(0, 0);

  tft.setTextColor(ST7735_BLACK); 

  for(int i=0;i<20;i++){
    tft.print("  this is line ");
    tft.println(i);
    tft.updateScreen();
    delay(100);
  }

  tft.fillScreen(ST7735_BLACK);
  tft.setScrollTextArea(40,50,120,120);
  tft.setScrollBackgroundColor(ST7735_GREEN);
  tft.setFont(ComicSansMS_10);

  tft.setTextSize(1);
  tft.setCursor(40, 50);

  for(int i=0;i<20;i++){
    tft.print("  this is line ");
    tft.println(i);
    tft.updateScreen();
    delay(500);
  }


}



void loop(void) {


}