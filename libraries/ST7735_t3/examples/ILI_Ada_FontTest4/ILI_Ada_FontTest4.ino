#include <Adafruit_GFX.h>

#include <ST7735_t3.h> // Hardware-specific library
#include <ST7789_t3.h> // Hardware-specific library
#include <SPI.h>

#include "font_Arial.h"
#include "font_ArialBold.h"
#include "font_ComicSansMS.h"
#include "font_OpenSans.h"
#include "font_DroidSans.h"
#include "font_Michroma.h"
#include "font_Crystal.h"
#include "font_ChanceryItalic.h"

#define CENTER ST7735_t3::CENTER

// maybe a few GFX FOnts?
#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/FreeSerif9pt7b.h>

#define TFT_MISO  12
#define TFT_MOSI  11  //a12
#define TFT_SCK   13  //a13
#define TFT_DC   9 
#define TFT_CS   10  
#define TFT_RST  8

typedef struct {
  const ILI9341_t3_font_t *ili_font;
  const GFXfont       *gfx_font;
  const char          *font_name;
  uint16_t            font_fg_color;
  uint16_t            font_bg_color;
} ili_fonts_test_t;


const ili_fonts_test_t font_test_list[] = {
  {&Arial_12, nullptr,  "Arial_12", ST7735_WHITE, ST7735_WHITE},
  {&Arial_12_Bold, nullptr,  "ArialBold 12", ST7735_YELLOW, ST7735_YELLOW},
  {&ComicSansMS_12, nullptr,  "ComicSansMS 12", ST7735_GREEN, ST7735_GREEN},
  {&DroidSans_12, nullptr,  "DroidSans_12", ST7735_WHITE, ST7735_WHITE},
  {&Michroma_12, nullptr,  "Michroma_12", ST7735_YELLOW, ST7735_YELLOW},
  {&Crystal_16_Italic, nullptr,  "CRYSTAL_16", ST7735_BLACK, ST7735_YELLOW},
  {&Chancery_16_Italic, nullptr,  "Chancery_16_Italic", ST7735_GREEN, ST7735_GREEN},
  {&OpenSans16, nullptr,  "OpenSans 16", ST7735_RED, ST7735_YELLOW},
  {nullptr, &FreeMono9pt7b,  "GFX FreeMono9pt7b", ST7735_WHITE, ST7735_WHITE},
  {nullptr, &FreeMono9pt7b,  "GFX FreeMono9pt7b", ST7735_RED, ST7735_YELLOW},
  {nullptr, &FreeSerif9pt7b,  "GFX FreeSerif9pt7b", ST7735_WHITE, ST7735_WHITE},
  {nullptr, &FreeSerif9pt7b,  "GFX FreeSerif9pt7b", ST7735_RED, ST7735_YELLOW},

} ;


// Option 1: use any pins but a little slower
// Note: code will detect if specified pins are the hardware SPI pins
//       and will use hardware SPI if appropriate
// For 1.44" and 1.8" TFT with ST7735 use
//ST7789_t3 tft = ST7735_t3(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCK, TFT_RST);

// For 1.54" or other TFT with ST7789, This has worked with some ST7789
// displays without CS pins, for those you can pass in -1 or 0xff for CS
// More notes by the tft.init call
ST7789_t3 tft = ST7789_t3(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// Option 2: must use the hardware SPI pins
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)
// For 1.44" and 1.8" TFT with ST7735 use
//ST7735_t3 tft = ST7735_t3(TFT_CS, TFT_DC, TFT_RST);

// For 1.54" TFT with ST7789
//ST7789_t3 tft = ST7789_t3(TFT_CS, TFT_DC, TFT_RST);


uint8_t test_screen_rotation = 0;


void setup() {
  Serial.begin(38400);
  long unsigned debug_start = millis ();
  while (!Serial && ((millis () - debug_start) <= 5000)) ;
  Serial.println("Setup");
  // Use this initializer if you're using a 1.8" TFT 128x160 displays
  //tft.initR(INITR_BLACKTAB);

  // Or use this initializer (uncomment) if you're using a 1.44" TFT (128x128)
  //tft.initR(INITR_144GREENTAB);

  // Or use this initializer (uncomment) if you're using a .96" TFT(160x80)
  //tft.initR(INITR_MINI160x80);

  // Or use this initializer (uncomment) for Some 1.44" displays use different memory offsets
  // Try it if yours is not working properly
  // May need to tweek the offsets
  //tft.setRowColStart(32,0);

  // Or use this initializer (uncomment) if you're using a 1.54" 240x240 TFT
  tft.init(240, 240);   // initialize a ST7789 chip, 240x240 pixels

  // OR use this initializer (uncomment) if using a 2.0" 320x240 TFT:
  //tft.init(240, 320);           // Init ST7789 320x240

  // OR use this initializer (uncomment) if using a 240x240 clone
  // that does not have a CS pin2.0" 320x240 TFT:
  //tft.init(240, 240, SPI_MODE2);           // Init ST7789 240x240 no CS

  tft.setRotation(4);
  tft.fillWindow(ST7735_BLACK);

  tft.setTextColor(ST7735_WHITE);
  tft.setFont(Arial_12);
  tft.println("Arial_12");
  displayStuff();

  tft.setTextColor(ST7735_YELLOW);
  tft.setFont(Arial_12_Bold);
  tft.println("ArialBold 12");
  displayStuff();
  nextPage();
  tft.setTextColor(ST7735_GREEN);
  tft.setFont(ComicSansMS_12);
  tft.println("ComicSansMS 12");
  displayStuff();


  tft.setTextColor(ST7735_WHITE);
  tft.setFont(DroidSans_12);
  tft.println("DroidSans_12");
  displayStuff();
  nextPage();

  tft.setTextColor(ST7735_YELLOW);
  tft.setFont(Michroma_12);
  tft.println("Michroma_12");
  displayStuff();

  tft.setTextColor(ST7735_BLACK, ST7735_YELLOW);
  tft.setFont(Crystal_16_Italic);
  tft.println("CRYSTAL_16");
  displayStuff();

  nextPage();

  tft.setTextColor(ST7735_GREEN);
  tft.setFont(Chancery_16_Italic);
  tft.println("Chancery_16_Italic");
  displayStuff();

  //anti-alias font OpenSans
  tft.setTextColor(ST7735_RED, ST7735_YELLOW);
  tft.setFont(OpenSans16);
  tft.println("OpenSans 18");
  displayStuff();

  Serial.println("Basic Font Display Complete");
  Serial.println("Loop test for alt colors + font");
}

void loop()
{
  tft.setFont(Arial_12);
  Serial.printf("\nRotation: %d\n", test_screen_rotation);
  tft.setRotation(test_screen_rotation);
  tft.fillWindow(ST7735_RED);
  tft.setCursor(CENTER, CENTER);
  tft.printf("Rotation: %d", test_screen_rotation);
  test_screen_rotation = (test_screen_rotation + 1) & 0x3;
  /*  tft.setCursor(200, 300);
    Serial.printf("  Set cursor(200, 300), retrieved(%d %d)",
                  tft.getCursorX(), tft.getCursorY());
  */
  tft.setCursor(25, 25);
  tft.write('0');
  tft.setCursor(tft.width() - 25, 25);
  tft.write('1');
  tft.setCursor(25, tft.height() - 25);
  tft.write('2');
  tft.setCursor(tft.width() - 25, tft.height() - 25);
  tft.write('3');

  for (uint8_t font_index = 0; font_index < (sizeof(font_test_list) / sizeof(font_test_list[0])); font_index++) {
    nextPage();
    if (font_test_list[font_index].font_fg_color != font_test_list[font_index].font_bg_color)
      tft.setTextColor(font_test_list[font_index].font_fg_color, font_test_list[font_index].font_bg_color);
    else
      tft.setTextColor(font_test_list[font_index].font_fg_color);
    if (font_test_list[font_index].ili_font) tft.setFont(*font_test_list[font_index].ili_font);
    else tft.setFont(font_test_list[font_index].gfx_font);
    tft.println(font_test_list[font_index].font_name);
    displayStuff1();
  }
  nextPage();
}

uint32_t displayStuff()
{
  elapsedMillis elapsed_time = 0;
  tft.println("ABCDEFGHIJKLM");
  tft.println("nopqrstuvwxyz");
  tft.println("0123456789");
  tft.println("!@#$%^ &*()-");
  tft.println(); tft.println();
  return (uint32_t) elapsed_time;
}

uint32_t displayStuff1()
{
  elapsedMillis elapsed_time = 0;
  tft.println("ABCDEFGHIJKLM");
  tft.println("nopqrstuvwxyz");
  tft.println("0123456789");
  tft.println("!@#$%^ &*()-");

  int16_t cursorX = tft.getCursorX();
  int16_t cursorY = tft.getCursorY();

  uint16_t width = tft.width();
  uint16_t height = tft.height();
  Serial.printf("DS1 (%d,%d) %d %d\n", cursorX, cursorY, width, height);
  uint16_t rect_x = width / 2 - 50;
  uint16_t rect_y = height - 50;
  tft.drawRect(rect_x, rect_y, 100, 40, ST7735_WHITE);
  for (uint16_t y = rect_y + 5; y < rect_y + 40; y += 5)
    tft.drawFastHLine(rect_x + 1, y, 98, ST77XX_PINK);
  for (uint16_t x = rect_x + 5; x < rect_x + 100; x += 5)
    tft.drawFastVLine(x, rect_y + 1, 38, ST77XX_PINK);
  tft.setCursor(width / 2, height - 30, true);
  tft.print("Center");

  // Lets try again with CENTER X keyword.
  rect_y -= 60;
  tft.drawRect(rect_x, rect_y, 100, 40, ST77XX_PINK);
  for (uint16_t y = rect_y + 5; y < rect_y + 40; y += 5)
    tft.drawFastHLine(rect_x + 1, y, 98, ST7735_CYAN);
  for (uint16_t x = rect_x + 5; x < rect_x + 100; x += 5)
    tft.drawFastVLine(x, rect_y + 1, 38, ST7735_CYAN);
  tft.setCursor(CENTER, rect_y);
  tft.print("XCENTR");

  // Lets try again with CENTER Y keyword.
  rect_x = 50;
  rect_y = tft.height() / 2 - 25;
  tft.drawRect(rect_x, rect_y, 100, 50, ST7735_CYAN);
  for (uint16_t y = rect_y + 5; y < rect_y + 50; y += 5)
    tft.drawFastHLine(rect_x + 1, y, 98, ST77XX_PINK);
  for (uint16_t x = rect_x + 5; x < rect_x + 100; x += 5)
    tft.setCursor(50, CENTER);
  tft.print("YCENTR");



  tft.setCursor(cursorX, cursorY);
  static const char alternating_text[] = "AbCdEfGhIjKlM\rNoPqRsTuVwXyZ";

  for (uint8_t i = 0; i < (sizeof(alternating_text) - 1); i++) {
    if (i & 1) tft.setTextColor(ST7735_WHITE, ST7735_RED);
    else tft.setTextColor(ST7735_YELLOW, ST7735_BLUE);
    tft.write(alternating_text[i]);
  }

  tft.println(); tft.println();



  return (uint32_t) elapsed_time;
}

void nextPage()
{
  Serial.println("Press anykey to continue");
  while (Serial.read() == -1) ;
  while (Serial.read() != -1) ;

  tft.fillWindow(ST7735_BLACK);
  tft.setCursor(0, 0);
}
