/***************************************************
  This is an example sketch for the Adafruit 1.8" SPI display.

  This library works with the Adafruit 1.8" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/358
  The 1.8" TFT shield
  ----> https://www.adafruit.com/product/802
  The 1.44" TFT breakout
  ----> https://www.adafruit.com/product/2088
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

#include <ST7735_t3.h>    // Hardware-specific library
#include <ST7789_t3.h>    // Hardware-specific library
#include <SPI.h>
#include <SD.h>

//#define IMAGE_COLORS_BGR // set if you colors are reversed

// This Teensy3 native optimized version requires specific pins
//
#define TFT_SCLK 13  // SCLK can also use pin 14
#define TFT_MOSI 11  // MOSI can also use pin 7
#define TFT_CS   10  // CS & DC can use pins 2, 6, 9, 10, 15, 20, 21, 22, 23
#define TFT_DC    9  //  but certain pairs must NOT be used: 2+10, 6+9, 20+23, 21+22
#define TFT_RST   8  // RST can use any pin
#define SD_CS     4  // CS for SD card, can use any pin

// Use one or the other
ST7735_t3 tft = ST7735_t3(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
//ST7789_t3 tft = ST7789_t3(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

void setup(void) {
  pinMode(SD_CS, INPUT_PULLUP);  // keep SD CS high when not using SD card
  Serial.begin(9600);

  // only uncomment one of these display initializers.
  // ST7735 - More options mentioned in examples for st7735_t3 library
  //tft.initR(INITR_BLACKTAB); // if you're using a 1.8" TFT 128x160 displays
  tft.initR(INITR_144GREENTAB); // if you're using a 1.44" TFT (128x128)
  //tft.initR(INITR_MINI160x80);  //if you're using a .96" TFT(160x80)

  // ST7789
  //tft.init(240, 240);           // initialize a ST7789 chip, 240x240 pixels
  //tft.init(240, 320);           // Init ST7789 2.0" 320x240
  //tft.init(135, 240);             // Init ST7789 1.4" 135x240
  //tft.init(240, 240, SPI_MODE2);    // clones Init ST7789 240x240 no CS
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.fillScreen(ST77XX_RED);
  tft.setCursor(ST7735_t3::CENTER, ST7735_t3::CENTER);
  tft.print("RED");
  delay(500);
  tft.fillScreen(ST77XX_GREEN);
  tft.setCursor(ST7735_t3::CENTER, ST7735_t3::CENTER);
  tft.print("GREEN");
  delay(500);
  tft.fillScreen(ST77XX_BLUE);
  tft.setCursor(ST7735_t3::CENTER, ST7735_t3::CENTER);
  tft.print("BLUE");
  delay(500);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0,0);
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("failed!");
    tft.fillScreen(ST7735_BLACK);
    tft.setCursor(5, tft.height() / 2 - 6);
    tft.print("Unable to access");
    tft.setCursor(32, tft.height() / 2 + 6);
    tft.print("SD card");
    while (1) {
      // do nothing if SD card not working
    }
  }
  Serial.println("OK!");

  bmpDraw("parrot.bmp", 0, 0);
}

void loop() {
  // uncomment these lines to draw bitmaps in different locations/rotations!
  /*
    tft.fillScreen(ST7735_BLACK); // Clear display
    for (int i=0; i<4; i++)       // Draw 4 parrots
      bmpDraw("parrot.bmp", tft.width() / 4 * i, tft.height() / 4 * i);
    delay(1000);
    tft.setRotation(tft.getRotation() + 1); // Inc rotation 90 degrees
  */
}

// This function opens a Windows Bitmap (BMP) file and
// displays it at the given coordinates.  It's sped up
// by reading many pixels worth of data at a time
// (rather than pixel by pixel).  Increasing the buffer
// size takes more of the Arduino's precious RAM but
// makes loading a little faster.  20 pixels seems a
// good balance.

#define BUFFPIXEL 20

//===========================================================
// Try Draw using writeRect
void bmpDraw(const char *filename, uint8_t x, uint16_t y) {

  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3 * BUFFPIXEL]; // pixel buffer (R+G+B per pixel)
  uint16_t buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();

  uint16_t awColors[320];  // hold colors for one row at a time...

  if ((x >= tft.width()) || (y >= tft.height())) return;

  Serial.println();
  Serial.print(F("Loading image '"));
  Serial.print(filename);
  Serial.println('\'');

  // Open requested file on SD card
  bmpFile = SD.open(filename);
  if (!bmpFile) {
    Serial.print("File not found");
    tft.fillScreen(ST7735_BLACK);
    tft.setCursor(12, tft.height() / 2 - 12);
    tft.print("Unable to");
    tft.setCursor(12, tft.height() / 2 - 0);
    tft.print("read file: ");
    tft.setCursor(12, tft.height() / 2 + 12);
    tft.setTextColor(ST7735_YELLOW);
    tft.print(filename);
    return;
  }

  // Parse BMP header
  if (read16(bmpFile) == 0x4D42) { // BMP signature
    Serial.print(F("File size: ")); Serial.println(read32(bmpFile));
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    Serial.print(F("Image Offset: ")); Serial.println(bmpImageoffset, DEC);
    // Read DIB header
    Serial.print(F("Header size: ")); Serial.println(read32(bmpFile));
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if (read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      Serial.print(F("Bit Depth: ")); Serial.println(bmpDepth);
      if ((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed

        goodBmp = true; // Supported BMP format -- proceed!
        Serial.print(F("Image size: "));
        Serial.print(bmpWidth);
        Serial.print('x');
        Serial.println(bmpHeight);

        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;

        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if (bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
        if ((x + w - 1) >= tft.width())  w = tft.width()  - x;
        if ((y + h - 1) >= tft.height()) h = tft.height() - y;

        for (row = 0; row < h; row++) { // For each scanline...

          // Seek to start of scan line.  It might seem labor-
          // intensive to be doing this on every line, but this
          // method covers a lot of gritty details like cropping
          // and scanline padding.  Also, the seek only takes
          // place if the file position actually needs to change
          // (avoids a lot of cluster math in SD library).
          if (flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;
          if (bmpFile.position() != pos) { // Need seek?
            bmpFile.seek(pos);
            buffidx = sizeof(sdbuffer); // Force buffer reload
          }

          for (col = 0; col < w; col++) { // For each pixel...
            // Time to read more pixel data?
            if (buffidx >= sizeof(sdbuffer)) { // Indeed
              bmpFile.read(sdbuffer, sizeof(sdbuffer));
              buffidx = 0; // Set index to beginning
            }

            // Convert pixel from BMP to TFT format, push to display
            #ifdef IMAGE_COLORS_BGR
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            #else
            r = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            b = sdbuffer[buffidx++];
            #endif
            awColors[col] = tft.Color565(r, g, b);
          } // end pixel
          tft.writeRect(0, row, w, 1, awColors);
        } // end scanline
        Serial.print(F("Loaded in "));
        Serial.print(millis() - startTime);
        Serial.println(" ms");
      } // end goodBmp
    }
  }

  bmpFile.close();
  if (!goodBmp) Serial.println(F("BMP format not recognized."));
}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(File f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(File f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}
