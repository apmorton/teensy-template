// SpeedTest written by Peter Loveday
//
// http://forum.pjrc.com/threads/15576-Teensy3-ST7735-Library?p=21355&viewfull=1#post21355

// This Teensy3 native optimized version requires specific pins
//
#define TFT_SCLK 13  // SCLK can also use pin 14
#define TFT_MOSI 11  // MOSI can also use pin 7
#define TFT_CS   10  // CS & DC can use pins 2, 6, 9, 10, 15, 20, 21, 22, 23
#define TFT_DC    9  //  but certain pairs must NOT be used: 2+10, 6+9, 20+23, 21+22
#define TFT_RST   8  // RST can use any pin
#define SD_CS     4  // CS for SD card, can use any pin

#include <Adafruit_GFX.h>
#include <ST7735_t3.h>
#include <SPI.h>

ST7735_t3 disp = ST7735_t3(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

#define RGB(r,g,b) (b<<11|g<<6|r)

float pi = 3.1415926f;

void testlines(uint16_t color)
{
	disp.fillScreen(ST7735_BLACK);

	for (int16_t x=0; x < disp.width(); x+=6)
		disp.drawLine(0, 0, x, disp.height()-1, color);

	for (int16_t y=0; y < disp.height(); y+=6)
		disp.drawLine(0, 0, disp.width()-1, y, color);

	for (int16_t x=0; x < disp.width(); x+=6)
		disp.drawLine(disp.width()-1, 0, x, disp.height()-1, color);

	for (int16_t y=0; y < disp.height(); y+=6)
		disp.drawLine(disp.width()-1, 0, 0, y, color);

	for (int16_t x=0; x < disp.width(); x+=6)
		disp.drawLine(0, disp.height()-1, x, 0, color);

	for (int16_t y=0; y < disp.height(); y+=6)
		disp.drawLine(0, disp.height()-1, disp.width()-1, y, color);

	for (int16_t x=0; x < disp.width(); x+=6)
		disp.drawLine(disp.width()-1, disp.height()-1, x, 0, color);

	for (int16_t y=0; y < disp.height(); y+=6)
		disp.drawLine(disp.width()-1, disp.height()-1, 0, y, color);
}

void testdrawtext(const char *text, uint16_t color, uint16_t bgcolor)
{
	disp.setCursor(0, 0);
	disp.setTextColor(color, bgcolor);
	disp.setTextWrap(true);
	disp.print(text);
}

void testfastlines(uint16_t color1, uint16_t color2)
{
	disp.fillScreen(ST7735_BLACK);

	for (int16_t y=0; y < disp.height(); y+=5)
		disp.drawFastHLine(0, y, disp.width(), color1);

	for (int16_t x=0; x < disp.width(); x+=5)
		disp.drawFastVLine(x, 0, disp.height(), color2);
}

void testdrawrects(uint16_t color)
{
	disp.fillScreen(ST7735_BLACK);

	for (int16_t x=0; x < disp.width(); x+=6)
		disp.drawRect(disp.width()/2 -x/2, disp.height()/2 -x/2 , x, x, color);
}

void testfillrects(uint16_t color1, uint16_t color2)
{
	disp.fillScreen(ST7735_BLACK);

	for (int16_t x=disp.width()-1; x > 6; x-=6) {
		disp.fillRect(disp.width()/2 -x/2, disp.height()/2 -x/2 , x, x, color1);
		disp.drawRect(disp.width()/2 -x/2, disp.height()/2 -x/2 , x, x, color2);
	}
}

void testfillcircles(uint8_t radius, uint16_t color)
{
	for (int16_t x=radius; x < disp.width(); x+=radius*2)
		for (int16_t y=radius; y < disp.height(); y+=radius*2)
			disp.fillCircle(x, y, radius, color);
}

void testdrawcircles(uint8_t radius, uint16_t color)
{
	for (int16_t x=0; x < disp.width()+radius; x+=radius*2)
		for (int16_t y=0; y < disp.height()+radius; y+=radius*2)
			disp.drawCircle(x, y, radius, color);
}

void testtriangles()
{
	disp.fillScreen(ST7735_BLACK);

	int color = 0xF800;
	int t;
	int w = 63;
	int x = 159;
	int y = 0;
	int z = 127;

	for (t = 0 ; t <= 15; t+=1) {
		disp.drawTriangle(w, y, y, x, z, x, color);
		x-=4;
		y+=4;
		z-=4;
		color+=100;
	}
}

void testroundrects()
{
	disp.fillScreen(ST7735_BLACK);

	int color = 100;
	int i;
	int t;

	for(t = 0 ; t <= 4; t+=1) {
		int x = 0;
		int y = 0;
		int w = 127;
		int h = 159;
		for(i = 0 ; i <= 24; i+=1) {
			disp.drawRoundRect(x, y, w, h, 5, color);
			x+=2;
			y+=3;
			w-=4;
			h-=6;
			color+=1100;
		}
		color+=100;
	}
}

#define SETCOLOR(c) disp.setTextColor(c, bg ? ST7735_BLACK : c);

void tftPrintTest(bool bg)
{
	disp.setTextWrap(false);
	disp.fillScreen(ST7735_BLACK);
	disp.setCursor(0, 30);
	SETCOLOR(ST7735_RED);
	disp.setTextSize(1);
	disp.println("Hello World!");
	SETCOLOR(ST7735_YELLOW);
	disp.setTextSize(2);
	disp.println("Hello World!");
	SETCOLOR(ST7735_GREEN);
	disp.setTextSize(3);
	disp.println("Hello World!");
	SETCOLOR(ST7735_BLUE);
	disp.setTextSize(4);
	disp.print(1234.567);

	disp.setCursor(0, 0);
	disp.fillScreen(ST7735_BLACK);
	SETCOLOR(ST7735_WHITE);
	disp.setTextSize(0);
	disp.println("Hello World!");
	disp.setTextSize(1);
	SETCOLOR(ST7735_GREEN);
	disp.print(pi, 6);
	disp.println(" Want pi?");
	disp.println(" ");
	disp.print(8675309, HEX);
	disp.println(" Print HEX!");
	disp.println(" ");
	SETCOLOR(ST7735_WHITE);
	disp.println("Sketch has been");
	disp.println("running for: ");
	SETCOLOR(ST7735_MAGENTA);
	disp.print(millis() / 1000);
	SETCOLOR(ST7735_WHITE);
	disp.print(" seconds.");
}

void mediabuttons()
{
	// play
	disp.fillScreen(ST7735_BLACK);
	disp.fillRoundRect(25, 10, 78, 60, 8, ST7735_WHITE);
	disp.fillTriangle(42, 20, 42, 60, 90, 40, ST7735_RED);

	// pause
	disp.fillRoundRect(25, 90, 78, 60, 8, ST7735_WHITE);
	disp.fillRoundRect(39, 98, 20, 45, 5, ST7735_GREEN);
	disp.fillRoundRect(69, 98, 20, 45, 5, ST7735_GREEN);

	// play color
	disp.fillTriangle(42, 20, 42, 60, 90, 40, ST7735_BLUE);

	// pause color
	disp.fillRoundRect(39, 98, 20, 45, 5, ST7735_RED);
	disp.fillRoundRect(69, 98, 20, 45, 5, ST7735_RED);
	// play color
	disp.fillTriangle(42, 20, 42, 60, 90, 40, ST7735_GREEN);
}

int t[20];
int n = 0;

void setup()
{
	pinMode(SD_CS, INPUT_PULLUP);  // keep SD CS high when not using SD card

	// Use this initializer if you're using a 1.8" TFT
	disp.initR(INITR_BLACKTAB);
	// Use this initializer (uncomment) if you're using a 1.44" TFT
	//disp.initR(INITR_144GREENTAB);

	disp.setRotation(0);
	disp.setTextWrap(true);
	disp.setTextColor(RGB(31,31,31), RGB(0,0,0));
	disp.setCursor(0, 0);

	disp.fillScreen(RGB(0,0,0));

	t[n++] = millis();
	disp.fillScreen(ST7735_BLACK);
	t[n++] = millis();
	testdrawtext("Lorem ipsum dolor sit amet, consectetur adipiscing elit. Curabitur adipiscing ante sed nibh tincidunt feugiat. Maecenas enim massa, fringilla sed malesuada et, malesuada sit amet turpis. Sed porttitor neque ut ante pretium vitae malesuada nunc bibendum. Nullam aliquet ultrices massa eu hendrerit. Ut sed nisi lorem. In vestibulum purus a tortor imperdiet posuere. ", ST7735_WHITE, ST7735_WHITE);
	t[n++] = millis();
	tftPrintTest(false);
	t[n++] = millis();
	testdrawtext("Lorem ipsum dolor sit amet, consectetur adipiscing elit. Curabitur adipiscing ante sed nibh tincidunt feugiat. Maecenas enim massa, fringilla sed malesuada et, malesuada sit amet turpis. Sed porttitor neque ut ante pretium vitae malesuada nunc bibendum. Nullam aliquet ultrices massa eu hendrerit. Ut sed nisi lorem. In vestibulum purus a tortor imperdiet posuere. ", ST7735_WHITE, ST7735_BLACK);
	t[n++] = millis();
	tftPrintTest(true);
	t[n++] = millis();
	testlines(ST7735_YELLOW);
	t[n++] = millis();
	testfastlines(ST7735_RED, ST7735_BLUE);
	t[n++] = millis();
	testdrawrects(ST7735_GREEN);
	t[n++] = millis();
	testfillrects(ST7735_YELLOW, ST7735_MAGENTA);
	t[n++] = millis();
	disp.fillScreen(ST7735_BLACK);
	t[n++] = millis();
	testfillcircles(10, ST7735_BLUE);
	t[n++] = millis();
	testdrawcircles(10, ST7735_WHITE);
	t[n++] = millis();
	testroundrects();
	t[n++] = millis();
	testtriangles();
	t[n++] = millis();
	mediabuttons();
	t[n++] = millis();
	disp.fillScreen(RGB(0,0,0));
}

// These are the results for the adafruit library, so we can report results as Nx

uint16_t normalize[] =
{
	129,
	216,
	348,
	955,
	630,
	1658,
	185,
	168,
	955,
	129,
	195,
	163,
	604,
	362,
	253,
	6950
};

void loop()
{
	static float fps = 0.0;
	static int h = 0;

	uint8_t r,g,b;
	if (h < 32) {
		r = 31; g = h; b = 0;
	} else if (h < 64) {
		r = 63-h; g = 31; b = 0;
	} else if (h < 96) {
		r = 0; g = 31; b = h-64;
	} else if (h < 128) {
		r = 0; g = 127-h; b = 31;
	} else if (h < 160) {
		r = h-128; g = 0; b = 31;
	} else if (h < 192) {
		r = 31; g = 0; b = 191-h;
	} else {
		r = 31; g = 0; b = 0;
		h = 0;
	}

	h++;

	int start = micros();
	disp.fillRect(120, 0, 8, 160, RGB(r,g,b));

	disp.setTextWrap(false);
	disp.setTextColor(RGB(31,31,31), RGB(0,0,0));
	disp.setCursor(0, 0);

	for (int i=1; i < n; i++) {
		disp.print(i < 10 ? "Test  " : "Test ");
		disp.print(i, DEC);
		disp.print(": ");
		disp.println(float(normalize[i-1])/float(t[i]-t[i-1]), 2);
	}

	disp.println("");
	disp.print("Total  : ");
	disp.println(float(normalize[n-1])/float(t[n-1]-t[0]), 2);

	disp.println("");
	disp.print("FPS    : ");
	disp.println(fps, 2);

	int end = micros();

	fps = 1000000.0f/float(end-start);
}
