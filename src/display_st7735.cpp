#include <stdlib.h>
#include "Arduino.h"
#include "display_st7735.h"
//#include <Adafruit_GFX.h>
#include "ST7735_t3.h"
#include "hal.h"
#include "dbg.h"

#define RGB(r,g,b) (b<<11|g<<6|r)
#define SETCOLOR(c) disp.setTextColor(c, bg ? ST7735_BLACK : c);

ST7735_t3 disp = ST7735_t3(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

struct st7735_ctx
{
    int x;
    int y;
};

static struct st7735_ctx ctx = { 0 };

static char map_cell_type_to_char(enum cell_type type)
{
    const char char_map[] = 
    {
        [CELL_TYPE_EMPTY]       = ' ',
        [CELL_TYPE_OBSTACLE]    = 'W',
        [CELL_TYPE_FRUIT]       = 'F',
        [CELL_TYPE_SNAKE]       = 'S'
    };

    return char_map[type];
}

void tftPrintTest(bool bg)
{
    float pi = 3.1415926f;
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

static void test(void)
{
    disp.setRotation(0);
    disp.setTextWrap(true);
    disp.setTextColor(RGB(31,31,31), RGB(0,0,0));
    disp.setCursor(0, 0);

    disp.fillScreen(RGB(0,0,0));
    disp.fillScreen(ST7735_BLACK);
    tftPrintTest(false);
}

bool diplay_st7735_init(int x, int y)
{
    ctx.x = x;
    ctx.y = y;
    disp.initR(INITR_BLACKTAB);
    test();
    return true;
}

bool display_st7735_update(void)
{

    return true;
}

bool display_st7735_draw_cell(int x, int y, enum cell_type type)
{

    return true;
}

bool display_st7735_display_score(int score)
{
    return true;
}