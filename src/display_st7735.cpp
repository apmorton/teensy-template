#include <stdlib.h>
#include <string.h>
#include "Arduino.h"
#include "display_st7735.h"
#include "ST7735_t3.h"
#include "hal.h"
#include "dbg.h"

#define XY_SIZE_SUPPORTED   (10)

#define RGB(r,g,b) (b<<11|g<<6|r)
#define SETCOLOR(c) disp.setTextColor(c, bg ? ST7735_BLACK : c);

ST7735_t3 disp = ST7735_t3(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

struct st7735_ctx
{
    int x;
    int y;
    char buff[XY_SIZE_SUPPORTED][XY_SIZE_SUPPORTED];
};

static struct st7735_ctx ctx = { 0 };

static char map_cell_type_to_char(enum cell_type type)
{
    const char char_map[] = 
    {
        [CELL_TYPE_EMPTY]       = ' ',
        [CELL_TYPE_OBSTACLE]    = '#',
        [CELL_TYPE_FRUIT]       = '+',
        [CELL_TYPE_SNAKE]       = 'O'
    };

    return char_map[type];
}

bool diplay_st7735_init(int x, int y)
{
    if (x != XY_SIZE_SUPPORTED || y != XY_SIZE_SUPPORTED)
        return false;

    ctx.x = x;
    ctx.y = y;
    disp.initR(INITR_BLACKTAB);
    disp.setRotation(2);
    disp.setTextColor(RGB(31,31,31), RGB(0,0,0));
    disp.fillScreen(ST7735_BLACK);
    disp.setCursor(0, 0);
    disp.setTextSize(1);
    disp.setTextWrap(false);

    return true;
}

bool display_st7735_update(void)
{
    int i = 0; 
    char line[32] = { 0 };

    disp.setTextSize(2);
    disp.setCursor(0, 0);

    for (i = 0; i < ctx.y; i++)
    {
        sprintf(line, "%c%c%c%c%c%c%c%c%c%c", ctx.buff[i][0], ctx.buff[i][1], ctx.buff[i][2], ctx.buff[i][3], ctx.buff[i][4],
                                              ctx.buff[i][5], ctx.buff[i][6], ctx.buff[i][7], ctx.buff[i][8], ctx.buff[i][9]);
        disp.println(line);
    }

    return true;
}

bool display_st7735_draw_cell(int x, int y, enum cell_type type)
{
    ctx.buff[x][y] = map_cell_type_to_char(type);

    return true;
}

bool display_st7735_display_score(int score)
{
    disp.fillScreen(ST7735_WHITE);
    disp.setTextColor(RGB(0,0,0), RGB(31,31,31));
    disp.setCursor(1, 1);
    disp.setTextSize(2);

    disp.println("\n Game Over   \n");
    disp.print(  " Score "); disp.print(score); disp.println("\n");
    disp.println(" Press \n enter\n");

    disp.setTextSize(1);
    disp.setCursor(0, 140);
    disp.println(" Exclusively for \n Conclusive by SP.");


    disp.setTextColor(RGB(31,31,31), RGB(0,0,0));

    return true;
}