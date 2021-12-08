#include <stdlib.h>
#include <string.h>

#include "Arduino.h"

#include "display_console.h"

#include "dbg.h"

struct console_ctx
{
    int x;
    int y;
    char buff[10][10];
};

static struct console_ctx ctx = { 0 };

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

bool diplay_console_init(int x, int y)
{
    ctx.x = x;
    ctx.y = y;

    return true;
}

bool display_console_update(void)
{
    int i = 0; 

    LOG("\r\n\n");
    for (i = 0; i < ctx.y; i++)
    {
        LOG("%c%c%c%c%c%c%c%c%c%c\r\n", ctx.buff[i][0], ctx.buff[i][1], ctx.buff[i][2], ctx.buff[i][3], ctx.buff[i][4],
                                        ctx.buff[i][5], ctx.buff[i][6], ctx.buff[i][7], ctx.buff[i][8], ctx.buff[i][9]);
    }

    return true;
}

bool display_console_draw_cell(int x, int y, enum cell_type type)
{
    ctx.buff[x][y] = map_cell_type_to_char(type);
    return true;
}

bool display_console_display_score(int score)
{
    LOG("\r\n\n\t*******************************\r\n");
    LOG(      "\t*  Game Over                  *\r\n");
    LOG(      "\t*  Score %3d                  *\r\n", score);
    LOG(      "\t*  Press enter to start again *\r\n");
    LOG(      "\t*******************************\r\n");
    return true;
}
