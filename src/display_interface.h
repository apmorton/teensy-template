/** @file display_interface.h
 *  @brief Interface methods placeholder and getter.
 *
 *  This exhibits interface needed to display a 
 *  XY board game.
 *
 *  @author Szymon (SP)
 *  @bug No known bugs.
 *  @todo -
 */

#ifndef DISPLAY_INTERFACE_H
#define DISPLAY_INTERFACE_H

#include <stdbool.h>

#include "cell.h"

enum display_interface_type
{
    DISPLAY_INTERFACE_TYPE_CONSOLE = 0,
    DISPLAY_INTERFACE_TYPE_ST7735_TFT,
    DISPLAY_INTERFACE_TYPE_UNKNOWN
};

struct display_interface
{
    bool (*init)(int x, int y);
    bool (*update)(void);
    bool (*draw_cell)(int x, int y, enum cell_type type);
    bool (*display_score)(int score);
};

const struct display_interface *get_interface(enum display_interface_type type);

#endif /* DISPLAY_INTERFACE_H */
