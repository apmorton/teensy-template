/** @file display_console.h
 *  @brief DBG console display interface implementation.
 *
 *  @author Szymon (SP)
 *  @bug Doesnt work for any other XY than 10x10.
 *  @todo Prettify cell drawing implementation.
 *  @todo Support for other XY sizes than 10 x 10 fields.
 */

#ifndef DISPLAY_CONSOLE_H
#define DISPLAY_CONSOLE_H

#include <stdbool.h>

#include "cell.h"

bool diplay_console_init(int x, int y);

bool display_console_update(void);

bool display_console_draw_cell(int x, int y, enum cell_type type);

bool display_console_display_score(int score);

#endif /* DISPLAY_CONSOLE_H */
