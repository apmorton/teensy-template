/** @file display_st7735.h
 *  @brief TFT display st7735 driver interface implementation.
 *
 *  @author Szymon (SP)
 *  @bug Doesnt work for any other XY than 10x10.
 *  @todo Prettify cell drawing implementation.
 *  @todo Support for other XY sizes than 10 x 10 fields.
 */

#ifndef DISPLAY_ST7735_H
#define DISPLAY_ST7735_H

#include <stdbool.h>
#include "cell.h"

#ifdef __cplusplus
extern "C"
{
#endif

bool diplay_st7735_init(int x, int y);

bool display_st7735_update(void);

bool display_st7735_draw_cell(int x, int y, enum cell_type type);

bool display_st7735_display_score(int score);


#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_ST7735_H */
