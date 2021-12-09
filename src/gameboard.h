/** @file gameboard.h
 *  @brief Gameboard XY cells container.
 *
 *  @author Szymon (SP)
 *  @bug -
 *  @todo - XY common interface???
 */

#ifndef GAMEBOARD_H
#define GAMEBOARD_H

#include <stdbool.h>
#include "cell.h"

struct gameboard_ctx;


enum cell_type gameboard_cell_get(struct gameboard_ctx *ctx, int x, int y);


bool gameboard_cell_set(struct gameboard_ctx *ctx, int x, int y, enum cell_type cell_type);


bool gameboard_clear_board(struct gameboard_ctx *ctx);


struct gameboard_ctx *gameboard_init(int x, int y);

#endif /* GAMEBOARD_H */