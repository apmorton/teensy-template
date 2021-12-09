/** @file cell.h
 *  @brief Cell type common definition.
 *
 *  @author Szymon (SP)
 *  @bug No known bugs.
 *  @todo add snake head with direction
 */
#ifndef CELL_H
#define CELL_H

#include <stdint.h>

enum cell_type
{
    CELL_TYPE_EMPTY = 0,
    CELL_TYPE_OBSTACLE,
    CELL_TYPE_FRUIT,
    CELL_TYPE_SNAKE,

    CELL_TYPE_ERROR = 0xff
};

struct cell
{
    enum cell_type type;
};

#endif /* CELL_H */