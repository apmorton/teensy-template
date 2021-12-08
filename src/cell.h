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
//    uint8_t x;
//    uint8_t y;
    enum cell_type type;
};

#endif /* CELL_H */