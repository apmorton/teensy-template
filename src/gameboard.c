#include <stdlib.h>
#include "gameboard.h"
#include "dbg.h"

struct gameboard_ctx
{
    int x_size;
    int y_size;
    struct cell **cell;
};

static void clear_board(struct gameboard_ctx *ctx)
{
    int i = 0;
    int j = 0;
    for (i = 0; i < ctx->x_size; i++)
    {
        for (j = 0; j < ctx->y_size; j++)
        {
            ctx->cell[i][j].type = CELL_TYPE_EMPTY;
        }
    }
}

bool gameboard_clear_board(struct gameboard_ctx *ctx)
{
    clear_board(ctx);
    return true;
}

enum cell_type gameboard_cell_get(struct gameboard_ctx *ctx, int x, int y)
{
    if (x >= ctx->x_size || y >= ctx->y_size)
        return CELL_TYPE_ERROR;

    return ctx->cell[x][y].type;
}

bool gameboard_cell_set(struct gameboard_ctx *ctx, int x, int y, enum cell_type cell_type)
{
    if (x >= ctx->x_size || y >= ctx->y_size)
        return false;

    ctx->cell[x][y].type = cell_type;
    return true;
}

struct gameboard_ctx *gameboard_init(int x, int y)
{
    struct gameboard_ctx *ctx = calloc(sizeof(struct gameboard_ctx), 1);
    int i = 0;

    if (ctx == NULL)
        goto cleanup;

    ctx->x_size = x;
    ctx->y_size = y;

    int lenx = x * sizeof(*ctx->cell);
    int leny = y * sizeof(**ctx->cell); 
    LOG("lenx %d leny %d cellsize %d\r\n", lenx, leny, sizeof(struct cell));

    ctx->cell = malloc(lenx);
    if (ctx->cell == NULL)
    {
        LOG("Failed to malloc board\r\n");
        goto cleanup;
    }

    for (i = 0; i < x; i++)
    {
        ctx->cell[i] = malloc(leny);
        if (ctx->cell[i] == NULL)
            goto cleanup;
    }

    clear_board(ctx);
    return ctx;

    /* TODO: cleanup */
cleanup:
    LOG("ERROR: Failed to init board\r\n");
    return NULL;
}


