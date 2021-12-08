#include <stdlib.h>
#include "snake.h"
#include "dbg.h"

struct node
{
    int x;
    int y;
};

struct snake_ctx
{
    int head;
    int tail;
    int occupied_len;
    int     nodes_cnt;
    struct  node *nodes;
};

/* ********************************************************************* */

static int saturate_change(int val)
{
    if (val == 0)
        return 0;

    if (val > 0)
        return 1;
    else
        return -1;
}

/* ********************************************************************* */

int snake_get_len(struct snake_ctx *ctx)
{
    if (ctx == NULL)
        return false;

    return ctx->occupied_len;
}

bool snake_get_node(struct snake_ctx *ctx, int node_offset_from_head, int *x, int *y)
{
    if (ctx == NULL || x == NULL || y == NULL)
        return false;

    if (node_offset_from_head >= ctx->occupied_len)
        return false;

    int node_idx = 0;
    if (ctx->head - node_offset_from_head < 0)
    {
        node_idx = ctx->nodes_cnt - (node_offset_from_head - ctx->head);
    }
    else
    {
        node_idx = ctx->head - node_offset_from_head;
    }

    *x = ctx->nodes[node_idx].x;
    *y = ctx->nodes[node_idx].y;
    return true;
}

bool snake_move(struct snake_ctx *ctx, int change_x, int change_y, bool grow)
{
    int next_head = (ctx->head + 1) % ctx->nodes_cnt;
    int next_tail = (ctx->tail + 1) % ctx->nodes_cnt;

    ctx->nodes[next_head].x = ctx->nodes[ctx->head].x + saturate_change(change_x);
    ctx->nodes[next_head].y = ctx->nodes[ctx->head].y + saturate_change(change_y);
    ctx->head = next_head;

    if (next_head == ctx->tail)
    {
        /* Little shortcut - tail just got overwritten by head, keep track of tail idx and do nothing more. */
        LOG("Snake maxed\r\n");
        ctx->tail = next_tail;
        return true;
    }

    if (!grow)
    {
        ctx->nodes[ctx->tail].x = 0;
        ctx->nodes[ctx->tail].y = 0;
        ctx->tail = next_tail;
    }
    else
    {
        if (ctx->occupied_len < ctx->nodes_cnt)
            ctx->occupied_len++;
    }

    return true;
}

bool snake_reset(struct snake_ctx *ctx, int start_x, int start_y)
{
    if (ctx == NULL)
        return false;

    int i = 0;
    for (i = 0; i < ctx->nodes_cnt; i++)
    {
        ctx->nodes[i].x = 0;
        ctx->nodes[i].y = 0;
    }

    ctx->head = 0;
    ctx->tail = 0;
    ctx->occupied_len = 1;
    ctx->nodes[0].x = start_x;
    ctx->nodes[0].y = start_y;

    return true;
}

struct snake_ctx *snake_init(int snake_max_len)
{
    if (snake_max_len < 1 || snake_max_len > 100)
        return NULL;

    struct snake_ctx *ctx = calloc(sizeof(*ctx), 1);

    if (ctx == NULL)
        goto cleanup;

    ctx->nodes = calloc(sizeof(*ctx->nodes), snake_max_len);

    if (ctx->nodes == NULL)
        goto cleanup;

    ctx->nodes_cnt = snake_max_len;

    snake_reset(ctx, 5, 5);
    return ctx;

    /* TODO: cleanup */
cleanup:
    LOG("ERROR: Failed to init snake\r\n");
    return NULL;
}

