#ifndef SNAKE_H
#define SNAKE_H

#include <stdbool.h>

struct snake_ctx;

int snake_get_len(struct snake_ctx *ctx);
bool snake_get_node(struct snake_ctx *ctx, int node_offset_from_head, int *x, int *y);

bool snake_move(struct snake_ctx *ctx, int change_x, int change_y, bool grow);

bool snake_reset(struct snake_ctx *ctx, int start_x, int start_y);
struct snake_ctx *snake_init(int snake_max_len);


#endif /* SNAKE_H */