#include <stdbool.h>
#include <stddef.h>

#include "game.h"
#include "gameboard.h"
#include "snake.h"
#include "dbg.h"
#include "display_interface.h"
#include "utils.h"

#define BOARD_X_SIZE    (10)
#define BOARD_Y_SIZE    (10)

struct game_ctx
{
    bool init;

    enum game_input_key previous_move;
    enum game_input_key next_move;
    int change_x;
    int change_y;
    int fruit_x;
    int fruit_y;
    int score;
    bool game_over;
    unsigned long process_timestamp;

    struct snake_ctx *snake;
    struct gameboard_ctx *board;
    const struct display_interface *display;
    rand_clbk rand;
    get_ms_clbk get_ms;
};

/* temporary game singleton */
struct game_ctx game_ctx;

/* ********************************************************************* */

static bool is_opposite(enum game_input_key key_one, enum game_input_key key_two)
{
    if (key_one == GAME_INPUT_KEY_UP && key_two == GAME_INPUT_KEY_DOWN)
        return true;

    if (key_one == GAME_INPUT_KEY_DOWN && key_two == GAME_INPUT_KEY_UP)
        return true;

    if (key_one == GAME_INPUT_KEY_LEFT && key_two == GAME_INPUT_KEY_RIGHT)
        return true;

    if (key_one == GAME_INPUT_KEY_RIGHT && key_two == GAME_INPUT_KEY_LEFT)
        return true;

    return false;
}

static bool display_sync(struct game_ctx *ctx)
{
    int i = 0;
    int j = 0;

    for (i = 0; i < BOARD_X_SIZE; i++)
    {
        for (j = 0; j < BOARD_Y_SIZE; j++)
        {
            ctx->display->draw_cell(i, j, gameboard_cell_get(ctx->board, i, j));
        }
    }

    return true;
}

static bool reset_board(struct game_ctx *ctx)
{
    gameboard_clear_board(ctx->board);

    int i = 0;
    for (i = 0; i < BOARD_X_SIZE; i++)
    {
        gameboard_cell_set(ctx->board, i, 0, CELL_TYPE_OBSTACLE);
        gameboard_cell_set(ctx->board, i, BOARD_Y_SIZE - 1, CELL_TYPE_OBSTACLE);
    }

    for (i = 0; i < BOARD_Y_SIZE; i++)
    {
        gameboard_cell_set(ctx->board, 0, i, CELL_TYPE_OBSTACLE);
        gameboard_cell_set(ctx->board, BOARD_X_SIZE - 1, i, CELL_TYPE_OBSTACLE);
    }

    return true;
}

static bool sync_board(struct game_ctx *ctx)
{
    reset_board(ctx);
    gameboard_cell_set(ctx->board, ctx->fruit_x, ctx->fruit_y, CELL_TYPE_FRUIT);

    int snake_len = snake_get_len(ctx->snake);
    int curr_len = snake_len;
    int x = 0;
    int y = 0;

    while (curr_len)
    {
        snake_get_node(ctx->snake, snake_len - curr_len, &x, &y);
        gameboard_cell_set(ctx->board, x, y, CELL_TYPE_SNAKE);
        curr_len--;
    }

    return true;
}

static bool place_fruit(struct game_ctx *ctx)
{
    while(1)
    {
        /* Rand bounds help to avoid board walls. */
        int x = ctx->rand(1, BOARD_X_SIZE - 1); 
        int y = ctx->rand(1, BOARD_Y_SIZE - 1);
        
        /* Rand and pray to find empty cell. 
           TODO: optimization */
        if (gameboard_cell_get(ctx->board, x, y) == CELL_TYPE_EMPTY)
        {
            gameboard_cell_set(ctx->board, x, y, CELL_TYPE_FRUIT);
            ctx->fruit_x = x;
            ctx->fruit_y = y;
            break;
        }
    }

    return true;
}

static bool reset_game(struct game_ctx *ctx)
{
    ctx->previous_move = GAME_INPUT_NO_KEY;
    ctx->next_move = GAME_INPUT_NO_KEY;
    ctx->change_x = 0;
    ctx->change_y = 0;
    ctx->score = 0;
    ctx->game_over = false;
    ctx->process_timestamp = ctx->get_ms();

    reset_board(ctx);
    snake_reset(ctx->snake, BOARD_X_SIZE / 2, BOARD_Y_SIZE / 2);
    place_fruit(ctx);
    /* stim */
    game_input(ctx, GAME_INPUT_KEY_DOWN);

    return true;
}

/* ********************************************************************* */
#define CALC_INTERVAL(_difficulty_level)  (1000 - ((_difficulty_level) * 25))

bool game_process(struct game_ctx *ctx)
{
    if (ctx->game_over)
        return false;

    unsigned long interval = CALC_INTERVAL(ctx->score);
    if (!wait_to_exec(&ctx->process_timestamp, (interval < 200) ? (200) : (interval)))
        return false;

    int x = 0;
    int y = 0;
    snake_get_node(ctx->snake, 0, &x, &y);
    x += ctx->change_x;
    y += ctx->change_y;

    if (gameboard_cell_get(ctx->board, x, y) == CELL_TYPE_OBSTACLE ||
        gameboard_cell_get(ctx->board, x, y) == CELL_TYPE_SNAKE)
    {
        LOG("GAME_OVER\r\nScore: %d", ctx->score);
        ctx->game_over = true;
        ctx->display->display_score(ctx->score);
        return false;
    }
    else if (gameboard_cell_get(ctx->board, x, y) == CELL_TYPE_EMPTY || 
             gameboard_cell_get(ctx->board, x, y) == CELL_TYPE_FRUIT)
    {
        bool is_fruit_eaten = gameboard_cell_get(ctx->board, x, y) == CELL_TYPE_FRUIT;
        snake_move(ctx->snake, ctx->change_x, ctx->change_y, is_fruit_eaten);
        if (is_fruit_eaten)
        {
            ctx->score++;
            place_fruit(ctx);
        }
    }

    ctx->previous_move = ctx->next_move;

    sync_board(ctx);
    display_sync(ctx);
    ctx->display->update();

    return true;
}

bool game_input(struct game_ctx *ctx, enum game_input_key key)
{
    if(is_opposite(ctx->previous_move, key))
    {
        return false;
    }

    ctx->next_move = key;
    ctx->change_y = 0;
    ctx->change_x = 0;

    switch (key)
    {
        case GAME_INPUT_KEY_UP:
            ctx->change_x = -1;
            break;
        case GAME_INPUT_KEY_DOWN:
            ctx->change_x = 1;
            break;
        case GAME_INPUT_KEY_LEFT:
            ctx->change_y = -1;
            break;
        case GAME_INPUT_KEY_RIGHT:
            ctx->change_y = 1;
            break;
        case GAME_INPUT_KEY_ENTER:
            reset_game(ctx);
            break;
        default:
            LOG("ERROR: game input unhandled key %d\r\n", key);
            return false;
    }

    return true;
}

struct game_ctx *game_init(rand_clbk rand, get_ms_clbk get_ms)
{
    if (game_ctx.init)
        return NULL;

    if (rand == NULL || get_ms == NULL)
        return NULL;

    game_ctx.rand = rand;
    game_ctx.get_ms = get_ms;

    game_ctx.board = gameboard_init(BOARD_X_SIZE, BOARD_Y_SIZE);
    if (game_ctx.board == NULL)
    {
        LOG("Failed to init board\r\n");
        return NULL;
    }

    game_ctx.snake = snake_init(10);
    if (game_ctx.snake == NULL)
    {
        LOG("Failed to init snake\r\n");
        return NULL;
    }

    game_ctx.display = get_interface(DISPLAY_INTERFACE_TYPE_ST7735_TFT);
    if (game_ctx.display == NULL)
    {
        LOG("Unsupported display chosen\r\n");
        return NULL;
    }

    if (!game_ctx.display->init(BOARD_X_SIZE, BOARD_Y_SIZE))
    {
        LOG("Failed to init display\r\n");
        return NULL;
    }

    reset_game(&game_ctx);
    sync_board(&game_ctx);
    display_sync(&game_ctx);
    game_ctx.display->update();

    game_ctx.init = true;
    return &game_ctx;
    /* TODO: clenaup */
}
