#include "display_interface.h"

#include "display_console.h"
#include "display_st7735.h"

const struct display_interface console_iface = 
{
    .init = diplay_console_init,
    .update = display_console_update,
    .draw_cell = display_console_draw_cell,
    .display_score = display_console_display_score
};


const struct display_interface st7735_iface = 
{
    .init = diplay_st7735_init,
    .update = display_st7735_update,
    .draw_cell = display_st7735_draw_cell,
    .display_score = display_st7735_display_score
};


const struct display_interface *get_interface(enum display_interface_type type)
{
    switch (type)
    {
        case DISPLAY_INTERFACE_TYPE_CONSOLE:
        return &console_iface;

        case DISPLAY_INTERFACE_TYPE_ST7735_TFT:
        return &st7735_iface;

        default:
        return &console_iface;
    }
}
