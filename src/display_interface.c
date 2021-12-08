#include "display_interface.h"

#include "display_console.h"

const struct display_interface console_iface = 
{
    .init = diplay_console_init,
    .update = display_console_update,
    .draw_cell = display_console_draw_cell,
    .display_score = display_console_display_score
};

const struct display_interface *get_interface(enum display_interface_type type)
{
    switch (type)
    {
        case DISPLAY_INTERFACE_TYPE_CONSOLE:
        return &console_iface;

        default:
        return &console_iface;
    }
}
