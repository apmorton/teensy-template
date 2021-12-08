
#include <string.h>
#include "dbg.h"
#include "usb_serial.h"

void debug_print(const char *format, ...)
{
    char tmp[128] = { 0 };
    va_list args;
    va_start(args, format);
    vsnprintf(tmp, sizeof(tmp), format, args);
    va_end(args);

    usb_serial_write(tmp, strlen(tmp));
}