#include <Arduino.h>
#include "hal.h"

bool hal_setup(void)
{
    /* Give some time for usb cdc to come up in host. */
    delay(1000);

    /* input pins */
    pinMode(HAL_KEYPAD_UP, INPUT_PULLUP);
    pinMode(HAL_KEYPAD_DOWN, INPUT_PULLUP);
    pinMode(HAL_KEYPAD_LEFT, INPUT_PULLUP);
    pinMode(HAL_KEYPAD_RIGHT, INPUT_PULLUP);
    pinMode(HAL_KEYPAD_MIDDLE, INPUT_PULLUP);

    /* LCD TFT */


    return true;
}
