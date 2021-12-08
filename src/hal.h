#ifndef HAL_H
#define HAL_H

#include <stdbool.h>

#define HAL_KEYPAD_UP       4u
#define HAL_KEYPAD_DOWN     0u
#define HAL_KEYPAD_LEFT     1u
#define HAL_KEYPAD_RIGHT    3u
#define HAL_KEYPAD_MIDDLE   2u

bool hal_setup();

#endif /* HAL_H */