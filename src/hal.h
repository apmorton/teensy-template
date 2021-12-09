/** @file hal.h
 *  @brief Teensy pin defs and pin setups.
 *
 *  @author Szymon (SP)
 *  @bug -
 *  @todo -
 */

#ifndef HAL_H
#define HAL_H

#include <stdbool.h>

#define HAL_KEYPAD_UP       4u
#define HAL_KEYPAD_DOWN     0u
#define HAL_KEYPAD_LEFT     1u
#define HAL_KEYPAD_RIGHT    3u
#define HAL_KEYPAD_MIDDLE   2u

#define TFT_SCLK 13  // SCLK can also use pin 14
#define TFT_MOSI 11  // MOSI can also use pin 7
#define TFT_CS   10  // CS & DC can use pins 2, 6, 9, 10, 15, 20, 21, 22, 23
#define TFT_DC    9  //  but certain pairs must NOT be used: 2+10, 6+9, 20+23, 21+22
#define TFT_RST   8  // RST can use any pin
#define TFT_BL    7  // backlight

bool hal_setup();

#endif /* HAL_H */
