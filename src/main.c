#include "WProgram.h"
#include <stdbool.h>

#include "usb_serial.h"
#include "game.h"
#include "utils.h"
#include "hal.h"
#include "dbg.h"

static void test(void)
{
    pinMode(13, OUTPUT);
    while (1)
    {
        digitalWriteFast(13, HIGH);
        delay(500);
        digitalWriteFast(13, LOW);
        delay(500);
    }
}

static enum game_input_key map_key_to_game_input(int key_pin)
{
  static const enum game_input_key key_to_game_lut[] =
  {
    [HAL_KEYPAD_UP] = GAME_INPUT_KEY_UP,
    [HAL_KEYPAD_DOWN] = GAME_INPUT_KEY_DOWN,
    [HAL_KEYPAD_LEFT] = GAME_INPUT_KEY_LEFT,
    [HAL_KEYPAD_RIGHT] = GAME_INPUT_KEY_RIGHT,
    [HAL_KEYPAD_MIDDLE] = GAME_INPUT_KEY_ENTER
  };

  return key_to_game_lut[key_pin];
}

static int pool_keys(void)
{
    static unsigned long input_pooling_timestamp = 0;

    if (!wait_to_exec(&input_pooling_timestamp, 50))
        return -1;

    int key = -1;
    int i = 0;
    for (; i < 5; i++)
    {
        if (!digitalRead(i))
        {
            key = i;
            break;
        }
    }

    return key;
}

static int find_random(int min, int max)
{
    return (int)random_C(min, max);
}

static unsigned long get_ms_tick(void)
{
    return millis();
}

struct game_ctx *game = NULL;

void setup(void)
{
    delay(3500);
    LOG("Started %s %s\r\n", __DATE__, __TIME__);
    if (hal_setup())
    {
        LOG("All set up\r\n");
    }

    uint32_t seed = analogRead(A13) * analogRead(A11);
    LOG("Rand seed %u\n", seed);
    randomSeed_C(seed);

    game = game_init(find_random, get_ms_tick);
    if (game == NULL)
        test();

    LOG("game initialized\n");
}

int main(void)
{    // To use Teensy 3.0 without Arduino, simply put your code here.    // For example:
#ifdef USING_MAKEFILE
    setup();
 
    while (1) 
    {
        int key = pool_keys();
        if (key != -1)
            game_input(game, map_key_to_game_input(key));

        game_process(game);

        delay(1);
    }

#else
	// Arduino's main() function just calls setup() and loop()....
	while (1) {
		loop();
		yield();
	}
#endif
}

