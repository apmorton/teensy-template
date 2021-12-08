//#define SERIAL_tt Serial // Send debug_tt output here. Must have SERIAL_tt.begin( ## )
//#include "debug_tt.h"
// Pin selections here are based on the original Adafruit Learning System
// guide for the Teensy 3.x project.  Some of these pin numbers don't even
// exist on the smaller SAMD M0 & M4 boards, so you may need to make other
// selections:

// GRAPHICS SETTINGS (appearance of eye) -----------------------------------

// If using a SINGLE EYE, you might want this next line enabled, which
// uses a simpler "football-shaped" eye that's left/right symmetrical.
// Default shape includes the caruncle, creating distinct left/right eyes.
// Otherwise your choice, standard is asymmetrical
  #define SYMMETRICAL_EYELID

// Enable ONE of these #includes -- HUGE graphics tables for various eyes:
#if( DISPLAY_SIZE == 240 )
#include "graphics/default_large.h"   // 240x240
#else
#include "graphics/defaultEye.h"      // Standard human-ish hazel eye -OR-
//#include "graphics/dragonEye.h"     // Slit pupil fiery dragon/demon eye -OR-
//#include "graphics/noScleraEye.h"   // Large iris, no sclera -OR-
//#include "graphics/goatEye.h"       // Horizontal pupil goat/Krampus eye -OR-
//#include "graphics/newtEye.h"       // Eye of newt -OR-
//#include "graphics/terminatorEye.h" // Git to da choppah!
//#include "graphics/catEye.h"        // Cartoonish cat (flat "2D" colors)
//#include "graphics/owlEye.h"        // Minerva the owl (DISABLE TRACKING)
//#include "graphics/naugaEye.h"      // Nauga googly eye (DISABLE TRACKING)
//#include "graphics/doeEye.h"        // Cartoon deer eye (DISABLE TRACKING)
//#include "graphics/Nebula.h"  //Dont work
//#include "graphics/MyEyeHuman1.h"
//#include "graphics/Human-HAL9000.h"
//#include "graphics/NebulaBlueGreen.h"
//#include "graphics/SpiralGalaxy.h"
//#include "graphics/ChameleonX_Eye.h"  //no work
//#include "graphics/MyEye.h"
#endif
// Optional: enable this line for startup logo (screen test/orient):
#if !defined(ADAFRUIT_HALLOWING)    // Hallowing can't always fit logo+eye
  #include "graphics/logo.h"        // Otherwise your choice, if it fits
#endif

// EYE LIST ----------------------------------------------------------------


// DISPLAY HARDWARE SETTINGS (screen type & connections) -------------------

  //#define TFT_SPI        SPI
  //#define TFT_PERIPH     PERIPH_SPI

  // Enable ONE of these #includes to specify the display type being used
  //#include <ST7735_t3.h> // TFT display library (enable one only)
  #include <ST7789_t3.h>
  
  #define SPI_FREQ 48000000    // TFT: use max SPI (clips to 12 MHz on M0)

// This table contains ONE LINE PER EYE.  The table MUST be present with
// this name and contain ONE OR MORE lines.  Each line contains THREE items:
// a pin number for the corresponding TFT/OLED display's SELECT line, a pin
// pin number for that eye's "wink" button (or -1 if not used), and a screen
// rotation value (0-3) for that eye.

eyeInfo_t eyeInfo[] = {
#ifdef ST77XX_ON_SPI_SPI1
  //CS  DC MOSI SCK RST WINK ROT INIT
  // Going to try to NO CS displays.
   {-1,  9, 11, 13,  8, -1,  0, INITR_144GREENTAB }, // LEFT EYE display-select and wink pins, no rotation
   {-1, 6, 26, 27, 7, -1,  0, INITR_144GREENTAB }, // RIGHT EYE display-select and wink pins, no rotation
#else
   {0,  2,  26, 27,  3, -1,  0, INITR_144GREENTAB }, // RIGHT EYE display-select and wink pins, no rotation
   {10, 9,  11, 13,  8, -1,  0, INITR_144GREENTAB }, // LEFT EYE display-select and wink pins, no rotation
#endif
};
// INPUT SETTINGS (for controlling eye motion) -----------------------------

// JOYSTICK_X_PIN and JOYSTICK_Y_PIN specify analog input pins for manually
// controlling the eye with an analog joystick.  If set to -1 or if not
// defined, the eye will move on its own.
// IRIS_PIN speficies an analog input pin for a photocell to make pupils
// react to light (or potentiometer for manual control).  If set to -1 or
// if not defined, the pupils will change on their own.
// BLINK_PIN specifies an input pin for a button (to ground) that will
// make any/all eyes blink.  If set to -1 or if not defined, the eyes will
// only blink if AUTOBLINK is defined, or if the eyeInfo[] table above
// includes wink button settings for each eye.

//#define JOYSTICK_X_PIN A0 // Analog pin for eye horiz pos (else auto)
//#define JOYSTICK_Y_PIN A1 // Analog pin for eye vert position (")
//#define JOYSTICK_X_FLIP   // If defined, reverse stick X axis
//#define JOYSTICK_Y_FLIP   // If defined, reverse stick Y axis


#define TRACKING            // If defined, eyelid tracks pupil
#define AUTOBLINK           // If defined, eyes also blink autonomously
  //#define BLINK_PIN         1 // Pin for manual blink button (BOTH eyes)
  //#define LIGHT_PIN      A3 // Photocell or potentiometer (else auto iris)
  //#define LIGHT_PIN_FLIP    // If defined, reverse reading from dial/photocell
  #define LIGHT_MIN       0 // Lower reading from sensor
  #define LIGHT_MAX    1023 // Upper reading from sensor

#define IRIS_SMOOTH         // If enabled, filter input from IRIS_PIN
#if !defined(IRIS_MIN)      // Each eye might have its own MIN/MAX
  #define IRIS_MIN      120 // Iris size (0-1023) in brightest light
#endif
#if !defined(IRIS_MAX)
  #define IRIS_MAX      900 // Iris size (0-1023) in darkest light
#endif
