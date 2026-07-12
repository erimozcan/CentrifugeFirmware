#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <Arduino.h>

#include "config.h"
#include "context.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(HAS_LED_STRIP) && HAS_LED_STRIP
#include <Adafruit_NeoPixel.h>
#endif

// Drives the WS2812 strip on PIN_LED_STRIP. Cosmetic only (kept out of the safety
// state machine): main.ino renders the current ctx LED fields each loop; this class
// throttles the actual strip.show() so it never runs every tick.
class LedController {
 public:
  void begin();
  // r/g/b + mode (LED_MODE_*). Call freely; internally rate-limited.
  void render(uint8_t r, uint8_t g, uint8_t b, uint8_t mode, uint32_t nowMs);

 private:
#if defined(ARDUINO_ARCH_ESP32) && defined(HAS_LED_STRIP) && HAS_LED_STRIP
  Adafruit_NeoPixel strip_{LED_COUNT, PIN_LED_STRIP, NEO_GRB + NEO_KHZ800};
#endif
  uint8_t lastR_ = 1;      // seeded != any real first command so the first solid
  uint8_t lastG_ = 1;      // frame always renders
  uint8_t lastB_ = 1;
  uint8_t lastMode_ = 255;
  uint16_t hue_ = 0;
  uint32_t lastFrameMs_ = 0;
  bool begun_ = false;
};

#endif  // LED_CONTROLLER_H
