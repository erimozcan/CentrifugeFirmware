#include "led_controller.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(HAS_LED_STRIP) && HAS_LED_STRIP

void LedController::begin() {
  // CRITICAL (Nano ESP32): with the default Arduino pin-remap, PIN_LED_STRIP (D11)
  // is the *logical* pin index 11, and the core converts it to GPIO38 only inside
  // Arduino APIs (digitalWrite/Serial/etc). Adafruit_NeoPixel's ESP32 RMT path drives
  // the raw pin value, so we must hand it the true GPIO or it toggles GPIO11 and the
  // strip on the D11 pad stays dark. digitalPinToGPIONumber() does the conversion
  // (and is a no-op identity when the remap is off), so this is correct either way.
  strip_.setPin(digitalPinToGPIONumber(PIN_LED_STRIP));
  strip_.begin();
  strip_.clear();
  strip_.show();
  begun_ = true;
}

void LedController::render(uint8_t r, uint8_t g, uint8_t b, uint8_t mode, uint32_t nowMs) {
  if (!begun_) {
    return;
  }

  if (mode == LED_MODE_RAINBOW) {
    // Animate at ~50 Hz; full hue sweep every ~5 s.
    if (static_cast<uint32_t>(nowMs - lastFrameMs_) < 20U) {
      return;
    }
    lastFrameMs_ = nowMs;
    hue_ += 256;
    for (uint16_t i = 0; i < LED_COUNT; ++i) {
      uint16_t pixelHue = static_cast<uint16_t>(hue_ + (i * 65536UL / LED_COUNT));
      strip_.setPixelColor(i, strip_.gamma32(strip_.ColorHSV(pixelHue)));
    }
    strip_.show();
    lastMode_ = mode;
    return;
  }

  // Solid: only push to the strip when the color/mode actually changed, so we are not
  // calling show() (which stalls the RMT transfer) on every loop iteration.
  if (mode == lastMode_ && r == lastR_ && g == lastG_ && b == lastB_) {
    return;
  }
  lastMode_ = mode;
  lastR_ = r;
  lastG_ = g;
  lastB_ = b;

  uint32_t color = strip_.Color(r, g, b);
  for (uint16_t i = 0; i < LED_COUNT; ++i) {
    strip_.setPixelColor(i, color);
  }
  strip_.show();
}

#else  // no strip on this target: no-op stubs keep the Due build linking

void LedController::begin() {}
void LedController::render(uint8_t, uint8_t, uint8_t, uint8_t, uint32_t) {}

#endif
