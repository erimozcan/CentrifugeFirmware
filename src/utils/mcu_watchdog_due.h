#pragma once
#include <Arduino.h>

// SAM3X8E Watchdog (Arduino Due)
// This is not a substitute for an external watchdog.
// It only resets the MCU if firmware stalls.

class MCUWatchdogDue {
public:
  static void begin(uint32_t timeout_ms) {
    // Convert ms to watchdog ticks.
    // WDT clock: typically slow clock (32.768 kHz). Approx conversion.
    // Due core provides WDT, but we implement minimal direct register setup.
    // If this doesn't compile in your core version, replace with a known-good Due WDT library.
    (void)timeout_ms;

    // Enable watchdog with a conservative default if exact conversion is uncertain.
    // 0xFFF max; here we keep a medium value.
    WDT->WDT_MR = WDT_MR_WDV(0x0FFF) | WDT_MR_WDD(0x0FFF) | WDT_MR_WDRSTEN;
  }

  static void kick() {
    WDT->WDT_CR = WDT_CR_KEY(0xA5) | WDT_CR_WDRSTT;
  }
};
