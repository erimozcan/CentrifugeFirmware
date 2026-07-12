// D10 digital continuity test (Phase C wiring check, level-2).
//
// Drives D10 as a plain digital output, HIGH (3.3 V) then LOW (0 V) every 1.5 s.
// Purpose: give an unambiguous DC level to meter, isolating "is the pin/wire
// electrically alive and at 3.3 V" from any servo-PWM subtlety. Meter D10-to-GND:
// it should read ~3.3 V then ~0 V, flipping every 1.5 s. Then meter the same at the
// actuator's signal wire to prove the wire carries it end to end.
//
// Build/flash ONLY via its env (main firmware excluded):
//   pio run -e nano_pin_test -t upload
#ifdef LOCK_DIGITAL_TEST

#include <Arduino.h>

#include "config.h"

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(400);
  Serial.println();
  Serial.println("=== D10 DIGITAL TOGGLE TEST ===");
  Serial.print("Pin D10 / GPIO");
  Serial.println(PIN_LOCK_ACTUATOR);
  Serial.println("HIGH(3.3 V) / LOW(0 V) every 1.5 s. Meter D10 -> GND, then the");
  Serial.println("actuator signal wire -> GND. Both should read ~3.3 V then ~0 V.");
  pinMode(PIN_LOCK_ACTUATOR, OUTPUT);
}

void loop() {
  Serial.println("-> HIGH (3.3 V)");
  digitalWrite(PIN_LOCK_ACTUATOR, HIGH);
  delay(1500);

  Serial.println("-> LOW (0 V)");
  digitalWrite(PIN_LOCK_ACTUATOR, LOW);
  delay(1500);
}

#endif  // LOCK_DIGITAL_TEST
