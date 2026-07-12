// Isolated lock-actuator bring-up test (Phase C wiring check).
//
// Purpose: drive ONLY the PQ12-R servo on D10 between its two endpoints, with no
// UI, no serial protocol, no state machine, no interlocks in the way. If the
// actuator sweeps in/out under this, the wiring (6 V + common GND + signal on D10),
// the pin mapping, and the ESP32Servo path are all proven good -> any "buttons don't
// move it" problem is upstream (firmware command path or UI). If it does NOT move
// here, the fault is hardware: power, ground, signal pin, or the servo itself.
//
// Build/flash ONLY via the dedicated env (main firmware is excluded there):
//   pio run -e nano_lock_test -t upload
//
// Guarded so it is inert if ever pulled into a normal build.
#ifdef LOCK_SWEEP_TEST

#include <Arduino.h>
#include <ESP32Servo.h>

#include "config.h"

static Servo lockServo;

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(400);
  Serial.println();
  Serial.println("=== LOCK SWEEP TEST ===");
  Serial.print("Servo signal pin: D10 / GPIO");
  Serial.println(PIN_LOCK_ACTUATOR);
  Serial.println("Sweeping 1000us <-> 2000us every 1.5 s. Watch the actuator.");
  Serial.println("Actuator MUST have: 6 V on V+, GND common with the Nano, signal on D10.");

  // ESP32Servo needs LEDC timers allocated before attach; do all four so it never
  // collides with another peripheral's timer.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  lockServo.setPeriodHertz(50);
  // Nano ESP32: ESP32Servo drives the RAW pin as a GPIO (no Arduino remap), so convert
  // D10 -> its true GPIO (21) or the pulse lands on GPIO10 and the actuator gets nothing.
  lockServo.attach(digitalPinToGPIONumber(PIN_LOCK_ACTUATOR), LOCK_PULSE_MIN_US, LOCK_PULSE_MAX_US);
}

void loop() {
  Serial.println("-> 1000 us  (fully retracted)");
  lockServo.writeMicroseconds(1000);
  delay(1500);

  Serial.println("-> 2000 us  (fully extended)");
  lockServo.writeMicroseconds(2000);
  delay(1500);
}

#endif  // LOCK_SWEEP_TEST
