// Door-motor FULL-DRIVE test (digitalWrite, no PWM) — isolates PWM-path vs hardware.
//
// Drives the DRV8871 IN1/IN2 (D6/D7) with plain digitalWrite at full 12 V, auto-cycling
// FWD -> stop -> REV -> stop. This is exactly what the real firmware's updateDoorMotor()
// does, and digitalWrite applies the Arduino pin-remap (D6->GPIO9, D7->GPIO10) correctly.
//   - Motor runs here but not under PWM  => the LEDC/PWM path is the problem.
//   - Motor doesn't run here either      => hardware: VM (12 V), common ground between the
//                                           DRV8871 and the Nano, motor on OUT1/OUT2, or IN wiring.
//
// Build/flash ONLY via its env:  pio run -e door_digi_test -t upload
#ifdef DOOR_DIGITAL_TEST

#include <Arduino.h>

#include "config.h"

#if DOOR_VM_VOLTAGE > DOOR_MOTOR_RATED_V
#error "door_digi_test drives the door N20 at FULL VM: forbidden while the DRV8871 VM rides the 24 V rail (DOOR_VM_VOLTAGE). Rewire VM to 12 V and set DOOR_VM_VOLTAGE=12 to bench-test."
#endif

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(400);
  Serial.println();
  Serial.println("=== DOOR MOTOR FULL-DRIVE TEST (D6/D7, digitalWrite, 12 V) ===");
  Serial.println("Auto: FWD 3s -> stop 1s -> REV 3s -> stop 1s, repeating.");
  Serial.println("Check: DRV8871 VM=12V, GND common with Nano, motor on OUT1/OUT2.");
  pinMode(PIN_DOOR_DRV_IN1, OUTPUT);
  pinMode(PIN_DOOR_DRV_IN2, OUTPUT);
  digitalWrite(PIN_DOOR_DRV_IN1, LOW);
  digitalWrite(PIN_DOOR_DRV_IN2, LOW);
}

void loop() {
  Serial.println("-> FORWARD (IN1=HIGH IN2=LOW)");
  digitalWrite(PIN_DOOR_DRV_IN1, HIGH);
  digitalWrite(PIN_DOOR_DRV_IN2, LOW);
  delay(3000);

  Serial.println("-> STOP");
  digitalWrite(PIN_DOOR_DRV_IN1, LOW);
  digitalWrite(PIN_DOOR_DRV_IN2, LOW);
  delay(1000);

  Serial.println("-> REVERSE (IN1=LOW IN2=HIGH)");
  digitalWrite(PIN_DOOR_DRV_IN1, LOW);
  digitalWrite(PIN_DOOR_DRV_IN2, HIGH);
  delay(3000);

  Serial.println("-> STOP");
  digitalWrite(PIN_DOOR_DRV_IN1, LOW);
  digitalWrite(PIN_DOOR_DRV_IN2, LOW);
  delay(1000);
}

#endif  // DOOR_DIGITAL_TEST
