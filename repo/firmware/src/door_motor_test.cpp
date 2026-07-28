// Isolated door-motor bring-up test (Phase B, motor only — door NOT attached).
//
// Reliable control: STOP and the startup KICK use digitalWrite (proven 100% on this
// motor in the full-drive test); only the slow *run* uses PWM (analogWrite). The hard
// stop always detaches PWM and digitalWrites both inputs LOW, so it can never run away.
//
// Nano ESP32 remap: analogWrite()/ledcDetachPin() take the RAW GPIO (no remap), so we
// use digitalPinToGPIONumber() for those; digitalWrite()/pinMode() take the Arduino pin
// (D6/D7) and remap themselves.
//
// Build/flash:  pio run -e door_test -t upload
// Serial (115200): f=forward  r=reverse  x=stop   +/- = run duty +/-8   p=print
#ifdef DOOR_MOTOR_TEST

#include <Arduino.h>

#include "config.h"

#if DOOR_VM_VOLTAGE > DOOR_MOTOR_RATED_V
#error "door_test kicks the door N20 at FULL VM (digitalWrite): forbidden while the DRV8871 VM rides the 24 V rail (DOOR_VM_VOLTAGE). Rewire VM to 12 V and set DOOR_VM_VOLTAGE=12 to bench-test."
#endif

static int runDuty = 64;         // ~25% run speed (target); tune with +/-
static int dir = 0;              // 1 fwd, -1 rev, 0 stop
static const int KICK_MS = 200;  // full-voltage kick to break worm-gear static friction
static int g1 = 0, g2 = 0;       // raw GPIOs for D6/D7 (for analogWrite / ledcDetach)

// Guaranteed hard stop: release PWM and drive both inputs LOW via digitalWrite.
static void hardStop() {
  dir = 0;
  ledcDetachPin(g1);
  ledcDetachPin(g2);
  pinMode(PIN_DOOR_DRV_IN1, OUTPUT);
  pinMode(PIN_DOOR_DRV_IN2, OUTPUT);
  digitalWrite(PIN_DOOR_DRV_IN1, LOW);
  digitalWrite(PIN_DOOR_DRV_IN2, LOW);
}

// Kick full (digitalWrite) to start, then settle the active input to the slow run duty.
static void startDir(int d) {
  hardStop();          // clean slate: both LOW, PWM detached
  dir = d;
  if (d == 1) digitalWrite(PIN_DOOR_DRV_IN1, HIGH);   // kick forward, IN2 stays LOW
  else        digitalWrite(PIN_DOOR_DRV_IN2, HIGH);   // kick reverse, IN1 stays LOW
  delay(KICK_MS);
  if (d == 1) analogWrite(g1, runDuty);               // slow run on active pin (PWM)
  else        analogWrite(g2, runDuty);               // inactive pin remains digitalWrite LOW
}

static void report() {
  Serial.print("dir=");
  Serial.print(dir == 1 ? "FWD" : dir == -1 ? "REV" : "STOP");
  Serial.print("  runDuty=");
  Serial.print(runDuty);
  Serial.print(" (");
  Serial.print((runDuty * 100) / 255);
  Serial.println("%)");
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(400);
  Serial.println();
  Serial.println("=== DOOR MOTOR TEST v2 (digitalWrite kick+stop, PWM slow run) ===");
  Serial.println("f=forward  r=reverse  x=stop   +/- = run duty +/-8");
  g1 = digitalPinToGPIONumber(PIN_DOOR_DRV_IN1);
  g2 = digitalPinToGPIONumber(PIN_DOOR_DRV_IN2);
  analogWriteFrequency(20000);   // 20 kHz -> inaudible
  hardStop();                    // boot STOPPED
  report();
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'f') { startDir(1); report(); }
    else if (c == 'r') { startDir(-1); report(); }
    else if (c == 'x' || c == 's' || c == ' ') { hardStop(); report(); }
    else if (c == '+') { runDuty += 8; if (runDuty > 255) runDuty = 255; if (dir) startDir(dir); report(); }
    else if (c == '-') { runDuty -= 8; if (runDuty < 0) runDuty = 0; if (dir) startDir(dir); report(); }
    else if (c == 'p') { report(); }
  }
}

#endif  // DOOR_MOTOR_TEST
