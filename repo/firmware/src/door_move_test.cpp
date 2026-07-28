// Isolated door CLOSED-LOOP move test (Phase B) — motor + hall sensors, NO FSM/ESC.
//
// This is the end-to-end "move the door until the OTHER signal is alive" behavior,
// proven in isolation before it goes anywhere near the state machine:
//   'o' -> drive toward OPEN,  auto-STOP the instant the OPEN hall reads active.
//   'c' -> drive toward CLOSED, auto-STOP the instant the CLOSED hall reads active.
// A DOOR_MOVE_TIMEOUT_MS watchdog stops the motor if the target hall never trips
// (matches the FSM's FAULT_DOOR_MOVE_TIMEOUT guard), so it can never run away.
//
// Drive recipe is the one proven in door_motor_test.cpp: a full-voltage digitalWrite
// KICK to break the N20 worm-gear stiction, then a slow 20 kHz PWM run. Kick + hall
// polling are NON-BLOCKING, so the auto-stop is honored even during the kick window.
//
// Direction mapping matches HardwareGuard::updateDoorMotor():
//   OPEN  -> IN1 HIGH / IN2 LOW      CLOSE -> IN1 LOW / IN2 HIGH.
//
// Hall polarity comes from config.h (DOOR_*_ACTIVE_LOW), the SAME source the FSM uses,
// so whatever we confirm here carries straight into the real firmware. Each print shows
// raw pin level AND interpreted active state so polarity can be re-verified at a glance.
//
// Nano ESP32 remap: analogWrite()/ledcDetachPin() take the RAW GPIO (no remap) -> use
// digitalPinToGPIONumber(); digitalWrite()/pinMode() take the Arduino pin (D6/D7).
//
// Build/flash:  pio run -e door_move_test -t upload      (then reflash nano_esp32 to restore)
// Serial (115200): o=open  c=close  x=stop  +/- = run duty +/-8  p=print
#ifdef DOOR_MOVE_TEST

#include <Arduino.h>

#include "config.h"

#if DOOR_VM_VOLTAGE > DOOR_MOTOR_RATED_V
#error "door_move_test kicks the door N20 at FULL VM (digitalWrite): forbidden while the DRV8871 VM rides the 24 V rail (DOOR_VM_VOLTAGE). Rewire VM to 12 V and set DOOR_VM_VOLTAGE=12 to bench-test."
#endif

enum MoveState : int8_t { IDLE = 0, MOVING_OPEN = -1, MOVING_CLOSE = 1 };

static int runDuty = 64;            // ~25% slow-run duty (tune with +/-); shared style w/ door_motor_test
static const int KICK_MS = 200;     // full-voltage kick to break worm-gear static friction
static int g1 = 0, g2 = 0;          // raw GPIOs for D6/D7 (analogWrite / ledcDetach)

static MoveState move = IDLE;
static uint32_t kickUntilMs = 0;    // full-drive until here, then drop to PWM
static uint32_t moveDeadlineMs = 0; // hard timeout for the whole move
static bool pwmApplied = false;
static uint32_t lastPrintMs = 0;

// active-low config -> pin reads LOW (0) when the door is AT that endpoint (magnet present).
static bool hallActive(uint8_t raw, bool activeLow) { return activeLow ? (raw == 0U) : (raw != 0U); }
static bool openActive()   { return hallActive(digitalRead(PIN_DOOR_OPEN_HALL)   == HIGH ? 1U : 0U, DOOR_OPEN_ACTIVE_LOW   != 0); }
static bool closedActive() { return hallActive(digitalRead(PIN_DOOR_CLOSED_HALL) == HIGH ? 1U : 0U, DOOR_CLOSED_ACTIVE_LOW != 0); }

// Guaranteed hard stop: release PWM and drive both DRV8871 inputs LOW.
static void hardStop() {
  move = IDLE;
  pwmApplied = false;
  ledcDetachPin(g1);
  ledcDetachPin(g2);
  pinMode(PIN_DOOR_DRV_IN1, OUTPUT);
  pinMode(PIN_DOOR_DRV_IN2, OUTPUT);
  digitalWrite(PIN_DOOR_DRV_IN1, LOW);
  digitalWrite(PIN_DOOR_DRV_IN2, LOW);
}

static void printState(const char *tag) {
  uint8_t oRaw = digitalRead(PIN_DOOR_OPEN_HALL);
  uint8_t cRaw = digitalRead(PIN_DOOR_CLOSED_HALL);
  Serial.print(tag);
  Serial.print("  move=");
  Serial.print(move == MOVING_OPEN ? "OPEN" : move == MOVING_CLOSE ? "CLOSE" : "IDLE");
  Serial.print("  openHall raw=");
  Serial.print(oRaw);
  Serial.print("/active=");
  Serial.print(openActive() ? 1 : 0);
  Serial.print("  closedHall raw=");
  Serial.print(cRaw);
  Serial.print("/active=");
  Serial.print(closedActive() ? 1 : 0);
  Serial.print("  duty=");
  Serial.print(runDuty);
  Serial.print(" (");
  Serial.print((runDuty * 100) / 255);
  Serial.println("%)");
}

// Begin a move: no-op if already at the target endpoint; else kick full then hand to PWM.
static void startMove(MoveState dir) {
  if (dir == MOVING_OPEN && openActive())   { Serial.println("already OPEN");   hardStop(); return; }
  if (dir == MOVING_CLOSE && closedActive()) { Serial.println("already CLOSED"); hardStop(); return; }

  hardStop();                 // clean slate: both LOW, PWM detached
  move = dir;
  pwmApplied = false;
  kickUntilMs = millis() + KICK_MS;
  moveDeadlineMs = millis() + DOOR_MOVE_TIMEOUT_MS;
  if (dir == MOVING_OPEN) digitalWrite(PIN_DOOR_DRV_IN1, HIGH);  // kick OPEN, IN2 stays LOW
  else                    digitalWrite(PIN_DOOR_DRV_IN2, HIGH);  // kick CLOSE, IN1 stays LOW
  printState(dir == MOVING_OPEN ? "-> OPEN" : "-> CLOSE");
}

// Non-blocking move service: honor the auto-stop / timeout every loop, incl. during the kick.
static void serviceMove() {
  if (move == IDLE) return;

  if (move == MOVING_OPEN && openActive())     { hardStop(); printState("reached OPEN");   return; }
  if (move == MOVING_CLOSE && closedActive())  { hardStop(); printState("reached CLOSED"); return; }

  if ((int32_t)(millis() - moveDeadlineMs) >= 0) { hardStop(); printState("TIMEOUT (no hall)"); return; }

  // Kick done -> switch the active input from full digitalWrite to the slow PWM run.
  if (!pwmApplied && (int32_t)(millis() - kickUntilMs) >= 0) {
    if (move == MOVING_OPEN) analogWrite(g1, runDuty);   // IN2 remains digitalWrite LOW
    else                     analogWrite(g2, runDuty);   // IN1 remains digitalWrite LOW
    pwmApplied = true;
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(400);
  Serial.println();
  Serial.println("=== DOOR MOVE TEST (PWM kick + hall auto-stop, NO FSM) ===");
  Serial.println("o=open  c=close  x=stop   +/- = run duty +/-8   p=print");
  Serial.print("polarity: OPEN active-low=");
  Serial.print(DOOR_OPEN_ACTIVE_LOW);
  Serial.print("  CLOSED active-low=");
  Serial.println(DOOR_CLOSED_ACTIVE_LOW);

  g1 = digitalPinToGPIONumber(PIN_DOOR_DRV_IN1);
  g2 = digitalPinToGPIONumber(PIN_DOOR_DRV_IN2);
  pinMode(PIN_DOOR_OPEN_HALL, INPUT_PULLUP);
  pinMode(PIN_DOOR_CLOSED_HALL, INPUT_PULLUP);
  analogWriteFrequency(20000);   // 20 kHz -> inaudible
  hardStop();                    // boot STOPPED
  printState("boot");
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'o') startMove(MOVING_OPEN);
    else if (c == 'c') startMove(MOVING_CLOSE);
    else if (c == 'x' || c == 's' || c == ' ') { hardStop(); printState("STOP"); }
    else if (c == '+') { runDuty += 8; if (runDuty > 255) runDuty = 255; if (move != IDLE && pwmApplied) analogWrite(move == MOVING_OPEN ? g1 : g2, runDuty); printState("duty+"); }
    else if (c == '-') { runDuty -= 8; if (runDuty < 0) runDuty = 0; if (move != IDLE && pwmApplied) analogWrite(move == MOVING_OPEN ? g1 : g2, runDuty); printState("duty-"); }
    else if (c == 'p') printState("print");
  }

  serviceMove();

  // Live hall telemetry at ~5 Hz so polarity + travel can be watched while stationary.
  if ((uint32_t)(millis() - lastPrintMs) >= 200U) {
    lastPrintMs = millis();
    if (move != IDLE) printState("...");
  }
}

#endif  // DOOR_MOVE_TEST
