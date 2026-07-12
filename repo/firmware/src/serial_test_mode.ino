#ifdef TEST_SERIAL

#include <Arduino.h>
#include "config.h"

// ============================================================================
// Centrifuge bench bring-up harness (TEST_SERIAL build only)
//
// Safety model:
//   - This sketch never runs the state machine, the ESC, or the lock.
//   - Every output starts and stays OFF until you explicitly command it.
//   - An idle auto-stop turns any energized motor off after a timeout, so
//     nothing can be left running unattended.
//   - The motor-enable / lock / ESC pins are never touched here.
//
// Subsystems wired in this build:
//   FAN  (DRV8871 #1): D5 (IN1, PWM speed) / D4 (IN2, held LOW), GND, 12 V.
//   DOOR (DRV8871 #2): D6 (IN1) / D7 (IN2), GND, motor supply.
//                      Slow jog both directions; no limit logic (door not
//                      mounted). Both pins driven via analogWrite so direction
//                      switching is clean on the SAM3X.
//   DOOR HALLS (3.3 V): D30 (open) / D31 (closed), read with INPUT_PULLUP and
//                       printed live (on change + in heartbeat).
//   DOOR ENCODER (XY-N20 quadrature, 3.3 V): C1->D8 (A), C2->D9 (B). Decoded in
//                       an ISR for position (count) + direction; velocity and
//                       raw A/B levels printed in the heartbeat. Encoder VCC
//                       MUST be 3.3 V (Due is not 5 V tolerant).
// ============================================================================

static const uint32_t kIdleTimeoutMs = 20000UL;  // auto-stop for any motor
static const uint32_t kHeartbeatMs = 5000UL;
static const uint8_t kDoorDuty = 90U;             // ~35% of 255: slow jog

enum DoorDir { DOOR_OFF = 0, DOOR_OPENING, DOOR_CLOSING };

static uint8_t g_fanDuty = 0U;
static uint32_t g_lastFanCmdMs = 0U;

static DoorDir g_doorDir = DOOR_OFF;
static uint32_t g_lastDoorCmdMs = 0U;

static uint32_t g_lastHeartbeatMs = 0U;
static int g_lastOpenRaw = -1;
static int g_lastClosedRaw = -1;

// Door encoder (quadrature). Updated in ISR; read with interrupts paused.
static volatile long g_encCount = 0;
static long g_lastEncCount = 0;       // snapshot at previous heartbeat
static uint32_t g_lastEncSampleMs = 0;

static const char *const kPinMap[] = {
  "PIN_MOTOR_ENABLE = D22 (NOT driven in this test)",
  "PIN_LOCK_ACTUATOR = D23 (NOT driven in this test)",
  "PIN_LOCK_SENSOR = D24 (NOT driven in this test)",
  "PIN_FAN_DRV_IN1 = D5 (ACTIVE - fan PWM)",
  "PIN_FAN_DRV_IN2 = D4 (ACTIVE - held LOW)",
  "PIN_DOOR_DRV_IN1 = D6 (ACTIVE - door)",
  "PIN_DOOR_DRV_IN2 = D7 (ACTIVE - door)",
  "PIN_DOOR_OPEN_HALL = D30 (ACTIVE - read)",
  "PIN_DOOR_CLOSED_HALL = D31 (ACTIVE - read)",
  "PIN_DOOR_ENC_A = D8 (ACTIVE - encoder C1)",
  "PIN_DOOR_ENC_B = D9 (ACTIVE - encoder C2)",
  "PIN_TEMP_NTC = A0 (NOT read in this test)",
  nullptr,
};

// Quadrature decode: interrupt on channel A edges, read B for direction.
// Sign is arbitrary until mapped to open/close by the o/c commands.
static void encoderISR() {
  bool a = digitalRead(PIN_DOOR_ENC_A);
  bool b = digitalRead(PIN_DOOR_ENC_B);
  if (a == b) {
    g_encCount++;
  } else {
    g_encCount--;
  }
}

static long readEncCount() {
  noInterrupts();
  long c = g_encCount;
  interrupts();
  return c;
}

static void printPinMap() {
  Serial.println("Current pin map:");
  for (const char *const *entry = kPinMap; *entry != nullptr; ++entry) {
    Serial.println(*entry);
  }
}

static void printHelp() {
  Serial.println("---- BENCH TEST COMMANDS ----");
  Serial.println("  FAN:  1=25%  2=50%  3=75%  4=100%   0=fan off");
  Serial.println("  DOOR: o=open(slow)  c=close(slow)   s=door stop");
  Serial.println("  ENC:  z=zero encoder count");
  Serial.println("  x  ALL OFF (fan + door)");
  Serial.println("  ?  help     m  pin map");
  Serial.print("Idle auto-stop after ");
  Serial.print(kIdleTimeoutMs / 1000UL);
  Serial.println(" s with no command.");
  Serial.println("-----------------------------");
}

static void applyFan(uint8_t duty) {
  digitalWrite(PIN_FAN_DRV_IN2, LOW);  // single direction
  analogWrite(PIN_FAN_DRV_IN1, duty);  // duty 0 => steady LOW
  g_fanDuty = duty;
  g_lastFanCmdMs = millis();
}

static void applyDoor(DoorDir dir) {
  // Both door pins are PWM-driven (analogWrite). duty 0 == steady LOW.
  switch (dir) {
    case DOOR_OPENING:
      analogWrite(PIN_DOOR_DRV_IN2, 0);
      analogWrite(PIN_DOOR_DRV_IN1, kDoorDuty);
      break;
    case DOOR_CLOSING:
      analogWrite(PIN_DOOR_DRV_IN1, 0);
      analogWrite(PIN_DOOR_DRV_IN2, kDoorDuty);
      break;
    case DOOR_OFF:
    default:
      analogWrite(PIN_DOOR_DRV_IN1, 0);
      analogWrite(PIN_DOOR_DRV_IN2, 0);
      break;
  }
  g_doorDir = dir;
  g_lastDoorCmdMs = millis();
}

static void allOff() {
  applyFan(0U);
  applyDoor(DOOR_OFF);
}

static const char *doorDirName(DoorDir d) {
  switch (d) {
    case DOOR_OPENING: return "OPENING";
    case DOOR_CLOSING: return "CLOSING";
    default:           return "OFF";
  }
}

static void reportFan() {
  uint16_t pct = (uint16_t)((g_fanDuty * 100U + 127U) / 255U);
  Serial.print("FAN: duty=");
  Serial.print(g_fanDuty);
  Serial.print("/255 (~");
  Serial.print(pct);
  Serial.println("%)");
}

static void printHalls(int openRaw, int closedRaw) {
  // Per config: DOOR_*_ACTIVE_LOW => a LOW reading means the magnet is present.
  bool openActive = (DOOR_OPEN_ACTIVE_LOW ? (openRaw == LOW) : (openRaw == HIGH));
  bool closedActive = (DOOR_CLOSED_ACTIVE_LOW ? (closedRaw == LOW) : (closedRaw == HIGH));
  Serial.print("HALL: open(D30)=");
  Serial.print(openRaw == HIGH ? "HIGH" : "LOW");
  Serial.print(openActive ? "[ACTIVE]" : "[----]");
  Serial.print("  closed(D31)=");
  Serial.print(closedRaw == HIGH ? "HIGH" : "LOW");
  Serial.println(closedActive ? "[ACTIVE]" : "[----]");
}

static void handleCommand(char c) {
  switch (c) {
    case '1': applyFan(64U);  Serial.println("CMD fan 25%");  reportFan(); break;
    case '2': applyFan(128U); Serial.println("CMD fan 50%");  reportFan(); break;
    case '3': applyFan(191U); Serial.println("CMD fan 75%");  reportFan(); break;
    case '4': applyFan(255U); Serial.println("CMD fan 100%"); reportFan(); break;
    case '0': applyFan(0U);   Serial.println("CMD fan OFF");  reportFan(); break;

    case 'o':
    case 'O': applyDoor(DOOR_OPENING); Serial.println("CMD door OPEN (slow)");  break;
    case 'c':
    case 'C': applyDoor(DOOR_CLOSING); Serial.println("CMD door CLOSE (slow)"); break;
    case 's':
    case 'S': applyDoor(DOOR_OFF);     Serial.println("CMD door STOP");         break;

    case 'z':
    case 'Z':
      noInterrupts();
      g_encCount = 0;
      interrupts();
      g_lastEncCount = 0;
      Serial.println("CMD encoder count zeroed");
      break;

    case 'x':
    case 'X': allOff(); Serial.println("CMD ALL OFF"); break;

    case '?':
    case 'h':
    case 'H': printHelp(); break;
    case 'm':
    case 'M': printPinMap(); break;
    default:
      Serial.print("Unknown command: '");
      Serial.print(c);
      Serial.println("'  (press ? for help)");
      break;
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  uint32_t startMs = millis();
  while (!Serial && millis() - startMs < 2000U) {
    delay(10);
  }

  // Outputs default to a safe OFF state before anything else.
  pinMode(PIN_FAN_DRV_IN1, OUTPUT);
  pinMode(PIN_FAN_DRV_IN2, OUTPUT);
  pinMode(PIN_DOOR_DRV_IN1, OUTPUT);
  pinMode(PIN_DOOR_DRV_IN2, OUTPUT);
  digitalWrite(PIN_FAN_DRV_IN1, LOW);
  digitalWrite(PIN_FAN_DRV_IN2, LOW);
  digitalWrite(PIN_DOOR_DRV_IN1, LOW);
  digitalWrite(PIN_DOOR_DRV_IN2, LOW);

  // Door Hall inputs (3.3 V), pulled up; sensors pull LOW when active.
  pinMode(PIN_DOOR_OPEN_HALL, INPUT_PULLUP);
  pinMode(PIN_DOOR_CLOSED_HALL, INPUT_PULLUP);

  // Door encoder inputs (3.3 V). Pull-ups are safe for push-pull or open-drain.
  pinMode(PIN_DOOR_ENC_A, INPUT_PULLUP);
  pinMode(PIN_DOOR_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_DOOR_ENC_A), encoderISR, CHANGE);
  g_lastEncSampleMs = millis();

  allOff();

  Serial.println();
  Serial.println("=== Centrifuge Due Bench Test: FAN + DOOR ===");
  Serial.println("Startup OK. Fan OFF, door OFF. ESC/lock/motor-enable NOT touched.");
  printPinMap();
  printHelp();
  printHalls(digitalRead(PIN_DOOR_OPEN_HALL), digitalRead(PIN_DOOR_CLOSED_HALL));

  g_lastHeartbeatMs = millis();
}

void loop() {
  // 1) Operator commands.
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n' || c == ' ') {
      continue;
    }
    handleCommand(c);
  }

  uint32_t now = millis();

  // 2) Live Hall edge reporting.
  int openRaw = digitalRead(PIN_DOOR_OPEN_HALL);
  int closedRaw = digitalRead(PIN_DOOR_CLOSED_HALL);
  if (openRaw != g_lastOpenRaw || closedRaw != g_lastClosedRaw) {
    g_lastOpenRaw = openRaw;
    g_lastClosedRaw = closedRaw;
    printHalls(openRaw, closedRaw);
  }

  // 3) Idle auto-stop: never leave a motor energized unattended.
  if (g_fanDuty > 0U && (uint32_t)(now - g_lastFanCmdMs) >= kIdleTimeoutMs) {
    applyFan(0U);
    Serial.println("AUTO-STOP: fan idle timeout, fan OFF.");
  }
  if (g_doorDir != DOOR_OFF && (uint32_t)(now - g_lastDoorCmdMs) >= kIdleTimeoutMs) {
    applyDoor(DOOR_OFF);
    Serial.println("AUTO-STOP: door idle timeout, door OFF.");
  }

  // 4) Heartbeat: fan + door dir + encoder (count, velocity, dir, raw A/B).
  if ((uint32_t)(now - g_lastHeartbeatMs) >= kHeartbeatMs) {
    g_lastHeartbeatMs = now;

    long count = readEncCount();
    long deltaCount = count - g_lastEncCount;
    uint32_t dtMs = now - g_lastEncSampleMs;
    long countsPerSec = (dtMs > 0U) ? (deltaCount * 1000L) / (long)dtMs : 0L;
    g_lastEncCount = count;
    g_lastEncSampleMs = now;

    const char *encDir = (deltaCount > 0) ? "+" : (deltaCount < 0) ? "-" : "0";

    Serial.print("alive  door=");
    Serial.print(doorDirName(g_doorDir));
    Serial.print("  ENC count=");
    Serial.print(count);
    Serial.print(" vel=");
    Serial.print(countsPerSec);
    Serial.print("c/s dir=");
    Serial.print(encDir);
    Serial.print(" A=");
    Serial.print(digitalRead(PIN_DOOR_ENC_A) == HIGH ? "1" : "0");
    Serial.print(" B=");
    Serial.print(digitalRead(PIN_DOOR_ENC_B) == HIGH ? "1" : "0");
    Serial.print("  ");
    reportFan();
  }
}

#endif  // TEST_SERIAL
