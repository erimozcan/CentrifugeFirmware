#include "hardware_guard.h"

#include "config.h"

void HardwareGuard::begin() {
  pinMode(PIN_MOTOR_ENABLE, OUTPUT);
#if defined(ARDUINO_ARCH_ESP32) && defined(LOCK_IS_SERVO)
  // PQ12-R lock is an RC servo: drive it with a 50 Hz pulse, start unlocked.
  // ESP32Servo needs an LEDC timer allocated before attach() or the pin never
  // outputs a pulse (this is why the isolated sweep test moved and an un-allocated
  // attach would not). Door/fan use plain digitalWrite, so timer 0 is free here.
  ESP32PWM::allocateTimer(0);
  lockServo_.setPeriodHertz(50);
  // Nano ESP32 pin-remap: ESP32Servo -> ledcAttachPin() uses the RAW pin value as the
  // GPIO (no Arduino remap), so hand it the true GPIO or the pulse comes out on GPIO10
  // instead of the D10 pad (GPIO21). Same class of bug as the WS2812 strip on D11.
  lockServo_.attach(digitalPinToGPIONumber(PIN_LOCK_ACTUATOR), LOCK_PULSE_MIN_US, LOCK_PULSE_MAX_US);
  // Boot RETRACTED: after a mid-cycle brownout reboot the gantry can be anywhere -- a
  // blind boot-extend once drove the pin into a bucket. The state machine re-engages
  // within ~1 s, as soon as the ESC's angle telemetry verifies a detent under the pin.
  lockServo_.writeMicroseconds(LOCK_PULSE_UNLOCKED_US);
#else
  pinMode(PIN_LOCK_ACTUATOR, OUTPUT);
#endif
  pinMode(PIN_LOCK_SENSOR, INPUT_PULLUP);
  pinMode(PIN_FAN_DRV_IN1, OUTPUT);
  pinMode(PIN_FAN_DRV_IN2, OUTPUT);
  pinMode(PIN_DOOR_OPEN_HALL, INPUT_PULLUP);
  pinMode(PIN_DOOR_CLOSED_HALL, INPUT_PULLUP);

  motorEnable_ = false;
  lockActuator_ = false;  // boot retracted; engaged only once the detent is verified
  fanEnabled_ = false;
  doorMotorCommand_ = DOOR_MOTOR_STOP;
  doorKicking_ = false;
  doorKickUntilMs_ = 0U;
  doorAppliedDuty_ = 0U;   // 0 forces the first analogWrite even at duty 0

  digitalWrite(PIN_MOTOR_ENABLE, LOW);
#if !(defined(ARDUINO_ARCH_ESP32) && defined(LOCK_IS_SERVO))
  digitalWrite(PIN_LOCK_ACTUATOR, LOW);   // servo target set above on the Nano
#endif
  digitalWrite(PIN_FAN_DRV_IN1, LOW);
  digitalWrite(PIN_FAN_DRV_IN2, LOW);

  // Door DRV8871 PWM via analogWrite. CRITICAL (Nano ESP32): analogWrite / ledcAttachPin /
  // ledcDetachPin are ALREADY remapped by the core (io_pin_remap.h wraps them in
  // digitalPinToGPIONumber), exactly like digitalWrite/pinMode. So pass the LOGICAL pin
  // (D6/D7 = PIN_DOOR_DRV_IN1/IN2) -- NOT digitalPinToGPIONumber(), which double-remaps and
  // sends the PWM to the wrong pad (that was why the run speed never changed).
#if defined(ARDUINO_ARCH_ESP32)
  analogWriteFrequency(DOOR_PWM_FREQ_HZ);   // 20 kHz -> inaudible
#endif
  pinMode(PIN_DOOR_DRV_IN1, OUTPUT);
  pinMode(PIN_DOOR_DRV_IN2, OUTPUT);
  digitalWrite(PIN_DOOR_DRV_IN1, LOW);
  digitalWrite(PIN_DOOR_DRV_IN2, LOW);
}

void HardwareGuard::updateMotorEnable(SystemState state) {
  bool enable = isMotorEnabledState(state);
  motorEnable_ = enable;
  digitalWrite(PIN_MOTOR_ENABLE, enable ? HIGH : LOW);
}

void HardwareGuard::updateLockActuator(bool engaged, uint16_t pulseUs) {
  lockActuator_ = engaged;
#if defined(ARDUINO_ARCH_ESP32) && defined(LOCK_IS_SERVO)
  // The state machine owns the policy (locked / sweep hold / retracted / manual debug)
  // and hands down the resolved target; this just drives it.
  lockServo_.writeMicroseconds(pulseUs);
#else
  // Digital (Due prototype) drive has no partial position -- partial holds are servo-only.
  (void)pulseUs;
  digitalWrite(PIN_LOCK_ACTUATOR, engaged ? HIGH : LOW);
#endif
}

void HardwareGuard::updateFan(bool enabled) {
  fanEnabled_ = enabled;
  digitalWrite(PIN_FAN_DRV_IN1, enabled ? HIGH : LOW);
  digitalWrite(PIN_FAN_DRV_IN2, LOW);
}

// Drive the door DRV8871 with a full-voltage kick (breaks the N20 worm-gear stiction)
// that settles into a slow PWM run. Mirrors door_motor_test.cpp exactly: the KICK is a
// digitalWrite on the Arduino pin (remaps correctly), the run is analogWrite on the RAW
// GPIO. Called every tick; kick is armed on the command EDGE and released by time, so it
// stays non-blocking and the FSM's hall auto-stop / timeout still land the instant they fire.
//
// SLOW-DECAY (brake) PWM so the run speed actually tracks the duty even with no load: hold
// the DRIVE input HIGH and PWM the OTHER input. During the PWM "on" slice the off-input is
// LOW -> the DRV8871 drives; during the "off" slice both inputs are HIGH -> it BRAKES. (The
// naive fast-decay scheme -- PWM one input, hold the other LOW -- coasts during the off
// slice, so an unloaded motor freewheels near full speed regardless of duty.)
//   OPEN  = forward: hold IN1 HIGH, PWM IN2.   CLOSE = reverse: hold IN2 HIGH, PWM IN1.
//   drive duty D (0-255) -> PWM pin gets analogWrite(255 - D): D=255 is full drive (kick),
//   small D is mostly braking = slow. runDuty is adjustable live from the UI.
//
// VM SCALING: every commanded duty (kick, run, UI DOOR_SPEED) is calibrated against the
// N20's 12 V rating; doorDutyOnVm() rescales it onto the actual VM rail at the write, so
// on the 24 V bus the motor's average voltage is unchanged (255 -> 127 applied) and full
// VM can never reach the 12 V motor through this path.
static uint8_t doorDutyOnVm(uint8_t duty) {
  return static_cast<uint8_t>(
      (static_cast<uint16_t>(duty) * DOOR_MOTOR_RATED_V) / DOOR_VM_VOLTAGE);
}

void HardwareGuard::updateDoorMotor(DoorMotorCommand command, uint8_t runDuty) {
  bool edge = (command != doorMotorCommand_);
  doorMotorCommand_ = command;

  if (command == DOOR_MOTOR_STOP) {
    if (edge) {
      doorKicking_ = false;
      doorCoast();
    }
    return;
  }

  bool driveIn1 = (command == DOOR_MOTOR_OPEN);
#if DOOR_DIR_INVERT
  driveIn1 = !driveIn1;   // flip motor direction to match the door mechanism / wiring
#endif
  // Both are LOGICAL pins (D6/D7); digitalWrite AND analogWrite remap them the same way.
  uint8_t holdPin = driveIn1 ? PIN_DOOR_DRV_IN1 : PIN_DOOR_DRV_IN2;   // held HIGH
  uint8_t pwmPin  = driveIn1 ? PIN_DOOR_DRV_IN2 : PIN_DOOR_DRV_IN1;   // PWM'd (inverted)

  if (edge) {
    // New move (or reversal): clean slate, hold the drive pin HIGH, kick at DOOR_KICK_DUTY
    // to break stiction. Runs at the kick strength until the kick timer expires.
    doorCoast();
    doorKicking_ = true;
    doorKickUntilMs_ = millis() + DOOR_KICK_MS;
    digitalWrite(holdPin, HIGH);
    analogWrite(pwmPin, 255 - doorDutyOnVm(DOOR_KICK_DUTY));
    doorAppliedDuty_ = DOOR_KICK_DUTY;
    return;
  }

  if (doorKicking_ && static_cast<int32_t>(millis() - doorKickUntilMs_) >= 0) {
    doorKicking_ = false;   // kick over -> fall through to the run duty below
  }

  uint8_t effectiveDuty = doorKicking_ ? DOOR_KICK_DUTY : runDuty;
  if (effectiveDuty != doorAppliedDuty_) {
    // Slow-decay: invert the duty onto the PWM (non-drive) pin. Honors live UI changes.
    analogWrite(pwmPin, 255 - doorDutyOnVm(effectiveDuty));
    doorAppliedDuty_ = effectiveDuty;
  }
}

// Motor off: release any PWM on the door inputs and drive both LOW (coast). Detaching the
// LEDC channel first means a subsequent digitalWrite hold is a true hard level, not PWM.
void HardwareGuard::doorCoast() {
#if defined(ARDUINO_ARCH_ESP32)
  ledcDetachPin(PIN_DOOR_DRV_IN1);   // logical pins -> the core remaps them internally
  ledcDetachPin(PIN_DOOR_DRV_IN2);
#endif
  pinMode(PIN_DOOR_DRV_IN1, OUTPUT);
  pinMode(PIN_DOOR_DRV_IN2, OUTPUT);
  digitalWrite(PIN_DOOR_DRV_IN1, LOW);
  digitalWrite(PIN_DOOR_DRV_IN2, LOW);
}

bool HardwareGuard::motorEnableOutput() const {
  return motorEnable_;
}
