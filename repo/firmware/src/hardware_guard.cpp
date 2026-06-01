#include "hardware_guard.h"

#include "config.h"

void HardwareGuard::begin() {
  pinMode(PIN_MOTOR_ENABLE, OUTPUT);
  pinMode(PIN_LOCK_ACTUATOR, OUTPUT);
  pinMode(PIN_LOCK_SENSOR, INPUT_PULLUP);
  pinMode(PIN_FAN_DRV_IN1, OUTPUT);
  pinMode(PIN_FAN_DRV_IN2, OUTPUT);
  pinMode(PIN_DOOR_DRV_IN1, OUTPUT);
  pinMode(PIN_DOOR_DRV_IN2, OUTPUT);
  pinMode(PIN_DOOR_OPEN_HALL, INPUT_PULLUP);
  pinMode(PIN_DOOR_CLOSED_HALL, INPUT_PULLUP);

  motorEnable_ = false;
  lockActuator_ = false;
  fanEnabled_ = false;
  doorMotorCommand_ = DOOR_MOTOR_STOP;

  digitalWrite(PIN_MOTOR_ENABLE, LOW);
  digitalWrite(PIN_LOCK_ACTUATOR, LOW);
  digitalWrite(PIN_FAN_DRV_IN1, LOW);
  digitalWrite(PIN_FAN_DRV_IN2, LOW);
  digitalWrite(PIN_DOOR_DRV_IN1, LOW);
  digitalWrite(PIN_DOOR_DRV_IN2, LOW);
}

void HardwareGuard::updateMotorEnable(SystemState state) {
  bool enable = isMotorEnabledState(state);
  motorEnable_ = enable;
  digitalWrite(PIN_MOTOR_ENABLE, enable ? HIGH : LOW);
}

void HardwareGuard::updateLockActuator(bool engaged) {
  lockActuator_ = engaged;
  digitalWrite(PIN_LOCK_ACTUATOR, engaged ? HIGH : LOW);
}

void HardwareGuard::updateFan(bool enabled) {
  fanEnabled_ = enabled;
  digitalWrite(PIN_FAN_DRV_IN1, enabled ? HIGH : LOW);
  digitalWrite(PIN_FAN_DRV_IN2, LOW);
}

void HardwareGuard::updateDoorMotor(DoorMotorCommand command) {
  doorMotorCommand_ = command;

  if (command == DOOR_MOTOR_OPEN) {
    digitalWrite(PIN_DOOR_DRV_IN1, HIGH);
    digitalWrite(PIN_DOOR_DRV_IN2, LOW);
    return;
  }

  if (command == DOOR_MOTOR_CLOSE) {
    digitalWrite(PIN_DOOR_DRV_IN1, LOW);
    digitalWrite(PIN_DOOR_DRV_IN2, HIGH);
    return;
  }

  digitalWrite(PIN_DOOR_DRV_IN1, LOW);
  digitalWrite(PIN_DOOR_DRV_IN2, LOW);
}

bool HardwareGuard::motorEnableOutput() const {
  return motorEnable_;
}
