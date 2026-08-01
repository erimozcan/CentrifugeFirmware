#ifndef HARDWARE_GUARD_H
#define HARDWARE_GUARD_H

#include <Arduino.h>

#include "context.h"   // pulls in config.h (defines LOCK_IS_SERVO on the Nano)

#if defined(ARDUINO_ARCH_ESP32) && defined(LOCK_IS_SERVO)
#include <ESP32Servo.h>
#endif

class HardwareGuard {
 public:
  void begin();
  void updateMotorEnable(SystemState state);
  void updateLockActuator(bool engaged, bool sweepHold);
  void updateFan(bool enabled);
  void updateDoorMotor(DoorMotorCommand command, uint8_t runDuty);

  bool motorEnableOutput() const;

 private:
  void doorCoast();   // release PWM + drive both DRV8871 inputs LOW (motor off)

#if defined(ARDUINO_ARCH_ESP32) && defined(LOCK_IS_SERVO)
  Servo lockServo_;
#endif
  bool motorEnable_;
  bool lockActuator_;
  bool fanEnabled_;
  DoorMotorCommand doorMotorCommand_;
  // Door drive shaping: full-voltage kick until doorKickUntilMs_, then slow PWM run.
  uint32_t doorKickUntilMs_;
  bool doorKicking_;
  uint8_t doorAppliedDuty_;   // last analogWrite duty (so live speed changes re-apply)
};

#endif
