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
  // engaged drives the digital (Due) build; pulseUs is the resolved servo target the
  // state machine already worked out (locked / sweep hold / retracted / manual debug).
  void updateLockActuator(bool engaged, uint16_t pulseUs);
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
