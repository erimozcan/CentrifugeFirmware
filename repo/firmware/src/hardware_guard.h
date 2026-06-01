#ifndef HARDWARE_GUARD_H
#define HARDWARE_GUARD_H

#include <Arduino.h>

#include "context.h"

class HardwareGuard {
 public:
  void begin();
  void updateMotorEnable(SystemState state);
  void updateLockActuator(bool engaged);
  void updateFan(bool enabled);
  void updateDoorMotor(DoorMotorCommand command);

  bool motorEnableOutput() const;

 private:
  bool motorEnable_;
  bool lockActuator_;
  bool fanEnabled_;
  DoorMotorCommand doorMotorCommand_;
};

#endif
