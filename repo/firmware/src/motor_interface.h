#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H

#include <Arduino.h>

class MotorInterface {
 public:
  void begin();
  void setCommandOutputEnabled(bool enabled);
  void sendVelocitySetpoint(int32_t rpm);
  void drainRx();

  int32_t lastMeasuredRpm() const;

 private:
  bool commandOutputEnabled_;
  int32_t lastCommandedRpm_;

  static size_t writeInt32(char *out, size_t maxLen, int32_t value);
  static size_t buildVelocityCommand(char *out, size_t maxLen, int32_t rpm);
};

#endif
