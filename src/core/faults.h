#pragma once
#include <Arduino.h>

enum class FaultLevel : uint8_t {
  OK = 0,
  WARN = 1,
  SOFT_STOP = 2,
  HARD_STOP = 3,
  FAULT_LATCHED = 4
};

enum class FaultCode : uint16_t {
  NONE = 0,
  ESTOP_ACTIVE = 1,
  ODRIVE_FAULT_LINE = 2,
  VBUS_CRITICAL = 3,
  RPM2_LOST_WHILE_MOVING = 4,
  SENSOR_FROZEN = 5,
  ODRIVE_UNRESPONSIVE = 6
};
