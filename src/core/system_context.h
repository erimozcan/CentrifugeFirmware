#pragma once
#include <Arduino.h>
#include "faults.h"

struct SystemContext {
  // Timing
  uint32_t time_ms = 0;
  uint64_t safety_cycles = 0;
  uint32_t last_safety_ok_ms = 0;

  // High-level latches
  bool fault_latched = false;
  FaultLevel fault_level = FaultLevel::OK;
  FaultCode fault_code = FaultCode::NONE;

  // Inputs
  bool estop_active = false;
  bool odrive_fault_active = false;

  // Motor / ODrive
  bool motor_enable_cmd = false;   // desired state (guard may override)
  bool motor_enabled_actual = false;
  bool odrive_heartbeat_ok = false;

  // Sensors
  float rpm_primary = 0.0f;    // if available from ODrive
  float rpm_secondary = 0.0f;  // derived from RPM2
  bool rpm_secondary_ok = false;

  float vbus_v = 0.0f;

  // Lock
  bool lock_engaged_cmd = true; // default to engaged (but your lock is not fail-locked)
  bool lock_output_state = false;
};
