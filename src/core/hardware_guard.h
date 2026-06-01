#pragma once
#include <Arduino.h>
#include "../config/pins.h"
#include "system_context.h"

class HardwareGuard {
public:
  void begin() {
    pinMode(PIN_MOTOR_ENABLE, OUTPUT);
    pinMode(PIN_LOCK_ACTUATE, OUTPUT);

    pinMode(PIN_ESTOP_N, INPUT_PULLUP);
    pinMode(PIN_ODRIVE_FAULT, INPUT_PULLUP);

    // Default safe: disabled
    digitalWrite(PIN_MOTOR_ENABLE, LOW);
    // Default: do not drive lock output unless you know your actuator polarity
    digitalWrite(PIN_LOCK_ACTUATE, LOW);
  }

  void force_disable() {
    digitalWrite(PIN_MOTOR_ENABLE, LOW);
  }

  void update_outputs(SystemContext& ctx) {
    // Motor enable is guarded: never enable if fault latched.
    if (ctx.fault_latched) {
      digitalWrite(PIN_MOTOR_ENABLE, LOW);
      ctx.motor_enabled_actual = false;
    } else {
      digitalWrite(PIN_MOTOR_ENABLE, ctx.motor_enable_cmd ? HIGH : LOW);
      ctx.motor_enabled_actual = ctx.motor_enable_cmd;
    }

    // Lock: since your lock is NOT fail-locked, this firmware refuses to unlock.
    // It may optionally assert a "lock engage" output, but only when RPM==0 sustained.
    const bool rpm0 = (ctx.rpm_primary < 1.0f && ctx.rpm_secondary < 1.0f);
    if (rpm0 && ctx.lock_engaged_cmd) {
      digitalWrite(PIN_LOCK_ACTUATE, HIGH);
      ctx.lock_output_state = true;
    } else {
      digitalWrite(PIN_LOCK_ACTUATE, LOW);
      ctx.lock_output_state = false;
    }
  }

  void hard_stop(SystemContext& ctx) {
    digitalWrite(PIN_MOTOR_ENABLE, LOW);
    ctx.motor_enabled_actual = false;
    // Keep lock output asserted only if safe and commanded; otherwise drop it.
    update_outputs(ctx);
  }
};
