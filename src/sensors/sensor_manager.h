#pragma once
#include <Arduino.h>
#include "../config/pins.h"
#include "../config/constants.h"
#include "../core/system_context.h"

class SensorManager {
public:
  void begin() {
    pinMode(PIN_RPM2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_RPM2), isr_rpm2_edge, RISING);

    _last_edge_us = micros();
    _rpm2_ok = false;

    analogReadResolution(12);
  }

  void update_isr_side() {
    // Nothing needed here in v0; ISR updates volatile fields.
  }

  void publish_to_context(SystemContext& ctx) {
    ctx.estop_active = read_estop();
    ctx.odrive_fault_active = read_odrive_fault();
    ctx.vbus_v = read_vbus();

    float rpm2 = 0.0f;
    bool ok = compute_rpm2(rpm2);
    ctx.rpm_secondary = rpm2;
    ctx.rpm_secondary_ok = ok;
  }

private:
  static void isr_rpm2_edge() {
    const uint32_t now = micros();
    const uint32_t dt = now - _last_edge_us;
    // Glitch reject: reject absurdly short dt
    if (dt > 50) {
      _last_period_us = dt;
      _last_edge_us = now;
      _edge_count++;
    }
  }

  bool read_estop() const {
    const int v = digitalRead(PIN_ESTOP_N);
    return ESTOP_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
  }

  bool read_odrive_fault() const {
    const int v = digitalRead(PIN_ODRIVE_FAULT);
    return ODRIVE_FAULT_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
  }

  float read_vbus() const {
    const uint32_t adc = analogRead(PIN_VBUS_ADC);
    return (float)adc * VBUS_ADC_TO_VOLTS;
  }

  bool compute_rpm2(float& rpm_out) const {
    const uint32_t now_ms = millis();

    // If we have not seen edges recently while "moving", mark lost.
    const uint32_t last_edge_ms = _last_edge_us / 1000UL;
    const uint32_t age = now_ms - last_edge_ms;
    if (age > RPM2_LOSS_TIMEOUT_MS) {
      rpm_out = 0.0f;
      return false;
    }

    uint32_t p = _last_period_us;
    if (p == 0) {
      rpm_out = 0.0f;
      return false;
    }

    // period method:
    // RPM = 60e6 / (pulsesPerRev * period_us)
    rpm_out = (60.0f * 1000000.0f) / ((float)RPM2_PULSES_PER_REV * (float)p);
    return true;
  }

  static inline volatile uint32_t _last_edge_us = 0;
  static inline volatile uint32_t _last_period_us = 0;
  static inline volatile uint32_t _edge_count = 0;

  bool _rpm2_ok = false;
};
