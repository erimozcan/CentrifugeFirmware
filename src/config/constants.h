#pragma once
#include <Arduino.h>

// ODrive UART
static constexpr uint32_t ODRIVE_BAUDRATE = 115200;

// Use Serial1/Serial2/Serial3 depending on wiring.
#define ODRIVE_SERIAL_PORT Serial1

// Input polarities
static constexpr bool ESTOP_ACTIVE_LOW = true;
static constexpr bool ODRIVE_FAULT_ACTIVE_LOW = true;

// Safety thresholds (placeholder - YOU MUST CALIBRATE)
// vbus divider scaling: Vbus = adc_counts * VBUS_ADC_TO_VOLTS
static constexpr float VBUS_ADC_TO_VOLTS = 0.01f;  // placeholder

static constexpr float VBUS_WARN_V  = 52.0f;  // placeholder
static constexpr float VBUS_CRIT_V  = 54.0f;  // placeholder

// RPM rules (placeholder)
static constexpr uint32_t RPM2_PULSES_PER_REV = 2;
static constexpr float RPM_PRIMARY_MIN_MOVING = 100.0f; // below this treat as "not moving"
static constexpr uint32_t RPM2_LOSS_TIMEOUT_MS = 100;   // if moving and no pulse for this long -> fault

// MCU watchdog timeout (ms). Keep short.
static constexpr uint32_t MCU_WDT_TIMEOUT_MS = 30;
