#pragma once
#include <Arduino.h>

// You MUST set these pins to match your wiring.
// Use Due pin names as Arduino pin numbers where applicable.

// MotorEnable to ODrive enable input.
// Recommended wiring: output -> transistor -> ODrive enable, plus external pull-down on enable line.
static constexpr uint8_t PIN_MOTOR_ENABLE = 22;

// Lock actuator output (DO NOT USE to unlock; firmware does not implement unlock).
static constexpr uint8_t PIN_LOCK_ACTUATE = 23;

// E-stop input (active LOW recommended with pull-up).
static constexpr uint8_t PIN_ESTOP_N = 24;

// ODrive fault input (active LOW or HIGH depends on wiring; set in constants.h)
static constexpr uint8_t PIN_ODRIVE_FAULT = 25;

// Secondary RPM sensor interrupt pin (hall/optical). Must be interrupt capable.
static constexpr uint8_t PIN_RPM2 = 2;

// DC bus voltage ADC pin (through divider). Due ADC: A0..A11
static constexpr uint8_t PIN_VBUS_ADC = A0;

// Accelerometer optional: not implemented in v0 (placeholder pins)
static constexpr uint8_t PIN_ACCEL_X = A1;
static constexpr uint8_t PIN_ACCEL_Y = A2;
static constexpr uint8_t PIN_ACCEL_Z = A3;
