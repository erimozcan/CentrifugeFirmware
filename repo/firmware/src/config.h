#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

#define SERIAL_BAUD 115200
#define ESC_UART_BAUD 115200
#define ESC_UART_CMD_MAX_LEN 24U

#define TICK_INTERVAL_US 1000U
#define MAX_TICK_CATCHUP 4U

#define RPM_SCALE 1000
#define MAX_RPM_BAREBONES 500
#define SAFE_UNLOCK_RPM 50

#define PIN_MOTOR_ENABLE 22
#define PIN_LOCK_ACTUATOR 23
#define PIN_LOCK_SENSOR 24
#define PIN_FAN_DRV_IN1 5
#define PIN_FAN_DRV_IN2 4
#define PIN_DOOR_DRV_IN1 6
#define PIN_DOOR_DRV_IN2 7
#define PIN_DOOR_OPEN_HALL 30
#define PIN_DOOR_CLOSED_HALL 31
#define PIN_TEMP_NTC A0

#define DOOR_OPEN_ACTIVE_LOW 1
#define DOOR_CLOSED_ACTIVE_LOW 1

#define LOCK_ENGAGE_TIMEOUT_MS 1500U
#define PRE_RUN_VERIFY_MS 200U
#define DOOR_MOVE_TIMEOUT_MS 2500U

// NTC (100k divider) thresholds in raw ADC counts (12-bit Due ADC)
// Lower ADC == hotter for pullup-divider wiring.
#define NTC_ADC_FAN_ON 1400
#define NTC_ADC_FAN_OFF 1700
#define NTC_ADC_CRITICAL_LOW 1100

#endif
