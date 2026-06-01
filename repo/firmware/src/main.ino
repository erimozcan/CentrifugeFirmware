#include <Arduino.h>

#include "command_interface.h"
#include "config.h"
#include "context.h"
#include "hardware_guard.h"
#include "motor_interface.h"
#include "state_machine.h"

namespace {

SystemContext g_ctx;
PendingCommand g_pending;
SensorData g_sensorShared;

StateMachine g_stateMachine;
HardwareGuard g_guard;
MotorInterface g_motor;
CommandInterface g_commands;

uint32_t g_lastTickUs = 0U;

void updateSharedSensors() {
  SensorData latest;
  latest.rpm1 = g_motor.lastMeasuredRpm();
  latest.lockSensor = (digitalRead(PIN_LOCK_SENSOR) == HIGH) ? 1U : 0U;
  latest.doorOpenHall = (digitalRead(PIN_DOOR_OPEN_HALL) == HIGH) ? 1U : 0U;
  latest.doorClosedHall = (digitalRead(PIN_DOOR_CLOSED_HALL) == HIGH) ? 1U : 0U;
  latest.ntcAdc = static_cast<uint16_t>(analogRead(PIN_TEMP_NTC));

  noInterrupts();
  g_sensorShared = latest;
  interrupts();
}

void runTick(uint32_t nowMs) {
  SensorData snapshot;
  noInterrupts();
  snapshot = g_sensorShared;
  interrupts();

  g_stateMachine.tick(g_ctx, snapshot, g_pending, g_motor, g_guard, nowMs);
}

}  // namespace

void setup() {
  Serial.begin(SERIAL_BAUD);
  analogReadResolution(12);

  g_guard.begin();
  g_motor.begin();
  g_commands.begin();

  clearPendingCommand(g_pending);

  noInterrupts();
  g_sensorShared.rpm1 = 0;
  g_sensorShared.lockSensor = 0U;
  g_sensorShared.doorOpenHall = 0U;
  g_sensorShared.doorClosedHall = 0U;
  g_sensorShared.ntcAdc = 0U;
  interrupts();

  g_stateMachine.begin(g_ctx, millis());
  g_lastTickUs = micros();
}

void loop() {
  g_commands.poll(g_pending, g_ctx);
  updateSharedSensors();

  uint32_t nowUs = micros();
  uint8_t catchupCount = 0U;

  while ((uint32_t)(nowUs - g_lastTickUs) >= TICK_INTERVAL_US && catchupCount < MAX_TICK_CATCHUP) {
    g_lastTickUs += TICK_INTERVAL_US;
    runTick(millis());
    catchupCount++;
    nowUs = micros();
  }
}
