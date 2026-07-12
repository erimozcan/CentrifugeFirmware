#include <Arduino.h>

#include "audio_controller.h"
#include "command_interface.h"
#include "config.h"
#include "context.h"
#include "hardware_guard.h"
#include "led_controller.h"
#include "motor_interface.h"
#include "net_server.h"
#include "protocol.h"
#include "state_machine.h"

namespace {

SystemContext g_ctx;
PendingCommand g_pending;
SensorData g_sensorShared;

StateMachine g_stateMachine;
HardwareGuard g_guard;
MotorInterface g_motor;
CommandInterface g_commands;
LedController g_led;
AudioController g_audio;

uint32_t g_lastTickUs = 0U;

void updateSharedSensors() {
  SensorData latest;
  latest.rpm1 = g_motor.lastMeasuredRpm();
  latest.lockSensor = (digitalRead(PIN_LOCK_SENSOR) == HIGH) ? 1U : 0U;
  latest.doorOpenHall = (digitalRead(PIN_DOOR_OPEN_HALL) == HIGH) ? 1U : 0U;
  latest.doorClosedHall = (digitalRead(PIN_DOOR_CLOSED_HALL) == HIGH) ? 1U : 0U;
#if HAS_NTC
  latest.ntcAdc = static_cast<uint16_t>(analogRead(PIN_TEMP_NTC));
#else
  latest.ntcAdc = NTC_ADC_NO_SENSOR;   // no thermistor fitted -> report "cool"
#endif

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

#ifndef TEST_SERIAL

void setup() {
  Serial.begin(SERIAL_BAUD);
  analogReadResolution(12);

  // Bring the WiFi AP + web/WebSocket console up FIRST (its own task on core 0), before any
  // potentially-slow init below, so the page is reachable within ~1 s of power-on. (no-op if
  // HAS_WIFI=0.)
  NetServer::begin();

  g_guard.begin();
  g_motor.begin();
  g_commands.begin();
  g_led.begin();
  g_audio.begin();

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

  // Drain any commands received over WiFi (queued by the core-0 net task). They run through
  // the SAME command handler as USB, here on core 1, with replies captured and sent back over
  // the WebSocket -- so the state machine only ever sees commands from one thread.
  {
    NetServer::Cmd nc;
    while (NetServer::popCommand(nc)) {
      static LineCapture cap;
      cap.reset();
      Protocol::setOutput(&cap);
      g_commands.handleExternalLine(nc.line, g_pending, g_ctx);
      Protocol::setOutput(nullptr);   // restore Serial
      if (cap.length() > 0) NetServer::pushReply(nc.client, cap.c_str());
    }
  }

  updateSharedSensors();

  uint32_t nowUs = micros();
  uint8_t catchupCount = 0U;

  while ((uint32_t)(nowUs - g_lastTickUs) >= TICK_INTERVAL_US && catchupCount < MAX_TICK_CATCHUP) {
    g_lastTickUs += TICK_INTERVAL_US;
    runTick(millis());
    catchupCount++;
    nowUs = micros();
  }

  // Render the LED strip from the (tick-updated) context. Self-throttled, so calling
  // it every loop is fine; kept out of the tick so strip.show() never stalls a tick.
  g_led.render(g_ctx.ledR, g_ctx.ledG, g_ctx.ledB, g_ctx.ledMode, millis());

  // Play event-driven audio cues by observing the context. Also kept out of the tick:
  // playback is fire-and-forget, but this is where manual requests are serviced too.
  g_audio.service(g_ctx, millis());
}

#endif  // TEST_SERIAL
