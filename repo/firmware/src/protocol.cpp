#include "protocol.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>

namespace {

size_t writeInt32(char *out, size_t maxLen, int32_t value) {
  if (maxLen == 0U) {
    return 0U;
  }

  if (value == 0) {
    out[0] = '0';
    return 1U;
  }

  uint32_t magnitude = 0U;
  bool negative = false;

  if (value < 0) {
    negative = true;
    magnitude = static_cast<uint32_t>(-(value + 1)) + 1U;
  } else {
    magnitude = static_cast<uint32_t>(value);
  }

  char digits[11];
  size_t count = 0U;
  while (magnitude > 0U && count < sizeof(digits)) {
    digits[count++] = static_cast<char>('0' + (magnitude % 10U));
    magnitude /= 10U;
  }

  size_t outIdx = 0U;
  if (negative) {
    if (outIdx >= maxLen) {
      return 0U;
    }
    out[outIdx++] = '-';
  }

  while (count > 0U) {
    if (outIdx >= maxLen) {
      break;
    }
    out[outIdx++] = digits[--count];
  }

  return outIdx;
}

size_t appendLiteral(char *out, size_t maxLen, size_t idx, const char *text) {
  while (*text != '\0' && idx < maxLen) {
    out[idx++] = *text++;
  }
  return idx;
}

size_t appendInt32(char *out, size_t maxLen, size_t idx, int32_t value) {
  if (idx >= maxLen) {
    return idx;
  }
  idx += writeInt32(&out[idx], maxLen - idx, value);
  return idx;
}

Print *g_out = &Serial;   // reply sink; default = USB serial

bool writeLineNonBlocking(const char *line, size_t len) {
  // Redirected sink (e.g. WebSocket capture): just write it, no Serial back-pressure logic.
  if (g_out != &Serial) {
    g_out->write(reinterpret_cast<const uint8_t *>(line), len);
    return true;
  }
  // Skip only when the whole line won't fit, so the master's tick loop never
  // stalls waiting on a host that stopped reading.
  if (Serial.availableForWrite() >= static_cast<int>(len)) {
    Serial.write(reinterpret_cast<const uint8_t *>(line), len);
    return true;
  }
#if defined(ARDUINO_ARCH_ESP32)
  // USB-CDC reports a small availableForWrite() (< a full STATUS line) even when
  // there is room, so honoring the guard above would silently drop replies.
  // Write anyway -- USB-CDC write is bounded (it drops when the host is absent),
  // so this can't hang the loop.
  Serial.write(reinterpret_cast<const uint8_t *>(line), len);
  return true;
#else
  return false;
#endif
}

}  // namespace

namespace Protocol {

bool parseInt32(const char *text, int32_t &value) {
  if (text == nullptr || *text == '\0') {
    return false;
  }

  char *endPtr = nullptr;
  long parsed = strtol(text, &endPtr, 10);
  if (endPtr == text || *endPtr != '\0') {
    return false;
  }

  if (parsed < LONG_MIN || parsed > LONG_MAX) {
    return false;
  }

  value = static_cast<int32_t>(parsed);
  return true;
}

bool parseKeyValueInt(const char *token, const char *key, int32_t &value) {
  if (token == nullptr || key == nullptr) {
    return false;
  }

  size_t keyLen = strlen(key);
  if (strncmp(token, key, keyLen) != 0 || token[keyLen] != '=') {
    return false;
  }

  return parseInt32(token + keyLen + 1U, value);
}

bool sendOk(int32_t seq, const char *payload) {
  char line[160];
  size_t idx = 0U;

  idx = appendInt32(line, sizeof(line), idx, seq);
  idx = appendLiteral(line, sizeof(line), idx, " OK");

  if (payload != nullptr && *payload != '\0') {
    idx = appendLiteral(line, sizeof(line), idx, " ");
    idx = appendLiteral(line, sizeof(line), idx, payload);
  }

  if (idx >= sizeof(line) - 1U) {
    return false;
  }

  line[idx++] = '\n';
  return writeLineNonBlocking(line, idx);
}

bool sendErr(int32_t seq, const char *code) {
  char line[128];
  size_t idx = 0U;

  idx = appendInt32(line, sizeof(line), idx, seq);
  idx = appendLiteral(line, sizeof(line), idx, " ERR CODE=");
  idx = appendLiteral(line, sizeof(line), idx, code);

  if (idx >= sizeof(line) - 1U) {
    return false;
  }

  line[idx++] = '\n';
  return writeLineNonBlocking(line, idx);
}

bool sendStatus(int32_t seq, const SystemContext &ctx) {
  char line[512];
  size_t idx = 0U;

  idx = appendInt32(line, sizeof(line), idx, seq);
  idx = appendLiteral(line, sizeof(line), idx, " OK STATE=");
  idx = appendInt32(line, sizeof(line), idx, static_cast<int32_t>(ctx.state));

  idx = appendLiteral(line, sizeof(line), idx, " RPM_CMD=");
  idx = appendInt32(line, sizeof(line), idx, ctx.rpmCmd);

  idx = appendLiteral(line, sizeof(line), idx, " RPM1=");
  idx = appendInt32(line, sizeof(line), idx, ctx.rpm1);

  idx = appendLiteral(line, sizeof(line), idx, " ESC_CAL=");
  idx = appendLiteral(line, sizeof(line), idx, ctx.escCalState);
  idx = appendLiteral(line, sizeof(line), idx, " ESC_CAL_STEP=");
  idx = appendInt32(line, sizeof(line), idx, ctx.escCalStep);
  idx = appendLiteral(line, sizeof(line), idx, " ESC_CAL_TARGET=");
  idx = appendInt32(line, sizeof(line), idx, ctx.escCalTargetRpm);
  idx = appendLiteral(line, sizeof(line), idx, " ESC_CUR_CA=");
  idx = appendInt32(line, sizeof(line), idx, ctx.escCurrentCentiamps);
  idx = appendLiteral(line, sizeof(line), idx, " ESC_OBS_RPM=");
  idx = appendInt32(line, sizeof(line), idx, ctx.escObserverRpm);
  idx = appendLiteral(line, sizeof(line), idx, " ESC_OBS_LOCK=");
  idx = appendInt32(line, sizeof(line), idx, static_cast<int32_t>(ctx.escObserverLock));
  idx = appendLiteral(line, sizeof(line), idx, " ESC_PHASE_ERR_D10=");
  idx = appendInt32(line, sizeof(line), idx, ctx.escPhaseErrDeg10);
  idx = appendLiteral(line, sizeof(line), idx, " ESC_LOOP_HZ=");
  idx = appendInt32(line, sizeof(line), idx, static_cast<int32_t>(ctx.escLoopHz));

  idx = appendLiteral(line, sizeof(line), idx, " FAULT=");
  idx = appendInt32(line, sizeof(line), idx, static_cast<int32_t>(ctx.fault));

  // LOCK = lock *sensor* (lockConfirmed); open-loop on the Nano (no sensor -> floats
  // "confirmed"). LOCKCMD = the actuator position we last *commanded* (1=locked), which
  // is the true feedback for the lock buttons during bring-up.
  idx = appendLiteral(line, sizeof(line), idx, " LOCK=");
  idx = appendInt32(line, sizeof(line), idx, ctx.lockConfirmed ? 1 : 0);

  idx = appendLiteral(line, sizeof(line), idx, " LOCKCMD=");
  idx = appendInt32(line, sizeof(line), idx, ctx.lockActuatorCommanded ? 1 : 0);

  idx = appendLiteral(line, sizeof(line), idx, " ENABLE=");
  idx = appendInt32(line, sizeof(line), idx, ctx.motorEnableOutput ? 1 : 0);

  idx = appendLiteral(line, sizeof(line), idx, " DOOR=");
  idx = appendInt32(line, sizeof(line), idx, static_cast<int32_t>(ctx.doorState));

  // DOORMOVE=1 while a motorized move is in progress; DOORCMD is the drive direction
  // (-1 opening, 0 stopped, 1 closing). Together they let the UI show live motion.
  idx = appendLiteral(line, sizeof(line), idx, " DOORMOVE=");
  idx = appendInt32(line, sizeof(line), idx, ctx.doorMoveActive ? 1 : 0);

  idx = appendLiteral(line, sizeof(line), idx, " DOORCMD=");
  idx = appendInt32(line, sizeof(line), idx, static_cast<int32_t>(ctx.doorMotorCommand));

  idx = appendLiteral(line, sizeof(line), idx, " DOORPWM=");
  idx = appendInt32(line, sizeof(line), idx, static_cast<int32_t>(ctx.doorPwmDuty));

  // Gantry indexing: TUBE = the position it's parked at (0 = unknown, e.g. after a spin);
  // ROTTGT = the tube an in-progress rotate is heading to (0 when not rotating).
  idx = appendLiteral(line, sizeof(line), idx, " TUBE=");
  idx = appendInt32(line, sizeof(line), idx, static_cast<int32_t>(ctx.currentTube));
  idx = appendLiteral(line, sizeof(line), idx, " HOMED=");
  idx = appendInt32(line, sizeof(line), idx, ctx.homed ? 1 : 0);
  idx = appendLiteral(line, sizeof(line), idx, " ROTTGT=");
  {
    bool rotating = (ctx.state == STATE_ROTATE_RELEASE || ctx.state == STATE_ROTATE_MOVING ||
                     ctx.state == STATE_ROTATE_ENGAGE);
    idx = appendInt32(line, sizeof(line), idx, rotating ? static_cast<int32_t>(ctx.rotateTube) : 0);
  }

  // Raw hall pin levels (1=HIGH) so the UI/console can verify polarity vs the
  // DOOR_*_ACTIVE_LOW config during door bring-up, independent of interpretation.
  idx = appendLiteral(line, sizeof(line), idx, " HOPEN=");
  idx = appendInt32(line, sizeof(line), idx, static_cast<int32_t>(ctx.doorOpenHallRaw));

  idx = appendLiteral(line, sizeof(line), idx, " HCLOSED=");
  idx = appendInt32(line, sizeof(line), idx, static_cast<int32_t>(ctx.doorClosedHallRaw));

  idx = appendLiteral(line, sizeof(line), idx, " TEMP_ADC=");
  idx = appendInt32(line, sizeof(line), idx, static_cast<int32_t>(ctx.ntcAdc));

  idx = appendLiteral(line, sizeof(line), idx, " FAN=");
  idx = appendInt32(line, sizeof(line), idx, ctx.fanEnabled ? 1 : 0);

  idx = appendLiteral(line, sizeof(line), idx, " POWER=");
  idx = appendInt32(line, sizeof(line), idx, ctx.devicePowered ? 1 : 0);

  // AUDIO = 1 if the DFPlayer PRO initialized OK (module wired + powered), else 0.
  idx = appendLiteral(line, sizeof(line), idx, " AUDIO=");
  idx = appendInt32(line, sizeof(line), idx, ctx.audioReady ? 1 : 0);

  idx = appendLiteral(line, sizeof(line), idx, " LEDMODE=");
  idx = appendInt32(line, sizeof(line), idx, static_cast<int32_t>(ctx.ledMode));
  idx = appendLiteral(line, sizeof(line), idx, " LEDR=");
  idx = appendInt32(line, sizeof(line), idx, static_cast<int32_t>(ctx.ledR));
  idx = appendLiteral(line, sizeof(line), idx, " LEDG=");
  idx = appendInt32(line, sizeof(line), idx, static_cast<int32_t>(ctx.ledG));
  idx = appendLiteral(line, sizeof(line), idx, " LEDB=");
  idx = appendInt32(line, sizeof(line), idx, static_cast<int32_t>(ctx.ledB));

  if (idx >= sizeof(line) - 1U) {
    return false;
  }

  line[idx++] = '\n';
  return writeLineNonBlocking(line, idx);
}

void setOutput(Print *out) { g_out = out ? out : &Serial; }
Print *output() { return g_out; }

}  // namespace Protocol
