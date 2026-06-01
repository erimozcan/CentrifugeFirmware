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

bool writeLineNonBlocking(const char *line, size_t len) {
  if (Serial.availableForWrite() < static_cast<int>(len)) {
    return false;
  }
  Serial.write(reinterpret_cast<const uint8_t *>(line), len);
  return true;
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
  char line[256];
  size_t idx = 0U;

  idx = appendInt32(line, sizeof(line), idx, seq);
  idx = appendLiteral(line, sizeof(line), idx, " OK STATE=");
  idx = appendInt32(line, sizeof(line), idx, static_cast<int32_t>(ctx.state));

  idx = appendLiteral(line, sizeof(line), idx, " RPM_CMD=");
  idx = appendInt32(line, sizeof(line), idx, ctx.rpmCmd);

  idx = appendLiteral(line, sizeof(line), idx, " RPM1=");
  idx = appendInt32(line, sizeof(line), idx, ctx.rpm1);

  idx = appendLiteral(line, sizeof(line), idx, " FAULT=");
  idx = appendInt32(line, sizeof(line), idx, static_cast<int32_t>(ctx.fault));

  idx = appendLiteral(line, sizeof(line), idx, " LOCK=");
  idx = appendInt32(line, sizeof(line), idx, ctx.lockConfirmed ? 1 : 0);

  idx = appendLiteral(line, sizeof(line), idx, " ENABLE=");
  idx = appendInt32(line, sizeof(line), idx, ctx.motorEnableOutput ? 1 : 0);

  idx = appendLiteral(line, sizeof(line), idx, " DOOR=");
  idx = appendInt32(line, sizeof(line), idx, static_cast<int32_t>(ctx.doorState));

  idx = appendLiteral(line, sizeof(line), idx, " TEMP_ADC=");
  idx = appendInt32(line, sizeof(line), idx, static_cast<int32_t>(ctx.ntcAdc));

  idx = appendLiteral(line, sizeof(line), idx, " FAN=");
  idx = appendInt32(line, sizeof(line), idx, ctx.fanEnabled ? 1 : 0);

  if (idx >= sizeof(line) - 1U) {
    return false;
  }

  line[idx++] = '\n';
  return writeLineNonBlocking(line, idx);
}

}  // namespace Protocol
