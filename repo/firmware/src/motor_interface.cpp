#include "motor_interface.h"

#include "config.h"

void MotorInterface::begin() {
  Serial1.begin(ESC_UART_BAUD);
  commandOutputEnabled_ = true;
  lastCommandedRpm_ = 0;
}

void MotorInterface::setCommandOutputEnabled(bool enabled) {
  commandOutputEnabled_ = enabled;
}

void MotorInterface::sendVelocitySetpoint(int32_t rpm) {
  if (!commandOutputEnabled_) {
    return;
  }

  if (rpm < 0) {
    rpm = 0;
  }

  if (rpm > MAX_RPM_BAREBONES) {
    rpm = MAX_RPM_BAREBONES;
  }

  char cmd[ESC_UART_CMD_MAX_LEN];
  size_t idx = buildVelocityCommand(cmd, sizeof(cmd), rpm);
  if (idx == 0U) {
    return;
  }

  if (Serial1.availableForWrite() < static_cast<int>(idx)) {
    return;
  }

  Serial1.write(reinterpret_cast<const uint8_t *>(cmd), idx);
  lastCommandedRpm_ = rpm;
}

void MotorInterface::drainRx() {
  while (Serial1.available()) {
    Serial1.read();
  }
}

int32_t MotorInterface::lastMeasuredRpm() const {
  return lastCommandedRpm_;
}

size_t MotorInterface::writeInt32(char *out, size_t maxLen, int32_t value) {
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

size_t MotorInterface::buildVelocityCommand(char *out, size_t maxLen, int32_t rpm) {
  if (maxLen == 0U) {
    return 0U;
  }

  if (maxLen < 6U) {
    return 0U;
  }

  // Generic ESC operator command format:
  // V=<rpm>\n
  // Example: V=250
  size_t idx = 0U;
  out[idx++] = 'V';
  out[idx++] = '=';

  idx += writeInt32(&out[idx], maxLen - idx, rpm);
  if (idx >= maxLen - 1U) {
    return 0U;
  }

  out[idx++] = '\n';

  return idx;
}
