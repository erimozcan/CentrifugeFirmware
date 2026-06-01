#include "command_interface.h"

#include <string.h>

#include "config.h"
#include "protocol.h"

namespace {

void toUpperAscii(char *text) {
  if (text == nullptr) {
    return;
  }

  while (*text != '\0') {
    if (*text >= 'a' && *text <= 'z') {
      *text = static_cast<char>(*text - ('a' - 'A'));
    }
    ++text;
  }
}

bool parseSeq(const char *token, int32_t &seq) {
  return Protocol::parseInt32(token, seq);
}

bool validateDuration(int32_t value) {
  return value >= 0;
}

bool isRunInterlockSafe(const SystemContext &ctx) {
  return ctx.doorState == DOOR_STATE_CLOSED && ctx.lockConfirmed && !ctx.overTempCritical;
}

}  // namespace

void CommandInterface::begin() {
  lineLength_ = 0U;
}

void CommandInterface::poll(PendingCommand &pending, const SystemContext &ctx) {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());

    if (c == '\r' || c == '\n') {
      if (lineLength_ == 0U) {
        continue;
      }

      lineBuffer_[lineLength_] = '\0';
      handleLine(lineBuffer_, pending, ctx);
      lineLength_ = 0U;
      continue;
    }

    if (lineLength_ < kMaxLineLength) {
      lineBuffer_[lineLength_++] = c;
    } else {
      lineLength_ = 0U;
      Protocol::sendErr(0, "LINE_TOO_LONG");
    }
  }
}

void CommandInterface::handleLine(char *line, PendingCommand &pending, const SystemContext &ctx) {
  char *savePtr = nullptr;
  char *seqToken = strtok_r(line, " ", &savePtr);
  char *cmdToken = strtok_r(nullptr, " ", &savePtr);

  if (seqToken == nullptr || cmdToken == nullptr) {
    Protocol::sendErr(0, "BAD_FORMAT");
    return;
  }

  int32_t seq = 0;
  if (!parseSeq(seqToken, seq)) {
    Protocol::sendErr(0, "BAD_SEQ");
    return;
  }

  toUpperAscii(cmdToken);

  if (strcmp(cmdToken, "PING") == 0) {
    Protocol::sendOk(seq, "PONG=1");
    return;
  }

  if (strcmp(cmdToken, "VERSION") == 0) {
    Protocol::sendOk(seq, "FW=GEN1_DUE_V1");
    return;
  }

  if (strcmp(cmdToken, "STATUS") == 0) {
    Protocol::sendStatus(seq, ctx);
    return;
  }

  if (strcmp(cmdToken, "INIT") == 0) {
    if (ctx.state != STATE_BOOT && ctx.state != STATE_SAFE_IDLE) {
      Protocol::sendErr(seq, "ILLEGAL_STATE");
      return;
    }

    queueFlag(pending, &PendingCommand::hasInit);
    Protocol::sendOk(seq, "QUEUED=1");
    return;
  }

  if (strcmp(cmdToken, "RUN") == 0) {
    if (ctx.state != STATE_SAFE_IDLE) {
      Protocol::sendErr(seq, "ILLEGAL_STATE");
      return;
    }
    if (!isRunInterlockSafe(ctx)) {
      Protocol::sendErr(seq, "INTERLOCK");
      return;
    }

    RunProfile profile;
    if (!parseRunProfile(savePtr, profile)) {
      Protocol::sendErr(seq, "BAD_FORMAT");
      return;
    }

    queueRun(pending, profile);
    Protocol::sendOk(seq, "QUEUED=1");
    return;
  }

  if (strcmp(cmdToken, "ABORT") == 0) {
    if (!isAbortAllowed(ctx.state)) {
      Protocol::sendErr(seq, "ILLEGAL_STATE");
      return;
    }

    queueFlag(pending, &PendingCommand::hasAbort);
    Protocol::sendOk(seq, "QUEUED=1");
    return;
  }

  if (strcmp(cmdToken, "HARDSTOP") == 0) {
    queueFlag(pending, &PendingCommand::hasHardStop);
    Protocol::sendOk(seq, "QUEUED=1");
    return;
  }

  if (strcmp(cmdToken, "CLEAR_FAULT") == 0) {
    if (ctx.state != STATE_SAFE_IDLE) {
      Protocol::sendErr(seq, "ILLEGAL_STATE");
      return;
    }

    queueFlag(pending, &PendingCommand::hasClearFault);
    Protocol::sendOk(seq, "QUEUED=1");
    return;
  }

  if (strcmp(cmdToken, "LOCK") == 0) {
    if (ctx.state != STATE_SAFE_IDLE) {
      Protocol::sendErr(seq, "ILLEGAL_STATE");
      return;
    }

    queueFlag(pending, &PendingCommand::hasLock);
    Protocol::sendOk(seq, "QUEUED=1");
    return;
  }

  if (strcmp(cmdToken, "UNLOCK") == 0) {
    if (ctx.state != STATE_SAFE_IDLE || ctx.rpmCmd >= SAFE_UNLOCK_RPM) {
      Protocol::sendErr(seq, "ILLEGAL_STATE");
      return;
    }

    queueFlag(pending, &PendingCommand::hasUnlock);
    Protocol::sendOk(seq, "QUEUED=1");
    return;
  }

  if (strcmp(cmdToken, "DOOR_OPEN") == 0) {
    if (ctx.state != STATE_SAFE_IDLE || ctx.rpmCmd >= SAFE_UNLOCK_RPM) {
      Protocol::sendErr(seq, "ILLEGAL_STATE");
      return;
    }
    queueFlag(pending, &PendingCommand::hasDoorOpen);
    Protocol::sendOk(seq, "QUEUED=1");
    return;
  }

  if (strcmp(cmdToken, "DOOR_CLOSE") == 0) {
    if (ctx.state != STATE_SAFE_IDLE || ctx.rpmCmd >= SAFE_UNLOCK_RPM) {
      Protocol::sendErr(seq, "ILLEGAL_STATE");
      return;
    }
    queueFlag(pending, &PendingCommand::hasDoorClose);
    Protocol::sendOk(seq, "QUEUED=1");
    return;
  }

  Protocol::sendErr(seq, "UNKNOWN_CMD");
}

bool CommandInterface::parseRunProfile(char *savePtr, RunProfile &profile) {
  bool hasLift = false;
  bool hasFinal = false;
  bool hasSeat = false;
  bool hasHold = false;
  bool hasRampUp = false;
  bool hasRampDown = false;

  int32_t lift = 0;
  int32_t finalRpm = 0;
  int32_t seat = 0;
  int32_t hold = 0;
  int32_t rampUp = 0;
  int32_t rampDown = 0;

  char *token = strtok_r(nullptr, " ", &savePtr);
  while (token != nullptr) {
    int32_t value = 0;
    if (Protocol::parseKeyValueInt(token, "LIFT", value)) {
      lift = value;
      hasLift = true;
    } else if (Protocol::parseKeyValueInt(token, "FINAL", value)) {
      finalRpm = value;
      hasFinal = true;
    } else if (Protocol::parseKeyValueInt(token, "SEAT", value)) {
      seat = value;
      hasSeat = true;
    } else if (Protocol::parseKeyValueInt(token, "HOLD", value)) {
      hold = value;
      hasHold = true;
    } else if (Protocol::parseKeyValueInt(token, "RAMPUP", value)) {
      rampUp = value;
      hasRampUp = true;
    } else if (Protocol::parseKeyValueInt(token, "RAMPDOWN", value)) {
      rampDown = value;
      hasRampDown = true;
    } else {
      return false;
    }

    token = strtok_r(nullptr, " ", &savePtr);
  }

  if (!hasLift || !hasFinal || !hasSeat || !hasHold || !hasRampUp || !hasRampDown) {
    return false;
  }

  if (!validateDuration(seat) || !validateDuration(hold) || !validateDuration(rampUp) || !validateDuration(rampDown)) {
    return false;
  }

  profile.liftRpm = lift;
  profile.finalRpm = finalRpm;
  profile.seatMs = static_cast<uint32_t>(seat);
  profile.holdMs = static_cast<uint32_t>(hold);
  profile.rampUpMs = static_cast<uint32_t>(rampUp);
  profile.rampDownMs = static_cast<uint32_t>(rampDown);

  return true;
}

void CommandInterface::queueRun(PendingCommand &pending, const RunProfile &profile) {
  noInterrupts();
  pending.hasRunRequest = true;
  pending.runProfile = profile;
  interrupts();
}

void CommandInterface::queueFlag(PendingCommand &pending, bool PendingCommand::*flag) {
  noInterrupts();
  pending.*flag = true;
  interrupts();
}

bool CommandInterface::isAbortAllowed(SystemState state) const {
  return state == STATE_SPIN_HOLD || state == STATE_STOPPING;
}
