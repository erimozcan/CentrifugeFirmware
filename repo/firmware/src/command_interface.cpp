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

uint8_t clampByte(int32_t value) {
  if (value < 0) {
    return 0U;
  }
  if (value > 255) {
    return 255U;
  }
  return static_cast<uint8_t>(value);
}

// A run is refused only for conditions the automated cycle cannot resolve itself: an
// over-temp, or a door-sensor fault (both halls active). The cycle closes + locks the
// door on its own, so it does NOT need the door pre-closed/locked here.
bool isRunStartAllowed(const SystemContext &ctx) {
  return !ctx.overTempCritical && ctx.doorState != DOOR_STATE_INVALID;
}

// Lock/unlock is allowed while the machine is stopped: at BOOT (so the actuator can
// be driven from the UI straight after flashing, before INIT) and in SAFE_IDLE. It
// is refused in every spin/ramp state so the gantry can't move while the rotor turns.
bool isLockControlState(SystemState state) {
  return state == STATE_BOOT || state == STATE_SAFE_IDLE;
}

// Door open/close follows the same stopped-only rule as the lock: BOOT or SAFE_IDLE,
// never while spinning. Allowing BOOT lets the door be driven from the UI straight
// after flashing (no INIT first), which matches how bring-up is done.
bool isDoorControlState(SystemState state) {
  return state == STATE_BOOT || state == STATE_SAFE_IDLE;
}

bool isEscServiceState(const SystemContext &ctx) {
  return (ctx.state == STATE_BOOT || ctx.state == STATE_SAFE_IDLE) &&
         ctx.rpmCmd < SAFE_UNLOCK_RPM &&
         ctx.rpm1 < SAFE_UNLOCK_RPM;
}

void forwardEscServiceCommand(int32_t seq, const char *line) {
  Serial1.print(line);
  Serial1.print('\n');
  Protocol::sendOk(seq, "FWD=1");
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

  if (strcmp(cmdToken, "CAL_START") == 0) {
    if (!isEscServiceState(ctx)) {
      Protocol::sendErr(seq, "ILLEGAL_STATE");
      return;
    }
    forwardEscServiceCommand(seq, "CAL START");
    return;
  }

  if (strcmp(cmdToken, "CAL_STOP") == 0) {
    if (!isEscServiceState(ctx)) {
      Protocol::sendErr(seq, "ILLEGAL_STATE");
      return;
    }
    forwardEscServiceCommand(seq, "CAL STOP");
    return;
  }

  if (strcmp(cmdToken, "CAL_STATUS") == 0) {
    if (!isEscServiceState(ctx)) {
      Protocol::sendErr(seq, "ILLEGAL_STATE");
      return;
    }
    forwardEscServiceCommand(seq, "CAL STATUS?");
    return;
  }

  if (strcmp(cmdToken, "CAL_APPLY") == 0) {
    if (!isEscServiceState(ctx)) {
      Protocol::sendErr(seq, "ILLEGAL_STATE");
      return;
    }
    forwardEscServiceCommand(seq, "CAL APPLY");
    return;
  }

  if (strcmp(cmdToken, "PROFILE_SPIN") == 0) {
    if (!isEscServiceState(ctx)) {
      Protocol::sendErr(seq, "ILLEGAL_STATE");
      return;
    }
    forwardEscServiceCommand(seq, "PROFILE SPIN");
    return;
  }

  if (strcmp(cmdToken, "PROFILE_CRAWL") == 0) {
    if (!isEscServiceState(ctx)) {
      Protocol::sendErr(seq, "ILLEGAL_STATE");
      return;
    }
    forwardEscServiceCommand(seq, "PROFILE CRAWL");
    return;
  }

  if (strcmp(cmdToken, "TUNE_STATUS") == 0 || strcmp(cmdToken, "TUNE?") == 0) {
    if (!isEscServiceState(ctx)) {
      Protocol::sendErr(seq, "ILLEGAL_STATE");
      return;
    }
    forwardEscServiceCommand(seq, "TUNE?");
    return;
  }

#ifdef ESC_DEBUG_RELAY
  // Debug passthrough: forward the rest of the line verbatim to the ESC's bench
  // command parser (e.g. "ESCRAW 2" / "ESCRAW l 1.5" / "ESCRAW v 8"). Lets us drive
  // the ESC open-loop for diagnostics without a separate serial link.
  if (strcmp(cmdToken, "ESCRAW") == 0) {
    if (savePtr != nullptr && *savePtr != '\0') {
      Serial1.print(savePtr);
      Serial1.print('\n');
    }
    Protocol::sendOk(seq, "FWD=1");
    return;
  }
#endif

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
    if (!isRunStartAllowed(ctx)) {
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

  // CLEAR_FAULT recovers from a LATCHED fault (e.g. an imbalance E-STOP) back to a clean
  // SAFE_IDLE -- no power-cycle needed. Allowed from any state; it forces a clean stop.
  if (strcmp(cmdToken, "CLEAR_FAULT") == 0) {
    queueFlag(pending, &PendingCommand::hasClearFault);
    Protocol::sendOk(seq, "QUEUED=1");
    return;
  }

  if (strcmp(cmdToken, "LOCK") == 0) {
    if (!isLockControlState(ctx.state)) {
      Protocol::sendErr(seq, "ILLEGAL_STATE");
      return;
    }

    queueFlag(pending, &PendingCommand::hasLock);
    Protocol::sendOk(seq, "QUEUED=1");
    return;
  }

  if (strcmp(cmdToken, "UNLOCK") == 0) {
    if (!isLockControlState(ctx.state) || ctx.rpmCmd >= SAFE_UNLOCK_RPM) {
      Protocol::sendErr(seq, "ILLEGAL_STATE");
      return;
    }

    queueFlag(pending, &PendingCommand::hasUnlock);
    Protocol::sendOk(seq, "QUEUED=1");
    return;
  }

  if (strcmp(cmdToken, "DOOR_OPEN") == 0) {
    if (!isDoorControlState(ctx.state) || ctx.rpmCmd >= SAFE_UNLOCK_RPM) {
      Protocol::sendErr(seq, "ILLEGAL_STATE");
      return;
    }
    queueFlag(pending, &PendingCommand::hasDoorOpen);
    Protocol::sendOk(seq, "QUEUED=1");
    return;
  }

  if (strcmp(cmdToken, "DOOR_CLOSE") == 0) {
    if (!isDoorControlState(ctx.state) || ctx.rpmCmd >= SAFE_UNLOCK_RPM) {
      Protocol::sendErr(seq, "ILLEGAL_STATE");
      return;
    }
    queueFlag(pending, &PendingCommand::hasDoorClose);
    Protocol::sendOk(seq, "QUEUED=1");
    return;
  }

  // Stop an in-progress door move. Always safe -> allowed in any state.
  if (strcmp(cmdToken, "DOOR_STOP") == 0) {
    queueFlag(pending, &PendingCommand::hasDoorStop);
    Protocol::sendOk(seq, "QUEUED=1");
    return;
  }

  // DOOR_SPEED <0-255>: set the slow-run PWM duty. Cosmetic/tuning -> allowed any state;
  // takes effect live if the door is moving.
  if (strcmp(cmdToken, "DOOR_SPEED") == 0) {
    char *valToken = strtok_r(nullptr, " ", &savePtr);
    int32_t value = 0;
    if (valToken == nullptr || !Protocol::parseInt32(valToken, value)) {
      Protocol::sendErr(seq, "BAD_FORMAT");
      return;
    }
    noInterrupts();
    pending.hasDoorSpeed = true;
    pending.doorSpeed = clampByte(value);
    interrupts();
    Protocol::sendOk(seq, "QUEUED=1");
    return;
  }

  // ROTATE <1-TUBE_COUNT>: index the gantry to a tube position. Allowed while stopped
  // (BOOT or SAFE_IDLE), like the door/lock -- no Init needed.
  if (strcmp(cmdToken, "ROTATE") == 0) {
    if (!isDoorControlState(ctx.state)) {
      Protocol::sendErr(seq, "ILLEGAL_STATE");
      return;
    }
    char *valToken = strtok_r(nullptr, " ", &savePtr);
    int32_t tube = 0;
    if (valToken == nullptr || !Protocol::parseInt32(valToken, tube) ||
        tube < 1 || tube > static_cast<int32_t>(TUBE_COUNT)) {
      Protocol::sendErr(seq, "BAD_FORMAT");
      return;
    }
    noInterrupts();
    pending.hasRotate = true;
    pending.rotateTube = static_cast<uint8_t>(tube);
    interrupts();
    Protocol::sendOk(seq, "QUEUED=1");
    return;
  }

  // HOME: capture the current shaft angle as the tube-1 detent reference. Press with the
  // gantry physically sitting in a detent. Allowed while stopped.
  if (strcmp(cmdToken, "HOME") == 0) {
    if (!isDoorControlState(ctx.state)) {
      Protocol::sendErr(seq, "ILLEGAL_STATE");
      return;
    }
    queueFlag(pending, &PendingCommand::hasHome);
    Protocol::sendOk(seq, "QUEUED=1");
    return;
  }

  if (strcmp(cmdToken, "POWER_ON") == 0) {
    queueFlag(pending, &PendingCommand::hasPowerOn);
    Protocol::sendOk(seq, "QUEUED=1");
    return;
  }

  if (strcmp(cmdToken, "POWER_OFF") == 0) {
    queueFlag(pending, &PendingCommand::hasPowerOff);
    Protocol::sendOk(seq, "QUEUED=1");
    return;
  }

  if (strcmp(cmdToken, "LED") == 0) {
    // LED R=<0-255> G=<0-255> B=<0-255> [MODE=<0 solid|1 rainbow>]. RGB alone implies
    // solid; MODE alone toggles the effect and keeps the last color. Cosmetic -> allowed
    // in any state.
    int32_t r = ctx.ledR;
    int32_t g = ctx.ledG;
    int32_t b = ctx.ledB;
    int32_t mode = ctx.ledMode;
    bool gotRgb = false;
    bool gotMode = false;

    char *token = strtok_r(nullptr, " ", &savePtr);
    while (token != nullptr) {
      int32_t value = 0;
      if (Protocol::parseKeyValueInt(token, "R", value)) {
        r = value;
        gotRgb = true;
      } else if (Protocol::parseKeyValueInt(token, "G", value)) {
        g = value;
        gotRgb = true;
      } else if (Protocol::parseKeyValueInt(token, "B", value)) {
        b = value;
        gotRgb = true;
      } else if (Protocol::parseKeyValueInt(token, "MODE", value)) {
        mode = value;
        gotMode = true;
      } else {
        Protocol::sendErr(seq, "BAD_FORMAT");
        return;
      }
      token = strtok_r(nullptr, " ", &savePtr);
    }

    if (!gotRgb && !gotMode) {
      Protocol::sendErr(seq, "BAD_FORMAT");
      return;
    }
    if (gotRgb && !gotMode) {
      mode = LED_MODE_SOLID;
    }
    if (mode < 0 || mode > LED_MODE_RAINBOW) {
      mode = LED_MODE_SOLID;
    }

    noInterrupts();
    pending.hasLed = true;
    pending.ledR = clampByte(r);
    pending.ledG = clampByte(g);
    pending.ledB = clampByte(b);
    pending.ledMode = static_cast<uint8_t>(mode);
    interrupts();

    Protocol::sendOk(seq, "LED=1");
    return;
  }

  // AUDIO <n>: play clip number n now (bench/manual test of the file mapping).
  // Cosmetic -> allowed in any state. Fire-and-forget in the AudioController.
  if (strcmp(cmdToken, "AUDIO") == 0) {
    char *valToken = strtok_r(nullptr, " ", &savePtr);
    int32_t track = 0;
    if (valToken == nullptr || !Protocol::parseInt32(valToken, track) || track < 1) {
      Protocol::sendErr(seq, "BAD_FORMAT");
      return;
    }
    noInterrupts();
    pending.hasAudioPlay = true;
    pending.audioTrack = clampByte(track);
    interrupts();
    Protocol::sendOk(seq, "QUEUED=1");
    return;
  }

  // AUDIO_VOL <0-30>: set playback volume. Cosmetic -> allowed in any state.
  if (strcmp(cmdToken, "AUDIO_VOL") == 0) {
    char *valToken = strtok_r(nullptr, " ", &savePtr);
    int32_t vol = 0;
    if (valToken == nullptr || !Protocol::parseInt32(valToken, vol) || vol < 0 || vol > 30) {
      Protocol::sendErr(seq, "BAD_FORMAT");
      return;
    }
    noInterrupts();
    pending.hasAudioVol = true;
    pending.audioVol = static_cast<uint8_t>(vol);
    interrupts();
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
