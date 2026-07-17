#include "motor_interface.h"

#include <stdlib.h>   // atol
#include <string.h>   // strncmp, strstr

#include "config.h"

void MotorInterface::begin() {
#if defined(ARDUINO_ARCH_ESP32)
  // ESP32 UART1 must be told which GPIOs to use (matrix-routed). Give it a real TX
  // ring buffer so availableForWrite() reports true free space and writes don't
  // stall the tick (default under-reports, which silently dropped SPIN commands).
  Serial1.setTxBufferSize(256);
  Serial1.begin(ESC_UART_BAUD, SERIAL_8N1, PIN_ESC_UART_RX, PIN_ESC_UART_TX);
#else
  Serial1.begin(ESC_UART_BAUD);
#endif
  commandOutputEnabled_ = true;
  lastCommandedRpm_ = 0;
  intent_ = INTENT_IDLE;
  lastSentRpm_ = -1;
  lastTxMs_ = 0U;
  rxLen_ = 0U;
  measuredRpm_ = 0;
  lastRxMs_ = 0U;
  strncpy(escCalState_, "idle", sizeof(escCalState_) - 1U);
  escCalState_[sizeof(escCalState_) - 1U] = '\0';
  escCalStep_ = 0;
  escCalTargetRpm_ = 0;
  escCurrentCentiamps_ = 0;
  escObserverRpm_ = 0;
  escObserverLock_ = 0U;
  escPhaseErrDeg10_ = 0;
  escLoopHz_ = 0U;
  indexActive_ = false;
  indexArrived_ = false;
  lastIndexTube_ = -1;
  rotPhase_ = ROT_INACTIVE;
  homeSet_ = false;
  homePending_ = true;   // auto-capture the detent reference from the startup (homed) position
}

void MotorInterface::setCommandOutputEnabled(bool enabled) {
  commandOutputEnabled_ = enabled;
}

// Map the state machine's per-tick ramped setpoint onto the ESC verb protocol.
// Called at ~1 kHz; actual UART writes are paced so we never flood the link and
// always keep the ESC's comms watchdog fed.
void MotorInterface::sendVelocitySetpoint(int32_t rpm) {
  if (rpm < 0) {
    rpm = 0;
  }
  if (rpm > MAX_RPM_BAREBONES) {
    rpm = MAX_RPM_BAREBONES;
  }
  lastCommandedRpm_ = rpm;

  // Desired ESC intent: torque-off on fault/hard-stop, spin if commanded > 0,
  // otherwise idle (stopped + disarmed).
  EscIntent want;
  if (!commandOutputEnabled_) {
    want = INTENT_ESTOP;
  } else if (rpm > 0) {
    want = INTENT_SPIN;
  } else {
    want = INTENT_IDLE;
  }

  uint32_t now = millis();

  // Intent transition: act immediately (the action verb), bypassing the rate cap
  // so STOP/ESTOP are prompt and the first SPIN arms the ESC without delay.
  if (want != intent_) {
    switch (want) {
      case INTENT_SPIN:  txSpin(rpm);        break;
      case INTENT_IDLE:  txLiteral("STOP");  break;
      case INTENT_ESTOP: txLiteral("ESTOP"); break;
    }
    intent_ = want;
    lastSentRpm_ = rpm;
    lastTxMs_ = now;
    return;
  }

  // Steady intent: rate-limited setpoint updates + watchdog heartbeat.
  if (want == INTENT_SPIN) {
    bool changed = (rpm != lastSentRpm_);
    uint32_t interval = changed ? ESC_TX_MIN_MS : ESC_TX_HEARTBEAT_MS;
    if ((uint32_t)(now - lastTxMs_) >= interval) {
      txSpin(rpm);
      lastSentRpm_ = rpm;
      lastTxMs_ = now;
    }
  } else {
    // Idle or latched-ESTOP: a periodic PING keeps the link alive without
    // re-arming or re-triggering the action verb.
    if ((uint32_t)(now - lastTxMs_) >= ESC_TX_HEARTBEAT_MS) {
      txLiteral("PING");
      lastTxMs_ = now;
    }
  }
}

// Drive the ESC's closed-loop angle move to a tube. Send INDEX once (or when the target
// changes), then keep the comms watchdog fed with PING while the ESC positions + holds.
void MotorInterface::commandIndex(uint8_t escTube) {
  uint32_t now = millis();
  if (!indexActive_ || static_cast<int32_t>(escTube) != lastIndexTube_) {
    txIndex(escTube);
    indexActive_ = true;
    indexArrived_ = false;
    lastIndexTube_ = static_cast<int32_t>(escTube);
    intent_ = INTENT_IDLE;   // velocity intent is stale while indexing
    lastTxMs_ = now;
    return;
  }
  if ((uint32_t)(now - lastTxMs_) >= ESC_TX_HEARTBEAT_MS) {
    if (indexArrived_) {
      txLiteral("PING");     // feed the ESC watchdog while it holds the angle
    } else {
      txIndex(escTube);      // re-send until arrival: INDEX is idempotent, and a single
                             // dropped UART line must not strand the move into a timeout
    }
    lastTxMs_ = now;
  }
}

// Leaving an index: tell the ESC to disarm the angle hold (the gantry lock now holds it).
void MotorInterface::finishIndexIfNeeded() {
  if (!indexActive_) {
    return;
  }
  txLiteral("STOP");
  indexActive_ = false;
  indexArrived_ = false;
  lastIndexTube_ = -1;
  intent_ = INTENT_IDLE;     // so the next sendVelocitySetpoint() doesn't re-send STOP
  lastTxMs_ = millis();
}

bool MotorInterface::indexArrived() const {
  return indexArrived_;
}

// ---- Absolute detent indexing (works with the ESC firmware already flashed) ----------
// The Nano commands SPIN (velocity) and reads the ESC's absolute shaft angle from its STAT
// line (ang=, requested with "?"). It crawls FORWARD (the current ESC's SPIN is one-way)
// until the angle reaches the target DETENT, then stops; the lock's taper does the final
// few degrees. Detents are learned relative to a homed reference (detentRef_).
namespace {
float mod1(float x) { x -= floorf(x); return (x < 0.0f) ? x + 1.0f : x; }          // -> [0,1)
float circDist(float a, float b) { float d = fabsf(mod1(a) - mod1(b)); return (d > 0.5f) ? 1.0f - d : d; }

const char *findValue(const char *line, const char *key) {
  const char *p = strstr(line, key);
  if (p == nullptr) {
    return nullptr;
  }
  return p + strlen(key);
}

void copyToken(char *out, size_t outLen, const char *value) {
  if (outLen == 0U || value == nullptr) {
    return;
  }
  size_t i = 0U;
  while (value[i] != '\0' && value[i] != ' ' && i < outLen - 1U) {
    out[i] = value[i];
    i++;
  }
  out[i] = '\0';
}
}  // namespace

void MotorInterface::requestHome() {
  homeSet_ = false;
  homePending_ = true;
  rotSeen_ = false;
  rotPollMs_ = 0U;
}

void MotorInterface::maintainHome() {
  if (!homePending_) {
    return;
  }
  if (rotSeen_) {                 // a fresh angle arrived -> that position is detent 0 (tube 1)
    detentRef_ = measuredFrac_;
    homeSet_ = true;
    homePending_ = false;
#ifdef ROTATE_VIA_INDEX
    txLiteral("HOME");            // sync the ESC's own detent reference to this position
#endif
    return;
  }
  uint32_t now = millis();
  if ((uint32_t)(now - rotPollMs_) >= ROTATE_POLL_MS) {
    txLiteral("?");
    rotPollMs_ = now;
  }
}

bool MotorInterface::homeSet() const { return homeSet_; }

uint8_t MotorInterface::arrivedTube() const {
  if (!homeSet_) {
    return 0U;
  }
  float rel = mod1(measuredFrac_ - detentRef_) * static_cast<float>(TUBE_COUNT);
  uint8_t t = static_cast<uint8_t>(lroundf(rel)) % TUBE_COUNT;   // 0..TUBE_COUNT-1
  return t + 1U;
}

// Boost the ESC's velocity PID P so the crawl has torque at low speed (the soft spin gains
// barely command current at a crawl -> stalls on the assembly's stiction). Restored by
// applyEscTuning() when the move ends.
void MotorInterface::applyRotateTuning() {
  txLiteral("PROFILE CRAWL");
}

void MotorInterface::startRotateToTube(uint8_t tube) {
  rotNearest_ = false;
  rotTargetFrac_ = mod1(detentRef_ + static_cast<float>(tube - 1U) / static_cast<float>(TUBE_COUNT));
  rotPhase_ = ROT_ACQUIRE;
  rotSeen_ = false;
  rotPollMs_ = 0U;
  applyRotateTuning();
}

void MotorInterface::startRotateToNearestDetent() {
  rotNearest_ = true;
  rotPhase_ = ROT_ACQUIRE;
  rotSeen_ = false;
  rotPollMs_ = 0U;
  applyRotateTuning();
}

void MotorInterface::updateVelocityRotate() {
  uint32_t now = millis();
  if ((uint32_t)(now - rotPollMs_) >= ROTATE_POLL_MS) {
    txLiteral("?");             // request a STAT line (parsed into measuredFrac_/measuredRev_)
    rotPollMs_ = now;
  }

  int32_t rpm = 0;
  if (rotSeen_) {
    // Distance (revs, circular) from the current angle to the target detent.
    float dist;
    if (rotNearest_) {
      dist = 1.0f;
      for (uint8_t k = 0U; k < TUBE_COUNT; k++) {
        float det = mod1(detentRef_ + static_cast<float>(k) / static_cast<float>(TUBE_COUNT));
        float d = circDist(measuredFrac_, det);
        if (d < dist) dist = d;
      }
    } else {
      dist = circDist(measuredFrac_, rotTargetFrac_);
    }

    switch (rotPhase_) {
      case ROT_ACQUIRE:
        rotLastRev_ = measuredRev_;
        rotStableMs_ = now;
        rotPhase_ = (dist < ROTATE_ARRIVE_TOL_REV) ? ROT_SETTLE : ROT_CRAWL;   // maybe already there
        break;
      case ROT_CRAWL:
        if (dist < ROTATE_ARRIVE_TOL_REV) {
          rotLastRev_ = measuredRev_;
          rotStableMs_ = now;
          rotPhase_ = ROT_SETTLE;   // reached the detent window -> stop + settle
        } else {
          // Two-speed: fast to break stiction + cover ground, slow for a precise landing.
          rpm = (dist > ROTATE_COARSE_TOL_REV) ? ROTATE_FAST_RPM : ROTATE_SLOW_RPM;
        }
        break;
      case ROT_SETTLE:
        if (fabsf(measuredRev_ - rotLastRev_) > ROTATE_STABLE_REV) {
          rotLastRev_ = measuredRev_;   // still moving -> restart the stable timer
          rotStableMs_ = now;
        } else if ((uint32_t)(now - rotStableMs_) >= ROTATE_SETTLE_MS) {
          rotPhase_ = ROT_DONE;
        }
        break;
      case ROT_DONE:
      case ROT_INACTIVE:
      default:
        break;
    }
  }
  sendVelocitySetpoint(rpm);   // SPIN at the crawl or STOP; also feeds the ESC watchdog
}

bool MotorInterface::velocityRotateArrived() const {
  return rotPhase_ == ROT_DONE;
}

void MotorInterface::endVelocityRotate() {
  if (rotPhase_ == ROT_INACTIVE) {
    return;
  }
  rotPhase_ = ROT_INACTIVE;
  sendVelocitySetpoint(0);   // ensure the ESC is stopped
  applyEscTuning();          // restore the soft spin gains (undo the crawl P boost)
}

void MotorInterface::txIndex(uint8_t escTube) {
  char cmd[ESC_UART_CMD_MAX_LEN];
  size_t idx = 0U;
  cmd[idx++] = 'I';
  cmd[idx++] = 'N';
  cmd[idx++] = 'D';
  cmd[idx++] = 'E';
  cmd[idx++] = 'X';
  cmd[idx++] = ' ';
  idx += writeInt32(&cmd[idx], sizeof(cmd) - idx, static_cast<int32_t>(escTube));
  if (idx >= sizeof(cmd) - 1U) {
    return;
  }
  cmd[idx++] = '\n';
  Serial1.write(reinterpret_cast<const uint8_t *>(cmd), idx);
}

void MotorInterface::applyEscTuning() {
  // The ESC now owns named PID/current profiles. The master only requests the mode
  // it needs instead of pushing raw bench parser gains.
  txLiteral("PROFILE SPIN");
}

void MotorInterface::txSpin(int32_t rpm) {
  char cmd[ESC_UART_CMD_MAX_LEN];
  size_t idx = 0U;
  cmd[idx++] = 'S';
  cmd[idx++] = 'P';
  cmd[idx++] = 'I';
  cmd[idx++] = 'N';
  cmd[idx++] = ' ';
  idx += writeInt32(&cmd[idx], sizeof(cmd) - idx, rpm);
  if (idx >= sizeof(cmd) - 1U) {
    return;
  }
  cmd[idx++] = '\n';

  // With the TX ring buffer sized above, this queues without blocking; never gate
  // on availableForWrite() (it under-reports on ESP32 and silently drops the line).
  Serial1.write(reinterpret_cast<const uint8_t *>(cmd), idx);
}

void MotorInterface::txLiteral(const char *verb) {
  size_t len = strlen(verb);
  if (len + 1U > ESC_UART_CMD_MAX_LEN) {
    return;
  }
  Serial1.write(reinterpret_cast<const uint8_t *>(verb), len);
  Serial1.write(static_cast<uint8_t>('\n'));
}

// Drain ESC -> Due bytes, assembling complete lines and parsing ST telemetry.
void MotorInterface::drainRx() {
  while (Serial1.available()) {
    char c = static_cast<char>(Serial1.read());
    if (c == '\n' || c == '\r') {
      if (rxLen_ > 0U) {
        rxBuf_[rxLen_] = '\0';
#ifdef ESC_DEBUG_RELAY
        Serial.print("ESC ");     // bring-up: mirror the ESC's serial to the USB console
        Serial.println(rxBuf_);
#endif
        parseEscLine(rxBuf_);
        rxLen_ = 0U;
      }
    } else if (rxLen_ < sizeof(rxBuf_) - 1U) {
      rxBuf_[rxLen_++] = c;
    } else {
      rxLen_ = 0U;  // overflow: drop the malformed line
    }
  }
}

// ESC telemetry looks like: "ST rpm=4000 tgt=4000 state=spin cur=1.20".
// Pull out the measured RPM; ignore everything else (acks, diagnostics).
void MotorInterface::parseEscLine(const char *line) {
  // Bench STAT line (requested with "?"): pull the cumulative revs for the rotate loop.
  // Checked before "ST " since "STAT " also begins with "ST".
  if (strncmp(line, "STAT ", 5) == 0) {
    const char *a = strstr(line, "ang=");   // absolute shaft angle [0,2pi) -> frac [0,1)
    if (a != nullptr) {
      measuredFrac_ = static_cast<float>(atof(a + 4)) * (1.0f / 6.2831853f);
      rotSeen_ = true;
    }
    const char *r = strstr(line, "rev=");   // cumulative revs (motion/settle detection)
    if (r != nullptr) {
      measuredRev_ = static_cast<float>(atof(r + 4));
    }
    return;
  }

  if (strncmp(line, "ST ", 3) != 0) {
    return;
  }
  const char *p = strstr(line, "rpm=");
  if (p == nullptr) {
    return;
  }
  measuredRpm_ = static_cast<int32_t>(atol(p + 4));
  lastRxMs_ = millis();

  const char *cal = findValue(line, "cal=");
  if (cal != nullptr) {
    copyToken(escCalState_, sizeof(escCalState_), cal);
  }
  const char *calStep = findValue(line, "cal_step=");
  if (calStep != nullptr) {
    escCalStep_ = static_cast<int32_t>(atol(calStep));
  }
  const char *calTarget = findValue(line, "cal_target=");
  if (calTarget != nullptr) {
    escCalTargetRpm_ = static_cast<int32_t>(atol(calTarget));
  }
  const char *cur = findValue(line, "cur=");
  if (cur != nullptr) {
    escCurrentCentiamps_ = static_cast<int32_t>(atof(cur) * 100.0f);
  }
  const char *obsRpm = findValue(line, "obs_rpm=");
  if (obsRpm != nullptr) {
    escObserverRpm_ = static_cast<int32_t>(atol(obsRpm));
  }
  const char *obsLock = findValue(line, "obs_lock=");
  if (obsLock != nullptr) {
    escObserverLock_ = static_cast<uint8_t>(atol(obsLock) != 0 ? 1U : 0U);
  }
  const char *phaseErr = findValue(line, "phase_err=");
  if (phaseErr != nullptr) {
    escPhaseErrDeg10_ = static_cast<int32_t>(atof(phaseErr) * 10.0f);
  }
  const char *hz = findValue(line, "hz=");
  if (hz != nullptr) {
    escLoopHz_ = static_cast<uint32_t>(atol(hz));
  }

  // The ST stream also carries the absolute shaft angle (pos=, degrees [0,360)). Feed it
  // into the same slot the STAT poll fills, so home capture and arrivedTube() stay fresh
  // at 10 Hz without extra "?" polling (the ESC-native INDEX path never polls STAT).
  const char *pos = findValue(line, "pos=");
  if (pos != nullptr) {
    measuredFrac_ = static_cast<float>(atof(pos)) * (1.0f / 360.0f);
    rotSeen_ = true;
  }

  // Index arrival: the ESC reports state=indexed once it has reached + is holding the tube.
  if (indexActive_) {
    indexArrived_ = (strstr(line, "state=indexed") != nullptr);
  }
}

int32_t MotorInterface::lastMeasuredRpm() const {
  return measuredRpm_;
}

void MotorInterface::copyEscCalState(char *out, size_t maxLen) const {
  copyToken(out, maxLen, escCalState_);
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
