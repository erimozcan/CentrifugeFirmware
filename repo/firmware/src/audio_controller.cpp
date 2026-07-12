#include "audio_controller.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(HAS_AUDIO) && HAS_AUDIO

#include <DFRobot_DF1201S.h>

// Handshake with the module and push one-time config. Blocks on the module's ACKs
// (up to ~1 s each, plus switchFunction's internal 1.5 s delay). Returns true if the
// module answered. After this the module stays in MUSIC mode and we drive playback
// with raw AT writes. Callers must only invoke this when it's safe to block (setup or
// an idle state) -- never during a spin.
bool AudioController::tryInit() {
  DFRobot_DF1201S df;
  if (!df.begin(Serial2)) {
    return false;   // no ACK: module not powered / not wired / wrong baud
  }
  df.switchFunction(DFRobot_DF1201S::MUSIC);  // U-disk music mode
  df.setPrompt(false);                        // no power-on/insert prompt tones
  df.setLED(false);                           // quiet the module's indicator LED
  df.setPlayMode(DFRobot_DF1201S::SINGLE);    // play one clip and stop (no looping)
  df.enableAMP();                             // built-in speaker amplifier on
  df.setVol(AUDIO_VOLUME);
  ready_ = true;
  return true;
}

void AudioController::begin() {
  // Serial2 (UART2) routed to the DFPlayer pins. 115200 = the PRO's default.
  Serial2.begin(AUDIO_UART_BAUD, SERIAL_8N1, PIN_DFPLAYER_RX, PIN_DFPLAYER_TX);
  // NON-BLOCKING: do NOT stall setup() handshaking the module here. tryInit() blocks ~1 s
  // per attempt on a missing ACK (and the module's 5 V rail may be off), which would delay
  // the WiFi server + control loop from starting -> the UI wouldn't load for seconds after
  // power-on. service() auto-retries init while idle, so audio self-heals within a few
  // seconds without holding up boot.
}

namespace {
// Map a tube number (1..TUBE_COUNT) to its "tube N selected" clip. 0 if out of range.
uint8_t tubeClip(uint8_t tube) {
  switch (tube) {
    case 1: return AUDIO_FILE_TUBE1;
    case 2: return AUDIO_FILE_TUBE2;
    case 3: return AUDIO_FILE_TUBE3;
    case 4: return AUDIO_FILE_TUBE4;
    default: return 0U;
  }
}
}  // namespace

void AudioController::enqueue(uint8_t fileNum) {
  // 0 = "no clip for this event" (see the AUDIO_FILE_* map) -> nothing to queue.
  if (!ready_ || fileNum == 0U) {
    return;
  }
  if (qCount_ >= kQueueLen) {
    return;   // queue full (a very long burst) -> drop the newest cue
  }
  queue_[(qHead_ + qCount_) % kQueueLen] = fileNum;
  ++qCount_;
}

void AudioController::playFile(uint8_t fileNum) {
  if (!ready_ || fileNum == 0U) {
    return;
  }
  // Fire-and-forget AT+PLAYNUM=<n> (non-blocking). The module's "OK" reply is drained
  // in service(); a new PLAYNUM simply interrupts any clip still playing.
  Serial2.print(F("AT+PLAYNUM="));
  Serial2.print(static_cast<int>(fileNum));
  Serial2.print(F("\r\n"));
}

void AudioController::setVolume(uint8_t vol) {
  if (!ready_) {
    return;
  }
  if (vol > 30U) {
    vol = 30U;
  }
  Serial2.print(F("AT+VOL="));
  Serial2.print(static_cast<int>(vol));
  Serial2.print(F("\r\n"));
}

void AudioController::service(SystemContext &ctx, uint32_t nowMs) {
  // Not up yet? Keep retrying, but ONLY while the machine is idle (BOOT/SAFE_IDLE) --
  // tryInit() can block ~1 s on a missing ACK, which must never happen during a spin.
  // This lets audio come alive on its own once the module's 5 V rail is powered.
  if (!ready_) {
    bool idle = (ctx.state == STATE_BOOT || ctx.state == STATE_SAFE_IDLE);
    if (idle && static_cast<uint32_t>(nowMs - lastInitMs_) >= AUDIO_INIT_IDLE_RETRY_MS) {
      lastInitMs_ = nowMs;
      tryInit();
    }
    ctx.audioReady = ready_;
    if (!ready_) {
      return;
    }
    // Just came up: fall through so the primed_ pass captures the current baseline.
  }
  ctx.audioReady = ready_;

  // Discard the module's ACK/echo bytes so the UART RX buffer never backs up.
  while (Serial2.available() > 0) {
    Serial2.read();
  }

  // First pass: capture the baseline so the initial boot state (e.g. lock engaged)
  // doesn't fire a spurious cue. No sounds until we've seen one real transition.
  if (!primed_) {
    lastState_ = ctx.state;
    lastDoorCmd_ = ctx.doorMotorCommand;
    lastLock_ = lockCandidate_ = ctx.lockActuatorCommanded;
    lockChangeMs_ = nowMs;
    lastPowered_ = ctx.devicePowered;
    inFault_ = (ctx.state == STATE_HARD_STOP || ctx.state == STATE_FAULT_LATCHED);
    primed_ = true;
    return;
  }

  // ---- Manual requests (bench/UI) ------------------------------------------ (always work)
  if (ctx.audioVolPending) {
    setVolume(ctx.audioVol);
    ctx.audioVolPending = false;
  }
  if (ctx.audioManualReq != 0U) {
    enqueue(ctx.audioManualReq);
    ctx.audioManualReq = 0U;
  }

  // ---- Fault handling ------------------------------------------------------
  // On an E-STOP/fault, FLUSH any pending narration immediately (the operator stopped the
  // machine -- don't keep talking). While faulted, suppress all automatic cues but keep the
  // edge trackers current so recovery is clean. On CLEAR (fault -> not fault), flush again and
  // re-baseline so none of the fault-recovery transitions (lock re-engage, state hop) fire.
  const bool fault = (ctx.state == STATE_HARD_STOP || ctx.state == STATE_FAULT_LATCHED);
  if (fault != inFault_) {
    qCount_ = 0;
    qHead_ = 0;
    inFault_ = fault;
    if (!fault) {   // just cleared -> clean slate, emit nothing this pass
      lastState_ = ctx.state;
      lastDoorCmd_ = ctx.doorMotorCommand;
      lastLock_ = lockCandidate_ = ctx.lockActuatorCommanded;
      lockChangeMs_ = nowMs;
      lastPowered_ = ctx.devicePowered;
      return;
    }
  }
  if (fault) {
    // Keep trackers synced to "now" (so exit is silent) but narrate nothing.
    lastState_ = ctx.state;
    lastDoorCmd_ = ctx.doorMotorCommand;
    lastLock_ = lockCandidate_ = ctx.lockActuatorCommanded;
    lockChangeMs_ = nowMs;
    lastPowered_ = ctx.devicePowered;
    return;
  }

  // ---- Automatic event cues (edge-triggered) -------------------------------
  // Door motor direction edge -> covers both manual DOOR_OPEN/CLOSE and the RUN cycle.
  if (ctx.doorMotorCommand != lastDoorCmd_) {
    if (ctx.doorMotorCommand == DOOR_MOTOR_CLOSE) {
      enqueue(AUDIO_FILE_DOOR_CLOSING);
    } else if (ctx.doorMotorCommand == DOOR_MOTOR_OPEN) {
      enqueue(AUDIO_FILE_DOOR_OPENING);
    }
    lastDoorCmd_ = ctx.doorMotorCommand;
  }

  // Gantry lock edge (state-derived), DEBOUNCED: only announce once the lock has held its
  // new value for AUDIO_LOCK_CUE_DEBOUNCE_MS, so rpm-jitter flips near SAFE_UNLOCK_RPM
  // (e.g. a coasting rotor) never spam locking/unlocking clips.
  if (ctx.lockActuatorCommanded != lockCandidate_) {
    lockCandidate_ = ctx.lockActuatorCommanded;
    lockChangeMs_ = nowMs;
  }
  if (lockCandidate_ != lastLock_ &&
      static_cast<uint32_t>(nowMs - lockChangeMs_) >= AUDIO_LOCK_CUE_DEBOUNCE_MS) {
    enqueue(lockCandidate_ ? AUDIO_FILE_LOCKING : AUDIO_FILE_UNLOCKING);
    lastLock_ = lockCandidate_;
  }

  // Master power on.
  if (ctx.devicePowered != lastPowered_) {
    if (ctx.devicePowered) {
      enqueue(AUDIO_FILE_POWER_ON);
    }
    lastPowered_ = ctx.devicePowered;
  }

  // Spin / rotate / fault phases, keyed off state entry.
  if (ctx.state != lastState_) {
    switch (ctx.state) {
      case STATE_LIFT_RAMP:
        enqueue(AUDIO_FILE_SPINNING_UP);
        break;
      case STATE_SPIN_HOLD:
        enqueue(AUDIO_FILE_AT_SPEED);
        break;
      case STATE_STOPPING:
        enqueue(AUDIO_FILE_SLOWING);
        break;
      case STATE_ROTATE_MOVING:
        // Announce the specific tube being selected (ctx.rotateTube = the target).
        enqueue(tubeClip(ctx.rotateTube));
        break;
      case STATE_HARD_STOP:
        enqueue(AUDIO_FILE_FAULT);
        break;
      case STATE_SAFE_IDLE:
        // Reaching idle at the END of a run cycle = run complete.
        if (lastState_ == STATE_RUN_OPENING) {
          enqueue(AUDIO_FILE_RUN_COMPLETE);
        }
        break;
      default:
        break;
    }
    lastState_ = ctx.state;
  }

  // Release one queued clip at a time, spaced by AUDIO_CUE_GAP_MS, so a burst of
  // near-simultaneous cues plays back cleanly instead of cutting each other off.
  if (qCount_ > 0U &&
      static_cast<uint32_t>(nowMs - lastPlayMs_) >= AUDIO_CUE_GAP_MS) {
    playFile(queue_[qHead_]);
    qHead_ = (qHead_ + 1U) % kQueueLen;
    --qCount_;
    lastPlayMs_ = nowMs;
  }
}

#else  // no DFPlayer on this target (e.g. the Due prototype): no-op stubs

void AudioController::begin() {}
void AudioController::service(SystemContext &ctx, uint32_t) { ctx.audioReady = false; }

#endif
