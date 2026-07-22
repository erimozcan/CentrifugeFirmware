#include "state_machine.h"

#include <string.h>

#include "hardware_guard.h"
#include "motor_interface.h"

namespace {

bool timeReached(uint32_t nowMs, uint32_t deadlineMs) {
  return static_cast<int32_t>(nowMs - deadlineMs) >= 0;
}

int32_t abs32(int32_t value) {
  return value < 0 ? -value : value;
}

bool hallActive(uint8_t raw, bool activeLow) {
  if (activeLow) {
    return raw == 0U;
  }
  return raw != 0U;
}

int32_t clampRpm(int32_t rpm) {
  if (rpm < 0) {
    return 0;
  }
  if (rpm > MAX_RPM_BAREBONES) {
    return MAX_RPM_BAREBONES;
  }
  return rpm;
}

uint32_t clampDuration(int32_t value) {
  if (value <= 0) {
    return 0U;
  }
  return static_cast<uint32_t>(value);
}

void sanitizeProfile(RunProfile &profile) {
  profile.liftRpm = clampRpm(profile.liftRpm);
  profile.finalRpm = clampRpm(profile.finalRpm);

  if (profile.finalRpm < profile.liftRpm) {
    profile.finalRpm = profile.liftRpm;
  }

  profile.seatMs = clampDuration(static_cast<int32_t>(profile.seatMs));
  profile.holdMs = clampDuration(static_cast<int32_t>(profile.holdMs));
  profile.rampUpMs = clampDuration(static_cast<int32_t>(profile.rampUpMs));
  profile.rampDownMs = clampDuration(static_cast<int32_t>(profile.rampDownMs));
}

bool isEscCalibrationActive(const SystemContext &ctx) {
  return strcmp(ctx.escCalState, "idle") != 0 &&
         strcmp(ctx.escCalState, "done") != 0 &&
         strcmp(ctx.escCalState, "failed") != 0;
}

bool isRunFanState(SystemState state) {
  switch (state) {
    case STATE_RUN_CLOSING:
    case STATE_RUN_RELEASE:
    case STATE_PRE_RUN_VERIFY:
    case STATE_LIFT_RAMP:
    case STATE_SEAT_HOLD:
    case STATE_MAIN_RAMP:
    case STATE_SPIN_HOLD:
    case STATE_STOPPING:
    case STATE_RUN_SETTLE:
    case STATE_RUN_INDEX:
    case STATE_RUN_ENGAGE:
      return true;
    default:
      return false;
  }
}

bool isRotateFanState(SystemState state) {
#if FAN_RUNS_DURING_ROTATE
  return state == STATE_ROTATE_RELEASE ||
         state == STATE_ROTATE_MOVING ||
         state == STATE_ROTATE_ENGAGE;
#else
  (void)state;
  return false;
#endif
}

bool isFanDemandActive(const SystemContext &ctx) {
  if (isRunFanState(ctx.state) || isRotateFanState(ctx.state)) {
    return true;
  }

#if FAN_RUNS_DURING_ESC_CAL
  if (isEscCalibrationActive(ctx)) {
    return true;
  }
#endif

  return abs32(ctx.rpm1) >= FAN_RPM_ACTIVE_THRESHOLD;
}

}  // namespace

void StateMachine::begin(SystemContext &ctx, uint32_t nowMs) {
  memset(&ctx, 0, sizeof(SystemContext));

  ctx.state = STATE_BOOT;
  ctx.fault = FAULT_NONE;
  ctx.faultLatched = false;
  fanDemandPreviously_ = false;
  fanCooldownUntilMs_ = nowMs;

  ctx.rpmInternal = 0;
  ctx.rpmInternalTarget = 0;
  ctx.rampStepInternal = 0;
  ctx.rpmCmd = 0;
  ctx.rpm1 = 0;
  strncpy(ctx.escCalState, "idle", sizeof(ctx.escCalState) - 1U);
  ctx.escCalState[sizeof(ctx.escCalState) - 1U] = '\0';
  ctx.escCalStep = 0;
  ctx.escCalTargetRpm = 0;
  ctx.escCurrentCentiamps = 0;
  ctx.escObserverRpm = 0;
  ctx.escObserverLock = 0U;
  ctx.escPhaseErrDeg10 = 0;
  ctx.escLoopHz = 0U;

  ctx.stateEntryMs = nowMs;
  ctx.seatDeadlineMs = nowMs;
  ctx.holdDeadlineMs = nowMs;

  ctx.commandOutputEnabled = true;
  ctx.lockActuatorCommanded = true;   // gantry starts LOCKED (indexing lock, rest position)
  ctx.lockConfirmed = false;
  ctx.lockAwaitingSensor = false;
  ctx.lockCommandStartMs = nowMs;
  ctx.doorState = DOOR_STATE_UNKNOWN;
  ctx.doorOpenSensorActive = false;
  ctx.doorClosedSensorActive = false;
  ctx.doorMotorCommand = DOOR_MOTOR_STOP;
  ctx.doorMoveActive = false;
  ctx.doorMoveDeadlineMs = nowMs;
  ctx.doorPwmDuty = DOOR_PWM_DUTY;
  ctx.rotateTube = 0U;
  ctx.currentTube = 1U;   // powered-on locked position is tube 1 (auto-homed there)
  ctx.rotateDeadlineMs = nowMs;
  ctx.ntcAdc = 0U;
  ctx.fanEnabled = false;
  ctx.overTempCritical = false;

  ctx.motorEnableOutput = false;

  ctx.activeProfile.liftRpm = 0;
  ctx.activeProfile.finalRpm = 0;
  ctx.activeProfile.seatMs = 0U;
  ctx.activeProfile.holdMs = 0U;
  ctx.activeProfile.rampUpMs = 0U;
  ctx.activeProfile.rampDownMs = 0U;
}

void StateMachine::tick(
    SystemContext &ctx,
    const SensorData &sensorSnapshot,
    PendingCommand &pending,
    MotorInterface &motor,
    HardwareGuard &guard,
    uint32_t nowMs) {
  ctx.rpm1 = sensorSnapshot.rpm1;
  strncpy(ctx.escCalState, sensorSnapshot.escCalState, sizeof(ctx.escCalState) - 1U);
  ctx.escCalState[sizeof(ctx.escCalState) - 1U] = '\0';
  ctx.escCalStep = sensorSnapshot.escCalStep;
  ctx.escCalTargetRpm = sensorSnapshot.escCalTargetRpm;
  ctx.escCurrentCentiamps = sensorSnapshot.escCurrentCentiamps;
  ctx.escObserverRpm = sensorSnapshot.escObserverRpm;
  ctx.escObserverLock = sensorSnapshot.escObserverLock;
  ctx.escPhaseErrDeg10 = sensorSnapshot.escPhaseErrDeg10;
  ctx.escLoopHz = sensorSnapshot.escLoopHz;
  updateInterlockState(ctx, sensorSnapshot);

  PendingCommand pendingLocal;
  noInterrupts();
  pendingLocal = pending;
  clearPendingCommand(pending);
  interrupts();

  applyPendingCommand(ctx, pendingLocal, nowMs);

  if (ctx.overTempCritical && isMotorEnabledState(ctx.state)) {
    enterHardStop(ctx, FAULT_OVER_TEMP, nowMs);
  }

  if (isMotorEnabledState(ctx.state) && ctx.doorState != DOOR_STATE_CLOSED) {
    enterHardStop(ctx, FAULT_DOOR_OPEN_DURING_SPIN, nowMs);
  }

  // NOTE: the gantry lock is RELEASED (not engaged) while spinning, so the old
  // "must be locked to spin" interlock is gone -- the lock is state-derived below.

  applyStateProgression(ctx, motor, nowMs);

  // Push the tuned closed-loop gains to the ESC once, on entry to PRE_RUN_VERIFY (interim,
  // until the ESC is reflashed with them). PRE_RUN_VERIFY precedes the first spin.
  if (ctx.needEscTuning) {
    motor.applyEscTuning();
    ctx.needEscTuning = false;
  }

  if (pendingLocal.hasHome) {
    motor.requestHome();   // (re)capture the detent reference from the current position
  }

  updateFixedPointRamp(ctx);

  ctx.rpmCmd = ctx.rpmInternal / RPM_SCALE;
  if (ctx.rpmCmd < 0) {
    ctx.rpmCmd = 0;
  }

  updateDoorMotorControl(ctx, nowMs);
  updateFanControl(ctx, nowMs);

  motor.setCommandOutputEnabled(ctx.commandOutputEnabled);
  bool indexingState = (ctx.state == STATE_ROTATE_MOVING || ctx.state == STATE_RUN_INDEX);
#ifdef ROTATE_VIA_INDEX
  // ESC-native closed-loop position (needs the updated ESC firmware).
  if (indexingState || ctx.state == STATE_ROTATE_ENGAGE) {
    // RUN_INDEX targets tube 1's detent (any detent is lockable); ROTATE targets its tube.
    uint8_t escTube = (ctx.state == STATE_RUN_INDEX) ? 0U : (ctx.rotateTube - 1U);
    motor.commandIndex(escTube);
  } else {
    motor.finishIndexIfNeeded();               // disarm the ESC angle-hold if we were indexing
    motor.maintainHome();                      // complete a pending detent-reference capture AND
                                               // sync the ESC's HOME -- this call was missing on
                                               // the INDEX path, so HOMED stayed 0 and every tube
                                               // target was ESC-power-on-relative (2026-07-22)
    motor.sendVelocitySetpoint(ctx.rpmCmd);
  }
#else
  // Nano closes the position loop over the ESC's existing SPIN + encoder telemetry.
  if (indexingState) {
    motor.updateVelocityRotate();
  } else {
    motor.endVelocityRotate();                 // stop + reset if a rotate was active
    motor.maintainHome();                      // complete a pending detent-reference capture
    motor.sendVelocitySetpoint(ctx.rpmCmd);
  }
#endif

  motor.drainRx();
  ctx.homed = motor.homeSet();

  guard.updateMotorEnable(ctx.state);
  ctx.motorEnableOutput = guard.motorEnableOutput();

  // Gantry lock is state-derived: locked at rest, released only across the spin window.
  ctx.lockActuatorCommanded = gantryLockEngaged(ctx.state);
  // Debug manual override (LOCK/UNLOCK) applies only while stopped; the auto policy governs
  // during any run/rotate.
  if (ctx.lockManualOverride && (ctx.state == STATE_SAFE_IDLE || ctx.state == STATE_BOOT)) {
    ctx.lockActuatorCommanded = ctx.lockManualEngaged;
  }
  // ESC calibration spins the spindle while the master stays in SAFE_IDLE/BOOT: keep the
  // gantry lock RELEASED for its whole duration (wins over the manual override). The ESC
  // dwells CAL_LOCK_RELEASE_MS before its alignment so the pin has time to retract.
  if (isEscCalibrationActive(ctx)) {
    ctx.lockActuatorCommanded = false;
  }
  // Safety net: never drive the lock into a still-turning rotor (e.g. coasting after an
  // E-STOP that jumped to a "locked" fault state) -- hold it released until near zero.
  if (ctx.rpm1 >= SAFE_UNLOCK_RPM) {
    ctx.lockActuatorCommanded = false;
  }

  guard.updateLockActuator(ctx.lockActuatorCommanded);
  guard.updateDoorMotor(ctx.doorMotorCommand, ctx.doorPwmDuty);
  guard.updateFan(ctx.fanEnabled);
}

bool StateMachine::isTransitionAllowed(SystemState from, SystemState to) {
  if (to == STATE_HARD_STOP) {
    return true;
  }

  switch (from) {
    case STATE_BOOT:
      return to == STATE_SAFE_IDLE ||
             to == STATE_ROTATE_RELEASE;  // ROTATE works from boot (like door/lock)
    case STATE_SAFE_IDLE:
      return to == STATE_RUN_CLOSING ||   // RUN starts the full cycle by closing the door
             to == STATE_ROTATE_RELEASE;  // ROTATE indexes the gantry to a tube
    case STATE_ROTATE_RELEASE:
      return to == STATE_ROTATE_MOVING;
    case STATE_ROTATE_MOVING:
      return to == STATE_ROTATE_ENGAGE;
    case STATE_ROTATE_ENGAGE:
      return to == STATE_SAFE_IDLE;
    case STATE_RUN_CLOSING:
      return to == STATE_RUN_RELEASE;
    case STATE_RUN_RELEASE:
      return to == STATE_PRE_RUN_VERIFY;
    case STATE_PRE_RUN_VERIFY:
      return to == STATE_LIFT_RAMP;
    case STATE_LIFT_RAMP:
      return to == STATE_SEAT_HOLD;
    case STATE_SEAT_HOLD:
      return to == STATE_MAIN_RAMP;
    case STATE_MAIN_RAMP:
      return to == STATE_SPIN_HOLD;
    case STATE_SPIN_HOLD:
      return to == STATE_STOPPING;
    case STATE_STOPPING:
      return to == STATE_RUN_SETTLE;    // post-spin unwind, not straight to idle
    case STATE_RUN_SETTLE:
      return to == STATE_RUN_INDEX;     // index to a detent before re-locking
    case STATE_RUN_INDEX:
      return to == STATE_RUN_ENGAGE;
    case STATE_RUN_ENGAGE:
      return to == STATE_RUN_OPENING;
    case STATE_RUN_OPENING:
      return to == STATE_SAFE_IDLE;
    case STATE_HARD_STOP:
      return to == STATE_FAULT_LATCHED;
    case STATE_FAULT_LATCHED:
      return false;
    default:
      return false;
  }
}

void StateMachine::applyPendingCommand(SystemContext &ctx, const PendingCommand &pendingLocal, uint32_t nowMs) {
  if (pendingLocal.hasHardStop) {
    enterHardStop(ctx, FAULT_HARDSTOP, nowMs);
    return;
  }

  if (pendingLocal.hasInit) {
    transitionTo(ctx, STATE_SAFE_IDLE, nowMs);
  }

  if (pendingLocal.hasClearFault) {
    if (ctx.faultLatched || ctx.state == STATE_HARD_STOP || ctx.state == STATE_FAULT_LATCHED) {
      // Recover from a latched fault: force a clean stop and drop straight to SAFE_IDLE
      // (bypasses the normal transition gate, which otherwise traps FAULT_LATCHED). The
      // ramp is zeroed so rpmCmd=0 -> the ESC gets STOP, never a re-spin.
      ctx.fault = FAULT_NONE;
      ctx.faultLatched = false;
      ctx.commandOutputEnabled = true;
      ctx.rpmInternal = 0;
      ctx.rpmInternalTarget = 0;
      ctx.rampStepInternal = 0;
      ctx.rpmCmd = 0;
      ctx.doorMotorCommand = DOOR_MOTOR_STOP;
      ctx.doorMoveActive = false;
      ctx.lockManualOverride = false;
      ctx.state = STATE_SAFE_IDLE;
      ctx.stateEntryMs = nowMs;
    } else {
      ctx.fault = FAULT_NONE;   // no latched fault -> just clear any stale code
    }
  }

  if (pendingLocal.hasRunRequest) {
    // The full cycle closes + locks itself, so we don't require the door pre-closed here.
    // Refuse only on conditions the choreography cannot fix: over-temp, or a door sensor
    // fault (both halls active at once).
    if (ctx.overTempCritical || ctx.doorState == DOOR_STATE_INVALID) {
      enterHardStop(ctx, FAULT_INTERLOCK_NOT_SAFE, nowMs);
      return;
    }

    ctx.activeProfile = pendingLocal.runProfile;
    sanitizeProfile(ctx.activeProfile);
    ctx.currentTube = 0U;   // a spin leaves the gantry at an unknown position
    ctx.lockManualOverride = false;   // auto policy governs the lock during a run
    transitionTo(ctx, STATE_RUN_CLOSING, nowMs);
  }

  // Index the gantry to a tube (only from SAFE_IDLE). The rotate choreography releases the
  // lock, has the ESC angle-move to the tube, then re-engages the lock.
  if (pendingLocal.hasRotate && (ctx.state == STATE_SAFE_IDLE || ctx.state == STATE_BOOT)) {
    uint8_t tube = pendingLocal.rotateTube;
    if (tube >= 1U && tube <= TUBE_COUNT) {
      ctx.rotateTube = tube;   // absolute target detent computed by motor.startRotateToTube()
      ctx.lockManualOverride = false;   // auto policy governs the lock during a rotate
      transitionTo(ctx, STATE_ROTATE_RELEASE, nowMs);
    }
  }

  if (pendingLocal.hasAbort) {
    transitionTo(ctx, STATE_STOPPING, nowMs);
  }

  // Manual LOCK/UNLOCK (debug panel): override the automatic lock policy while idle. The
  // override is cleared when a run/rotate starts (below), and the rpm safety net always wins.
  if (pendingLocal.hasLock) {
    ctx.lockManualOverride = true;
    ctx.lockManualEngaged = true;
  }
  if (pendingLocal.hasUnlock) {
    ctx.lockManualOverride = true;
    ctx.lockManualEngaged = false;
  }

  // Door moves key off the RAW hall sensors (doorOpenSensorActive/doorClosedSensorActive),
  // not doorState, so "move until the other signal is alive" works even if doorState is
  // INVALID/UNKNOWN mid-travel or forced by BYPASS_DOOR_INTERLOCK.
  if (pendingLocal.hasDoorStop) {
    ctx.doorMotorCommand = DOOR_MOTOR_STOP;
    ctx.doorMoveActive = false;
  }

  if (pendingLocal.hasDoorSpeed) {
    ctx.doorPwmDuty = pendingLocal.doorSpeed;   // live run-speed change (0-255)
  }

  if (pendingLocal.hasDoorOpen) {
    if (ctx.doorOpenSensorActive) {
      ctx.doorMotorCommand = DOOR_MOTOR_STOP;
      ctx.doorMoveActive = false;
    } else {
      ctx.doorMotorCommand = DOOR_MOTOR_OPEN;
      ctx.doorMoveActive = true;
      ctx.doorMoveDeadlineMs = nowMs + DOOR_MOVE_TIMEOUT_MS;
      if (!ctx.faultLatched && ctx.fault == FAULT_DOOR_MOVE_TIMEOUT) {
        ctx.fault = FAULT_NONE;   // clear a previous (non-latching) move-timeout note
      }
    }
  }

  if (pendingLocal.hasDoorClose) {
    if (ctx.doorClosedSensorActive) {
      ctx.doorMotorCommand = DOOR_MOTOR_STOP;
      ctx.doorMoveActive = false;
    } else {
      ctx.doorMotorCommand = DOOR_MOTOR_CLOSE;
      ctx.doorMoveActive = true;
      ctx.doorMoveDeadlineMs = nowMs + DOOR_MOVE_TIMEOUT_MS;
      if (!ctx.faultLatched && ctx.fault == FAULT_DOOR_MOVE_TIMEOUT) {
        ctx.fault = FAULT_NONE;
      }
    }
  }

  if (pendingLocal.hasLed) {
    ctx.ledR = pendingLocal.ledR;
    ctx.ledG = pendingLocal.ledG;
    ctx.ledB = pendingLocal.ledB;
    ctx.ledMode = pendingLocal.ledMode;
  }

  // Audio: hand manual play/volume requests to the AudioController mailbox (consumed
  // in loop()). Automatic event cues are observed there directly -- nothing to do here.
  if (pendingLocal.hasAudioPlay) {
    ctx.audioManualReq = pendingLocal.audioTrack;
  }
  if (pendingLocal.hasAudioVol) {
    ctx.audioVolPending = true;
    ctx.audioVol = pendingLocal.audioVol;
  }

  // Device master power (UI "Turn on"): powered -> fan runs continuously + LEDs rainbow;
  // off -> fan off + LEDs off. No temperature sensor is fitted.
  if (pendingLocal.hasPowerOn) {
    ctx.devicePowered = true;
    ctx.ledMode = LED_MODE_RAINBOW;
  }
  if (pendingLocal.hasPowerOff) {
    ctx.devicePowered = false;
    ctx.ledMode = LED_MODE_SOLID;
    ctx.ledR = 0;
    ctx.ledG = 0;
    ctx.ledB = 0;
  }
}

void StateMachine::applyStateProgression(SystemContext &ctx, MotorInterface &motor, uint32_t nowMs) {
  switch (ctx.state) {
    case STATE_RUN_CLOSING:
      // Wait for the door to reach CLOSED (updateDoorMotorControl drives + stops the motor).
      if (ctx.doorClosedSensorActive) {
        transitionTo(ctx, STATE_RUN_RELEASE, nowMs);
      } else if (timeReached(nowMs, ctx.doorMoveDeadlineMs)) {
        enterHardStop(ctx, FAULT_DOOR_MOVE_TIMEOUT, nowMs);
      }
      break;

    case STATE_RUN_RELEASE:
      // Gantry lock is released on entry (state-derived); wait for the actuator to retract
      // before the rotor starts to turn, then verify + spin.
      if (timeReached(nowMs, ctx.stateEntryMs + RUN_RELEASE_MS)) {
        transitionTo(ctx, STATE_PRE_RUN_VERIFY, nowMs);
      }
      break;

    case STATE_PRE_RUN_VERIFY:
      if (timeReached(nowMs, ctx.stateEntryMs + PRE_RUN_VERIFY_MS)) {
        transitionTo(ctx, STATE_LIFT_RAMP, nowMs);
      }
      break;

    case STATE_LIFT_RAMP:
      if (ctx.rpmInternal >= ctx.rpmInternalTarget) {
        transitionTo(ctx, STATE_SEAT_HOLD, nowMs);
      }
      break;

    case STATE_SEAT_HOLD:
      if (timeReached(nowMs, ctx.seatDeadlineMs)) {
        transitionTo(ctx, STATE_MAIN_RAMP, nowMs);
      }
      break;

    case STATE_MAIN_RAMP:
      if (ctx.rpmInternal >= ctx.rpmInternalTarget) {
        transitionTo(ctx, STATE_SPIN_HOLD, nowMs);
      }
      break;

    case STATE_SPIN_HOLD:
      if (timeReached(nowMs, ctx.holdDeadlineMs)) {
        transitionTo(ctx, STATE_STOPPING, nowMs);
      }
      break;

    case STATE_STOPPING:
      if (ctx.rpmInternal <= 0) {
        transitionTo(ctx, STATE_RUN_SETTLE, nowMs);
      }
      break;

    case STATE_RUN_SETTLE:
      // Dwell at rest so the rotor is truly stopped, then index to the nearest detent so
      // the lock can mesh (a spin stops the rotor at a random, non-lockable angle).
      if (timeReached(nowMs, ctx.stateEntryMs + RUN_SETTLE_MS)) {
#ifndef ROTATE_VIA_INDEX
        motor.startRotateToNearestDetent();
#endif
        ctx.rotateDeadlineMs = nowMs + ROTATE_MOVE_TIMEOUT_MS;
        transitionTo(ctx, STATE_RUN_INDEX, nowMs);
      }
      break;

    case STATE_RUN_INDEX:
      // Crawl to the nearest detent (tick() drives it), then re-engage the lock.
#ifdef ROTATE_VIA_INDEX
      if (motor.indexArrived()) {
#else
      if (motor.velocityRotateArrived()) {
#endif
        ctx.currentTube = motor.arrivedTube();
        transitionTo(ctx, STATE_RUN_ENGAGE, nowMs);
      } else if (timeReached(nowMs, ctx.rotateDeadlineMs)) {
        enterHardStop(ctx, FAULT_ROTATE_TIMEOUT, nowMs);
      }
      break;

    case STATE_RUN_ENGAGE:
      // Unlock is commanded on entry; give the actuator time to retract, then open.
      if (timeReached(nowMs, ctx.stateEntryMs + RUN_ENGAGE_MS)) {
        transitionTo(ctx, STATE_RUN_OPENING, nowMs);
      }
      break;

    case STATE_RUN_OPENING:
      if (ctx.doorOpenSensorActive) {
        transitionTo(ctx, STATE_SAFE_IDLE, nowMs);
      } else if (timeReached(nowMs, ctx.doorMoveDeadlineMs)) {
        enterHardStop(ctx, FAULT_DOOR_MOVE_TIMEOUT, nowMs);
      }
      break;

    case STATE_ROTATE_RELEASE:
      // Lock releases on entry (state-derived); wait for the actuator to retract, then move.
      if (timeReached(nowMs, ctx.stateEntryMs + RUN_RELEASE_MS)) {
#ifndef ROTATE_VIA_INDEX
        motor.startRotateToTube(ctx.rotateTube);   // absolute crawl to the tube's detent
#endif
        transitionTo(ctx, STATE_ROTATE_MOVING, nowMs);
      }
      break;

    case STATE_ROTATE_MOVING:
      // tick() drives the move (ESC INDEX, or Nano SPIN+encoder); advance on arrival.
#ifdef ROTATE_VIA_INDEX
      if (motor.indexArrived()) {
#else
      if (motor.velocityRotateArrived()) {
#endif
        ctx.currentTube = ctx.rotateTube;
        transitionTo(ctx, STATE_ROTATE_ENGAGE, nowMs);
      } else if (timeReached(nowMs, ctx.rotateDeadlineMs)) {
        enterHardStop(ctx, FAULT_ROTATE_TIMEOUT, nowMs);
      }
      break;

    case STATE_ROTATE_ENGAGE:
      // Lock re-engages on entry (state-derived); dwell for the actuator to seat, then the
      // gantry holds the tube and tick() disarms the ESC hold on the way back to idle.
      if (timeReached(nowMs, ctx.stateEntryMs + RUN_ENGAGE_MS)) {
        ctx.currentTube = ctx.rotateTube;   // now parked at this tube
        transitionTo(ctx, STATE_SAFE_IDLE, nowMs);
      }
      break;

    case STATE_HARD_STOP:
      transitionTo(ctx, STATE_FAULT_LATCHED, nowMs);
      break;

    case STATE_BOOT:
    case STATE_SAFE_IDLE:
    case STATE_FAULT_LATCHED:
    default:
      break;
  }
}

void StateMachine::updateInterlockState(SystemContext &ctx, const SensorData &sensorSnapshot) {
  ctx.lockConfirmed = (sensorSnapshot.lockSensor != 0U);
  ctx.ntcAdc = sensorSnapshot.ntcAdc;
  ctx.overTempCritical = ctx.ntcAdc <= NTC_ADC_CRITICAL_LOW;

  bool openActive = hallActive(sensorSnapshot.doorOpenHall, DOOR_OPEN_ACTIVE_LOW != 0);
  bool closedActive = hallActive(sensorSnapshot.doorClosedHall, DOOR_CLOSED_ACTIVE_LOW != 0);

  ctx.doorOpenSensorActive = openActive;
  ctx.doorClosedSensorActive = closedActive;
  ctx.doorOpenHallRaw = sensorSnapshot.doorOpenHall;
  ctx.doorClosedHallRaw = sensorSnapshot.doorClosedHall;

  if (openActive && !closedActive) {
    ctx.doorState = DOOR_STATE_OPEN;
  } else if (closedActive && !openActive) {
    ctx.doorState = DOOR_STATE_CLOSED;
  } else if (openActive && closedActive) {
    ctx.doorState = DOOR_STATE_INVALID;
  } else {
    ctx.doorState = DOOR_STATE_UNKNOWN;
  }

#ifdef BYPASS_DOOR_INTERLOCK
  // BENCH BRING-UP ONLY: no door hardware yet, so force the door "closed" for the
  // spin interlocks. Remove this flag once the door + hall sensors are wired.
  ctx.doorState = DOOR_STATE_CLOSED;
#endif
}

void StateMachine::updateDoorMotorControl(SystemContext &ctx, uint32_t nowMs) {
  if (!ctx.doorMoveActive) {
    return;
  }

  // Stop the instant the TARGET hall goes active -- "move until the other signal is alive".
  if (ctx.doorMotorCommand == DOOR_MOTOR_OPEN && ctx.doorOpenSensorActive) {
    ctx.doorMotorCommand = DOOR_MOTOR_STOP;
    ctx.doorMoveActive = false;
    return;
  }

  if (ctx.doorMotorCommand == DOOR_MOTOR_CLOSE && ctx.doorClosedSensorActive) {
    ctx.doorMotorCommand = DOOR_MOTOR_STOP;
    ctx.doorMoveActive = false;
    return;
  }

  // Timeout: the target hall never tripped. Door moves only happen while the rotor is
  // stopped, so this is a benign "didn't reach the endpoint" -- abandon the move and note
  // it (non-latching), do NOT hard-stop the machine. The operator can just retry.
  if (timeReached(nowMs, ctx.doorMoveDeadlineMs)) {
    ctx.doorMotorCommand = DOOR_MOTOR_STOP;
    ctx.doorMoveActive = false;
    if (!ctx.faultLatched) {
      ctx.fault = FAULT_DOOR_MOVE_TIMEOUT;
    }
  }
}

void StateMachine::updateFanControl(SystemContext &ctx, uint32_t nowMs) {
  // POWER_OFF is authoritative: no run/cooldown/calibration demand may keep the fan on.
  if (!ctx.devicePowered) {
    fanDemandPreviously_ = false;
    fanCooldownUntilMs_ = nowMs;
    ctx.fanEnabled = false;
    return;
  }

  bool fanDemand = isFanDemandActive(ctx);
  if (fanDemand) {
    fanCooldownUntilMs_ = nowMs + FAN_COOLDOWN_MS;
  } else if (fanDemandPreviously_) {
    fanCooldownUntilMs_ = nowMs + FAN_COOLDOWN_MS;
  }

  fanDemandPreviously_ = fanDemand;
  bool cooldownActive = !timeReached(nowMs, fanCooldownUntilMs_);
  ctx.fanEnabled = fanDemand || cooldownActive;
}

bool StateMachine::isRunInterlockSafe(const SystemContext &ctx) const {
  return ctx.doorState == DOOR_STATE_CLOSED && ctx.lockConfirmed && !ctx.overTempCritical;
}

void StateMachine::updateFixedPointRamp(SystemContext &ctx) {
  if (ctx.rampStepInternal == 0 || ctx.rpmInternal == ctx.rpmInternalTarget) {
    return;
  }

  int32_t next = ctx.rpmInternal + ctx.rampStepInternal;

  if (ctx.rampStepInternal > 0 && next > ctx.rpmInternalTarget) {
    next = ctx.rpmInternalTarget;
  }
  if (ctx.rampStepInternal < 0 && next < ctx.rpmInternalTarget) {
    next = ctx.rpmInternalTarget;
  }

  ctx.rpmInternal = next;

  if (ctx.rpmInternal == ctx.rpmInternalTarget) {
    ctx.rampStepInternal = 0;
  }
}

void StateMachine::transitionTo(SystemContext &ctx, SystemState next, uint32_t nowMs) {
  if (ctx.state == next) {
    return;
  }

  if (!isTransitionAllowed(ctx.state, next)) {
    enterHardStop(ctx, FAULT_ILLEGAL_STATE, nowMs);
    return;
  }

  ctx.state = next;
  ctx.stateEntryMs = nowMs;

  switch (ctx.state) {
    case STATE_SAFE_IDLE:
      setRampTarget(ctx, 0, 0);
      ctx.commandOutputEnabled = true;
      ctx.doorMotorCommand = DOOR_MOTOR_STOP;
      ctx.doorMoveActive = false;
      break;

    case STATE_RUN_CLOSING:
      setRampTarget(ctx, 0, 0);
      ctx.commandOutputEnabled = true;
      if (ctx.doorClosedSensorActive) {
        ctx.doorMotorCommand = DOOR_MOTOR_STOP;   // already closed -> skip on next tick
        ctx.doorMoveActive = false;
      } else {
        ctx.doorMotorCommand = DOOR_MOTOR_CLOSE;
        ctx.doorMoveActive = true;
        ctx.doorMoveDeadlineMs = nowMs + DOOR_MOVE_TIMEOUT_MS;
      }
      break;

    case STATE_RUN_RELEASE:
      // Door is closed; the gantry lock releases here (state-derived) so the rotor can spin.
      ctx.doorMotorCommand = DOOR_MOTOR_STOP;
      ctx.doorMoveActive = false;
      break;

    case STATE_PRE_RUN_VERIFY:
      setRampTarget(ctx, 0, 0);
      ctx.doorMotorCommand = DOOR_MOTOR_STOP;
      ctx.doorMoveActive = false;
      ctx.needEscTuning = true;   // push tuned gains before the first spin
      break;

    case STATE_LIFT_RAMP:
      setRampTarget(ctx, ctx.activeProfile.liftRpm, ctx.activeProfile.rampUpMs);
      break;

    case STATE_SEAT_HOLD:
      setRampTarget(ctx, ctx.activeProfile.liftRpm, 0);
      ctx.seatDeadlineMs = nowMs + ctx.activeProfile.seatMs;
      break;

    case STATE_MAIN_RAMP:
      setRampTarget(ctx, ctx.activeProfile.finalRpm, ctx.activeProfile.rampUpMs);
      break;

    case STATE_SPIN_HOLD:
      setRampTarget(ctx, ctx.activeProfile.finalRpm, 0);
      ctx.holdDeadlineMs = nowMs + ctx.activeProfile.holdMs;
      break;

    case STATE_STOPPING:
      setRampTarget(ctx, 0, ctx.activeProfile.rampDownMs);
      break;

    case STATE_RUN_SETTLE:
      setRampTarget(ctx, 0, 0);   // ensure the target is a hard stop
      ctx.doorMotorCommand = DOOR_MOTOR_STOP;
      ctx.doorMoveActive = false;
      break;

    case STATE_RUN_ENGAGE:
      // Rotor is stopped; the gantry lock re-engages here (state-derived). Dwell lets the
      // actuator extend before the door opens.
      break;

    case STATE_RUN_OPENING:
      if (ctx.doorOpenSensorActive) {
        ctx.doorMotorCommand = DOOR_MOTOR_STOP;   // already open -> skip on next tick
        ctx.doorMoveActive = false;
      } else {
        ctx.doorMotorCommand = DOOR_MOTOR_OPEN;
        ctx.doorMoveActive = true;
        ctx.doorMoveDeadlineMs = nowMs + DOOR_MOVE_TIMEOUT_MS;
      }
      break;

    case STATE_ROTATE_RELEASE:
      // Gantry lock releases here (state-derived) so the rotor can be indexed.
      setRampTarget(ctx, 0, 0);
      ctx.commandOutputEnabled = true;
      ctx.doorMotorCommand = DOOR_MOTOR_STOP;
      ctx.doorMoveActive = false;
      break;

    case STATE_ROTATE_MOVING:
      // tick() streams INDEX to the ESC; arm the arrival timeout.
      ctx.rotateDeadlineMs = nowMs + ROTATE_MOVE_TIMEOUT_MS;
      break;

    case STATE_ROTATE_ENGAGE:
      // Gantry lock re-engages here (state-derived) now that the tube is reached.
      break;

    case STATE_HARD_STOP:
      ctx.commandOutputEnabled = false;
      ctx.faultLatched = true;
      ctx.rpmInternalTarget = ctx.rpmInternal;
      ctx.rampStepInternal = 0;
      ctx.doorMotorCommand = DOOR_MOTOR_STOP;
      ctx.doorMoveActive = false;
      break;

    case STATE_FAULT_LATCHED:
      ctx.commandOutputEnabled = false;
      ctx.faultLatched = true;
      ctx.rpmInternalTarget = ctx.rpmInternal;
      ctx.rampStepInternal = 0;
      ctx.doorMotorCommand = DOOR_MOTOR_STOP;
      ctx.doorMoveActive = false;
      break;

    case STATE_BOOT:
    default:
      break;
  }
}

void StateMachine::enterHardStop(SystemContext &ctx, FaultCode reason, uint32_t nowMs) {
  ctx.fault = reason;
  ctx.faultLatched = true;
  transitionTo(ctx, STATE_HARD_STOP, nowMs);
}

void StateMachine::setRampTarget(SystemContext &ctx, int32_t rpmTarget, uint32_t durationMs) {
  int32_t clampedRpm = clampRpm(rpmTarget);
  int32_t targetInternal = clampedRpm * RPM_SCALE;
  int32_t delta = targetInternal - ctx.rpmInternal;

  ctx.rpmInternalTarget = targetInternal;

  if (delta == 0) {
    ctx.rampStepInternal = 0;
    return;
  }

  if (durationMs == 0U) {
    ctx.rampStepInternal = delta;
    return;
  }

  int32_t ticks = static_cast<int32_t>(durationMs);
  int32_t step = delta / ticks;

  if (step == 0) {
    step = (delta > 0) ? 1 : -1;
  }

  ctx.rampStepInternal = step;
}
