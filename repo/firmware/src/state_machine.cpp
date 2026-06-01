#include "state_machine.h"

#include "hardware_guard.h"
#include "motor_interface.h"

namespace {

bool timeReached(uint32_t nowMs, uint32_t deadlineMs) {
  return static_cast<int32_t>(nowMs - deadlineMs) >= 0;
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

}  // namespace

void StateMachine::begin(SystemContext &ctx, uint32_t nowMs) {
  memset(&ctx, 0, sizeof(SystemContext));

  ctx.state = STATE_BOOT;
  ctx.fault = FAULT_NONE;
  ctx.faultLatched = false;

  ctx.rpmInternal = 0;
  ctx.rpmInternalTarget = 0;
  ctx.rampStepInternal = 0;
  ctx.rpmCmd = 0;
  ctx.rpm1 = 0;

  ctx.stateEntryMs = nowMs;
  ctx.seatDeadlineMs = nowMs;
  ctx.holdDeadlineMs = nowMs;

  ctx.commandOutputEnabled = true;
  ctx.lockActuatorCommanded = false;
  ctx.lockConfirmed = false;
  ctx.lockAwaitingSensor = false;
  ctx.lockCommandStartMs = nowMs;
  ctx.doorState = DOOR_STATE_UNKNOWN;
  ctx.doorOpenSensorActive = false;
  ctx.doorClosedSensorActive = false;
  ctx.doorMotorCommand = DOOR_MOTOR_STOP;
  ctx.doorMoveActive = false;
  ctx.doorMoveDeadlineMs = nowMs;
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

  if (isMotorEnabledState(ctx.state) && !ctx.lockConfirmed) {
    enterHardStop(ctx, FAULT_INTERLOCK_NOT_SAFE, nowMs);
  }

  if (ctx.lockActuatorCommanded && !ctx.lockConfirmed && ctx.lockAwaitingSensor) {
    if (timeReached(nowMs, ctx.lockCommandStartMs + LOCK_ENGAGE_TIMEOUT_MS)) {
      enterHardStop(ctx, FAULT_LOCK_TIMEOUT, nowMs);
    }
  }
  if (ctx.lockConfirmed) {
    ctx.lockAwaitingSensor = false;
  }

  applyStateProgression(ctx, nowMs);

  updateFixedPointRamp(ctx);

  ctx.rpmCmd = ctx.rpmInternal / RPM_SCALE;
  if (ctx.rpmCmd < 0) {
    ctx.rpmCmd = 0;
  }

  updateDoorMotorControl(ctx, nowMs);
  updateFanControl(ctx);

  motor.setCommandOutputEnabled(ctx.commandOutputEnabled);
  motor.sendVelocitySetpoint(ctx.rpmCmd);

  motor.drainRx();

  guard.updateMotorEnable(ctx.state);
  ctx.motorEnableOutput = guard.motorEnableOutput();

  guard.updateLockActuator(ctx.lockActuatorCommanded);
  guard.updateDoorMotor(ctx.doorMotorCommand);
  guard.updateFan(ctx.fanEnabled);
}

bool StateMachine::isTransitionAllowed(SystemState from, SystemState to) {
  if (to == STATE_HARD_STOP) {
    return true;
  }

  switch (from) {
    case STATE_BOOT:
      return to == STATE_SAFE_IDLE;
    case STATE_SAFE_IDLE:
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
    if (ctx.state == STATE_SAFE_IDLE) {
      ctx.fault = FAULT_NONE;
      ctx.faultLatched = false;
    }
  }

  if (pendingLocal.hasRunRequest) {
    if (!isRunInterlockSafe(ctx)) {
      enterHardStop(ctx, FAULT_INTERLOCK_NOT_SAFE, nowMs);
      return;
    }

    ctx.activeProfile = pendingLocal.runProfile;
    sanitizeProfile(ctx.activeProfile);
    transitionTo(ctx, STATE_PRE_RUN_VERIFY, nowMs);
  }

  if (pendingLocal.hasAbort) {
    transitionTo(ctx, STATE_STOPPING, nowMs);
  }

  if (pendingLocal.hasLock) {
    ctx.lockActuatorCommanded = true;
    ctx.lockAwaitingSensor = true;
    ctx.lockCommandStartMs = nowMs;
  }

  if (pendingLocal.hasUnlock) {
    ctx.lockActuatorCommanded = false;
    ctx.lockAwaitingSensor = false;
  }

  if (pendingLocal.hasDoorOpen) {
    if (ctx.doorState == DOOR_STATE_OPEN) {
      ctx.doorMotorCommand = DOOR_MOTOR_STOP;
      ctx.doorMoveActive = false;
    } else {
      ctx.doorMotorCommand = DOOR_MOTOR_OPEN;
      ctx.doorMoveActive = true;
      ctx.doorMoveDeadlineMs = nowMs + DOOR_MOVE_TIMEOUT_MS;
    }
  }

  if (pendingLocal.hasDoorClose) {
    if (ctx.doorState == DOOR_STATE_CLOSED) {
      ctx.doorMotorCommand = DOOR_MOTOR_STOP;
      ctx.doorMoveActive = false;
    } else {
      ctx.doorMotorCommand = DOOR_MOTOR_CLOSE;
      ctx.doorMoveActive = true;
      ctx.doorMoveDeadlineMs = nowMs + DOOR_MOVE_TIMEOUT_MS;
    }
  }
}

void StateMachine::applyStateProgression(SystemContext &ctx, uint32_t nowMs) {
  switch (ctx.state) {
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

  if (openActive && !closedActive) {
    ctx.doorState = DOOR_STATE_OPEN;
  } else if (closedActive && !openActive) {
    ctx.doorState = DOOR_STATE_CLOSED;
  } else if (openActive && closedActive) {
    ctx.doorState = DOOR_STATE_INVALID;
  } else {
    ctx.doorState = DOOR_STATE_UNKNOWN;
  }
}

void StateMachine::updateDoorMotorControl(SystemContext &ctx, uint32_t nowMs) {
  if (!ctx.doorMoveActive) {
    return;
  }

  if (ctx.doorMotorCommand == DOOR_MOTOR_OPEN && ctx.doorState == DOOR_STATE_OPEN) {
    ctx.doorMotorCommand = DOOR_MOTOR_STOP;
    ctx.doorMoveActive = false;
    return;
  }

  if (ctx.doorMotorCommand == DOOR_MOTOR_CLOSE && ctx.doorState == DOOR_STATE_CLOSED) {
    ctx.doorMotorCommand = DOOR_MOTOR_STOP;
    ctx.doorMoveActive = false;
    return;
  }

  if (timeReached(nowMs, ctx.doorMoveDeadlineMs)) {
    ctx.doorMotorCommand = DOOR_MOTOR_STOP;
    ctx.doorMoveActive = false;
    enterHardStop(ctx, FAULT_DOOR_MOVE_TIMEOUT, nowMs);
  }
}

void StateMachine::updateFanControl(SystemContext &ctx) {
  if (ctx.ntcAdc <= NTC_ADC_FAN_ON) {
    ctx.fanEnabled = true;
    return;
  }

  if (ctx.ntcAdc >= NTC_ADC_FAN_OFF) {
    ctx.fanEnabled = false;
  }
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

    case STATE_PRE_RUN_VERIFY:
      setRampTarget(ctx, 0, 0);
      ctx.doorMotorCommand = DOOR_MOTOR_STOP;
      ctx.doorMoveActive = false;
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
