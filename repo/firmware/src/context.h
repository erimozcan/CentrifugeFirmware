#ifndef CONTEXT_H
#define CONTEXT_H

#include <Arduino.h>
#include <string.h>

#include "config.h"

enum SystemState : uint8_t {
  STATE_BOOT = 0,
  STATE_SAFE_IDLE,
  STATE_PRE_RUN_VERIFY,
  STATE_LIFT_RAMP,
  STATE_SEAT_HOLD,
  STATE_MAIN_RAMP,
  STATE_SPIN_HOLD,
  STATE_STOPPING,
  STATE_HARD_STOP,
  STATE_FAULT_LATCHED
};

enum FaultCode : uint8_t {
  FAULT_NONE = 0,
  FAULT_ILLEGAL_STATE = 1,
  FAULT_BAD_COMMAND = 2,
  FAULT_HARDSTOP = 3,
  FAULT_LOCK_TIMEOUT = 4,
  FAULT_DOOR_INVALID = 5,
  FAULT_DOOR_OPEN_DURING_SPIN = 6,
  FAULT_DOOR_MOVE_TIMEOUT = 7,
  FAULT_OVER_TEMP = 8,
  FAULT_INTERLOCK_NOT_SAFE = 9
};

enum DoorState : uint8_t {
  DOOR_STATE_UNKNOWN = 0,
  DOOR_STATE_OPEN = 1,
  DOOR_STATE_CLOSED = 2,
  DOOR_STATE_INVALID = 3
};

enum DoorMotorCommand : int8_t {
  DOOR_MOTOR_STOP = 0,
  DOOR_MOTOR_OPEN = -1,
  DOOR_MOTOR_CLOSE = 1
};

struct RunProfile {
  int32_t liftRpm;
  int32_t finalRpm;
  uint32_t seatMs;
  uint32_t holdMs;
  uint32_t rampUpMs;
  uint32_t rampDownMs;
};

struct PendingCommand {
  bool hasRunRequest;
  RunProfile runProfile;

  bool hasAbort;
  bool hasHardStop;
  bool hasClearFault;

  bool hasInit;
  bool hasLock;
  bool hasUnlock;
  bool hasDoorOpen;
  bool hasDoorClose;
};

struct SensorData {
  int32_t rpm1;
  uint8_t lockSensor;
  uint8_t doorOpenHall;
  uint8_t doorClosedHall;
  uint16_t ntcAdc;
};

struct SystemContext {
  SystemState state;
  FaultCode fault;
  bool faultLatched;

  int32_t rpmInternal;
  int32_t rpmInternalTarget;
  int32_t rampStepInternal;
  int32_t rpmCmd;
  int32_t rpm1;

  RunProfile activeProfile;

  uint32_t stateEntryMs;
  uint32_t seatDeadlineMs;
  uint32_t holdDeadlineMs;

  bool commandOutputEnabled;
  bool lockActuatorCommanded;
  bool lockConfirmed;
  bool lockAwaitingSensor;
  uint32_t lockCommandStartMs;
  DoorState doorState;
  bool doorOpenSensorActive;
  bool doorClosedSensorActive;
  DoorMotorCommand doorMotorCommand;
  bool doorMoveActive;
  uint32_t doorMoveDeadlineMs;
  uint16_t ntcAdc;
  bool fanEnabled;
  bool overTempCritical;

  bool motorEnableOutput;
};

inline void clearPendingCommand(PendingCommand &pending) {
  memset(&pending, 0, sizeof(PendingCommand));
}

inline bool isMotorEnabledState(SystemState state) {
  return state == STATE_LIFT_RAMP || state == STATE_MAIN_RAMP || state == STATE_SPIN_HOLD;
}

#endif
