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
  STATE_FAULT_LATCHED,
  // Automated full-cycle run choreography (appended so existing numeric state values,
  // used by the UI/protocol, do not shift). The gantry lock indexes the rotor at a tube
  // position for pipetting -> LOCKED at rest, RELEASED only to spin. So: close the door,
  // RELEASE the lock, spin; after ramp-down settle at rest, RE-ENGAGE the lock, open door.
  STATE_RUN_CLOSING = 10,
  STATE_RUN_RELEASE = 11,   // door closed -> release the gantry lock so the rotor can spin
  STATE_RUN_SETTLE = 12,    // post-spin: let the rotor coast fully to rest (still released)
  STATE_RUN_ENGAGE = 13,    // rotor stopped -> re-engage the gantry lock
  STATE_RUN_OPENING = 14,
  // Index the gantry to a tube position: release the lock, ESC angle-moves to the tube,
  // re-engage the lock. Used to present the next tube to the pipette (door open).
  STATE_ROTATE_RELEASE = 15,
  STATE_ROTATE_MOVING = 16,
  STATE_ROTATE_ENGAGE = 17,
  // Post-spin (in the run cycle): index the gantry to the nearest detent so the lock can
  // actually mesh -- a spin stops the rotor at a random angle, not a lockable position.
  STATE_RUN_INDEX = 18,
  // Post-spin bucket-sweep pass (door still closed): after RUN_INDEX first parks at a
  // detent, the lock extends to 50% -- its sweeper extrusion into the buckets' path --
  // and the gantry does one slow full turn, knocking every bucket flat as it passes.
  // Then RUN_INDEX runs again and the normal full lock + door-open follow.
  STATE_RUN_SWEEP_EXTEND = 19, // parked at the detent: lock extending to the 50% hold
  STATE_RUN_SWEEP_TURN = 20    // slow full knockdown revolution with the hold in place
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
  FAULT_INTERLOCK_NOT_SAFE = 9,
  FAULT_ROTATE_TIMEOUT = 10,
  // An index "arrival" whose ESC-reported shaft angle is NOT on a learned detent --
  // locking there would drive the pin into a bucket. Latched instead of engaging.
  FAULT_DETENT_MISMATCH = 11
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

enum LedMode : uint8_t {
  LED_MODE_SOLID = 0,    // show ledR/ledG/ledB (all-zero = off)
  LED_MODE_RAINBOW = 1   // animated hue sweep
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
  bool hasDoorStop;
  bool hasDoorSpeed;      // set the door slow-run PWM duty (0-255)
  uint8_t doorSpeed;

  bool hasLed;
  uint8_t ledR;
  uint8_t ledG;
  uint8_t ledB;
  uint8_t ledMode;

  bool hasPowerOn;
  bool hasPowerOff;

  bool hasRotate;         // index the gantry to a tube position (1..TUBE_COUNT)
  uint8_t rotateTube;
  bool hasHome;           // capture the current shaft angle as the detent reference

  bool hasAudioPlay;      // manual/bench: play a specific clip by file number
  uint8_t audioTrack;
  bool hasAudioVol;       // set audio volume (0-30)
  uint8_t audioVol;
};

struct SensorData {
  int32_t rpm1;
  char escCalState[18];
  int32_t escCalStep;
  int32_t escCalTargetRpm;
  int32_t escCurrentCentiamps;
  int32_t escObserverRpm;
  uint8_t escObserverLock;
  int32_t escPhaseErrDeg10;
  uint32_t escLoopHz;
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
  char escCalState[18];
  int32_t escCalStep;
  int32_t escCalTargetRpm;
  int32_t escCurrentCentiamps;
  int32_t escObserverRpm;
  uint8_t escObserverLock;
  int32_t escPhaseErrDeg10;
  uint32_t escLoopHz;

  RunProfile activeProfile;

  uint32_t stateEntryMs;
  uint32_t seatDeadlineMs;
  uint32_t holdDeadlineMs;

  bool commandOutputEnabled;
  bool lockActuatorCommanded;
  bool lockConfirmed;
  bool lockAwaitingSensor;
  uint32_t lockCommandStartMs;
  bool lockManualOverride;   // debug: manual LOCK/UNLOCK overrides the auto policy (idle only)
  bool lockManualEngaged;
  DoorState doorState;
  bool doorOpenSensorActive;
  bool doorClosedSensorActive;
  uint8_t doorOpenHallRaw;     // raw pin level (1=HIGH); surfaced for polarity bring-up
  uint8_t doorClosedHallRaw;
  DoorMotorCommand doorMotorCommand;
  bool doorMoveActive;
  uint32_t doorMoveDeadlineMs;
  uint8_t doorPwmDuty;     // live slow-run PWM duty (0-255), adjustable from the UI

  uint8_t rotateTube;      // tube the in-progress rotate is heading to (1..TUBE_COUNT)
  uint8_t currentTube;     // 0 = unknown (e.g. after a spin), else the last indexed tube
  uint32_t rotateDeadlineMs;
  bool sweepDone;          // knockdown turn finished -> next RUN_INDEX locks + opens
  bool lockSweepHold;      // lock is holding the partial 50% sweep extend right now
  bool homed;              // detent reference captured (mirror of MotorInterface::homeSet())
  uint16_t ntcAdc;
  bool fanEnabled;
  bool overTempCritical;
  bool devicePowered;   // UI "Turn on": fan runs + LEDs rainbow while true
  bool needEscTuning;   // one-shot: push tuned ESC gains on entry to PRE_RUN_VERIFY

  bool motorEnableOutput;

  uint8_t ledR;
  uint8_t ledG;
  uint8_t ledB;
  uint8_t ledMode;

  // Audio: a mailbox for manual play/volume requests (set by the command handler,
  // consumed + cleared by the AudioController in loop()). Automatic event cues are
  // observed by the AudioController directly from the state/door/lock fields above.
  uint8_t audioManualReq;   // 0 = none, else a file number (>=1) to play now
  bool audioVolPending;     // a volume change is queued
  uint8_t audioVol;         // requested volume (0-30)
  bool audioReady;          // mirror of AudioController readiness (module initialized)
};

inline void clearPendingCommand(PendingCommand &pending) {
  memset(&pending, 0, sizeof(PendingCommand));
}

// States where the rotor may be turning -> the door-closed + over-temp interlocks apply.
// Covers every phase from lift through ramp-down (SEAT_HOLD and STOPPING spin too), and
// the post-run knockdown revolution (slow, but the rotor IS turning with the door shut).
inline bool isMotorEnabledState(SystemState state) {
  return state == STATE_LIFT_RAMP || state == STATE_SEAT_HOLD ||
         state == STATE_MAIN_RAMP || state == STATE_SPIN_HOLD ||
         state == STATE_STOPPING || state == STATE_RUN_SWEEP_TURN;
}

// Gantry lock policy (state-derived, authoritative): the lock indexes the rotor at a
// discrete tube position so tubes stay put for pipetting. It is ENGAGED (locked) whenever
// the rotor is at rest, and RELEASED (unlocked) ONLY across the spin window -- from the
// pre-spin release (door already closed) through ramp-down and the coast-to-rest settle.
inline bool gantryLockEngaged(SystemState state) {
  switch (state) {
    case STATE_RUN_RELEASE:
    case STATE_PRE_RUN_VERIFY:
    case STATE_LIFT_RAMP:
    case STATE_SEAT_HOLD:
    case STATE_MAIN_RAMP:
    case STATE_SPIN_HOLD:
    case STATE_STOPPING:
    case STATE_RUN_SETTLE:
    case STATE_ROTATE_RELEASE:   // releasing so the gantry can be indexed
    case STATE_ROTATE_MOVING:    // motor is moving the gantry to the next tube
    case STATE_RUN_INDEX:        // post-spin: crawling to the nearest detent before re-lock
    case STATE_RUN_SWEEP_EXTEND: // bucket sweep: never fully locked -- the 50% hold is
    case STATE_RUN_SWEEP_TURN:   // driven separately via lockSweepHold
      return false;   // released (spinning / about to spin / coasting / indexing)
    default:
      return true;    // locked (idle, powered on/off, closing, engaging, opening, fault)
  }
}

#endif
