#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <Arduino.h>

#include "context.h"

class MotorInterface;
class HardwareGuard;

class StateMachine {
 public:
  void begin(SystemContext &ctx, uint32_t nowMs);
  void tick(
      SystemContext &ctx,
      const SensorData &sensorSnapshot,
      PendingCommand &pending,
      MotorInterface &motor,
      HardwareGuard &guard,
      uint32_t nowMs);

  static bool isTransitionAllowed(SystemState from, SystemState to);

 private:
  void updateInterlockState(SystemContext &ctx, const SensorData &sensorSnapshot);
  void updateDoorMotorControl(SystemContext &ctx, uint32_t nowMs);
  void updateFanControl(SystemContext &ctx);
  bool isRunInterlockSafe(const SystemContext &ctx) const;
  void applyPendingCommand(SystemContext &ctx, const PendingCommand &pendingLocal, uint32_t nowMs);
  void applyStateProgression(SystemContext &ctx, MotorInterface &motor, uint32_t nowMs);
  void updateFixedPointRamp(SystemContext &ctx);
  void transitionTo(SystemContext &ctx, SystemState next, uint32_t nowMs);
  void enterHardStop(SystemContext &ctx, FaultCode reason, uint32_t nowMs);
  void setRampTarget(SystemContext &ctx, int32_t rpmTarget, uint32_t durationMs);
};

#endif
