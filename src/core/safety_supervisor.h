#pragma once
#include "system_context.h"
#include "../config/constants.h"

class SafetySupervisor {
public:
  static FaultLevel evaluate(SystemContext& ctx) {
    if (ctx.fault_latched) {
      ctx.fault_code = FaultCode::NONE;
      return FaultLevel::FAULT_LATCHED;
    }

    if (ctx.estop_active) {
      ctx.fault_code = FaultCode::ESTOP_ACTIVE;
      return FaultLevel::HARD_STOP;
    }

    if (ctx.odrive_fault_active) {
      ctx.fault_code = FaultCode::ODRIVE_FAULT_LINE;
      return FaultLevel::HARD_STOP;
    }

    if (ctx.vbus_v >= VBUS_CRIT_V) {
      ctx.fault_code = FaultCode::VBUS_CRITICAL;
      return FaultLevel::HARD_STOP;
    }

    // If rotor is moving and rpm2 is lost -> HARD_STOP
    const bool moving = (ctx.rpm_primary >= RPM_PRIMARY_MIN_MOVING);
    if (moving && !ctx.rpm_secondary_ok) {
      ctx.fault_code = FaultCode::RPM2_LOST_WHILE_MOVING;
      return FaultLevel::HARD_STOP;
    }

    if (ctx.vbus_v >= VBUS_WARN_V) {
      ctx.fault_code = FaultCode::NONE;
      return FaultLevel::WARN;
    }

    ctx.fault_code = FaultCode::NONE;
    return FaultLevel::OK;
  }
};
