#pragma once
#include <Arduino.h>
#include "../core/system_context.h"
#include "../core/hardware_guard.h"
#include "../drivers/odrive_uart.h"

class CLI {
public:
  void begin() {
    _buf_len = 0;
  }

  void poll(SystemContext& ctx, HardwareGuard& guard, ODriveUART& odrive) {
    while (SerialUSB.available() > 0) {
      char c = (char)SerialUSB.read();
      if (c == '\r') continue;
      if (c == '\n') {
        _buf[_buf_len] = 0;
        handle_line(ctx, guard, odrive, _buf);
        _buf_len = 0;
      } else if (_buf_len < sizeof(_buf) - 1) {
        _buf[_buf_len++] = c;
      }
    }
  }

private:
  void handle_line(SystemContext& ctx, HardwareGuard& guard, ODriveUART&, const char* line) {
    if (strcmp(line, "status") == 0) {
      print_status(ctx);
      return;
    }

    if (strcmp(line, "hardstop") == 0) {
      ctx.fault_latched = true;
      ctx.fault_level = FaultLevel::FAULT_LATCHED;
      guard.hard_stop(ctx);
      SerialUSB.println("OK hardstop");
      return;
    }

    if (strcmp(line, "clear_fault") == 0) {
      const bool rpm0 = (ctx.rpm_primary < 1.0f && ctx.rpm_secondary < 1.0f);
      if (!rpm0) {
        SerialUSB.println("DENY rpm_not_zero");
        return;
      }
      ctx.fault_latched = false;
      ctx.fault_level = FaultLevel::OK;
      ctx.fault_code = FaultCode::NONE;
      SerialUSB.println("OK cleared");
      return;
    }

    if (strcmp(line, "maint_on") == 0) {
      const bool rpm0 = (ctx.rpm_primary < 1.0f && ctx.rpm_secondary < 1.0f);
      if (!rpm0) {
        SerialUSB.println("DENY rpm_not_zero");
        return;
      }
      // Maintenance mode is not a state machine here; it's just a flag you can extend.
      SerialUSB.println("OK maint_on");
      return;
    }

    if (strcmp(line, "maint_off") == 0) {
      SerialUSB.println("OK maint_off");
      return;
    }

    SerialUSB.println("ERR unknown");
  }

  void print_status(const SystemContext& ctx) {
    SerialUSB.print("t_ms="); SerialUSB.print(ctx.time_ms);
    SerialUSB.print(" cycles="); SerialUSB.print((uint32_t)(ctx.safety_cycles & 0xFFFFFFFF));
    SerialUSB.print(" fault_latched="); SerialUSB.print(ctx.fault_latched ? 1 : 0);
    SerialUSB.print(" lvl="); SerialUSB.print((int)ctx.fault_level);
    SerialUSB.print(" code="); SerialUSB.print((int)ctx.fault_code);
    SerialUSB.print(" estop="); SerialUSB.print(ctx.estop_active ? 1 : 0);
    SerialUSB.print(" odrv_fault="); SerialUSB.print(ctx.odrive_fault_active ? 1 : 0);
    SerialUSB.print(" vbus="); SerialUSB.print(ctx.vbus_v, 2);
    SerialUSB.print(" rpm2="); SerialUSB.print(ctx.rpm_secondary, 1);
    SerialUSB.print(" rpm2_ok="); SerialUSB.print(ctx.rpm_secondary_ok ? 1 : 0);
    SerialUSB.println();
  }

  char _buf[96];
  size_t _buf_len = 0;
};
