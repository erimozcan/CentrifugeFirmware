#pragma once
#include <Arduino.h>
#include "../core/system_context.h"

class ODriveUART {
public:
  void begin(HardwareSerial& port, uint32_t baud) {
    _port = &port;
    _port->begin(baud);
    _last_rx_ms = millis();
    _heartbeat_ok = false;
  }

  void poll() {
    // Non-blocking read: just drain available bytes and update heartbeat timestamp.
    while (_port && _port->available() > 0) {
      (void)_port->read();
      _last_rx_ms = millis();
    }

    // Heartbeat: if we have seen any bytes recently, we call it "alive".
    // Replace with a real ODrive protocol later.
    const uint32_t now = millis();
    _heartbeat_ok = (now - _last_rx_ms) < 200; // placeholder
  }

  void refresh_watchdog() {
    // Placeholder: you must implement the actual ODrive watchdog refresh command
    // based on how you configured ODrive (ASCII protocol, CAN, etc).
    // For now, we just send a lightweight newline to create bus activity.
    if (_port) {
      _port->write('\n');
    }
  }

  void publish_to_context(SystemContext& ctx) {
    ctx.odrive_heartbeat_ok = _heartbeat_ok;
    // rpm_primary and vbus could be populated here once protocol is implemented.
  }

private:
  HardwareSerial* _port = nullptr;
  uint32_t _last_rx_ms = 0;
  bool _heartbeat_ok = false;
};
