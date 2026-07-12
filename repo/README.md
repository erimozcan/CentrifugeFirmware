# Project code

- `firmware/` — master-controller firmware (Arduino Nano ESP32): state machine,
  safety interlocks, door/lock/fan/LED/audio, ESC bridge.
- `esc_firmware/` — spindle ESC firmware (B-G431B-ESC1 + SimpleFOC): all
  closed-loop motor control.
- `ui/` — browser operator console (Web Serial over USB; also embedded in the
  firmware for the device's WiFi mode).

See the [top-level README](../README.md) for the system overview and quick
start, and each subproject's README for details.
