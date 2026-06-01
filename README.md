GEN1 Swing-Bucket Centrifuge Firmware (Safety Kernel v0)

Target: Arduino Due (SAM3X8E)
Motor controller: ODrive S1 (UART)
This package is intentionally limited:
- No spin / ramp state machine is implemented.
- It only implements the deterministic safety loop, fault latch, MotorEnable authority,
  and ODrive + MCU watchdog refresh gating.

CRITICAL:
- Your lock is NOT fail-locked. This firmware will NOT unlock the lock at all.
  It only supports "lock engage" output (optional) and requires RPM == 0 sustained.

What you must configure before using:
- pins in config/pins.h
- whether MotorEnable has an external pull-down (recommended)
- ODrive watchdog timeout and behavior via ODrive configuration

Build:
- Arduino IDE: select "Arduino Due (Programming Port)"
- Open gen1_safety_kernel.ino and upload

Serial CLI (SerialUSB, 115200):
- status
- hardstop
- clear_fault  (allowed only when RPM == 0 sustained)
- maint_on     (requires RPM==0 sustained; does not unlock lock)
- maint_off
