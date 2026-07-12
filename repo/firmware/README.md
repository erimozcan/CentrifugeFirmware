# Master-controller firmware (Arduino Nano ESP32)

The firmware running on the device's master controller. It owns the safety
interlocks and state machine (1 kHz tick), drives the door motor, gantry-lock
actuator, fan, LED ring, and voice audio, and bridges operator commands to the
spindle ESC over UART.

Full wiring / pin map: [PINMAP_NANO_ESP32.md](PINMAP_NANO_ESP32.md).

## Modules

| Files | Role |
|---|---|
| `main.ino` | setup + main loop; services the tick, USB serial, WiFi server, LED/audio controllers |
| `context.h` | `SystemContext` (all shared state), `PendingCommand` mailbox, state/fault IDs, lock policy |
| `state_machine.*` | the run/rotate choreography, interlocks, door auto-stop, fault latching |
| `command_interface.*` | parses `<seq> CMD args` lines and gates them by state |
| `protocol.*` | response formatting + `STATUS` line |
| `motor_interface.*` | ESC bridge on `Serial1`: SPIN/STOP/ESTOP verbs, heartbeat, telemetry parse, gantry rotate/home crawl |
| `hardware_guard.*` | raw outputs: DRV8871 door PWM (slow-decay + kick), lock servo, fan, motor-enable |
| `led_controller.*` | WS2812 ring (solid color / rainbow) |
| `audio_controller.*` | DFPlayer PRO voice lines, edge-triggered off state changes |
| `net_server.*` + `web_assets.S` + `data/` | WiFi AP + embedded copy of the operator UI (secondary to USB operation) |

## Build & flash

```
pio run -d repo/firmware -e nano_esp32 -t upload
```

The Nano flashes over its USB port (DFU). Known quirks, all benign:

- An upload sometimes fails verification or wedges DFU ("Failed to retrieve
  language identifiers"): unplug/replug the USB cable and flash again
  (double-tap RESET to force DFU if needed).
- After flashing, confirm the new firmware actually took by checking `STATUS`
  for a field the new build adds/changes.

## Operator serial protocol (USB CDC, 115200)

Lines of the form `<seq> CMD [args]` → `<seq> OK ...` or `<seq> ERR CODE=...`.
The operator UI in `../ui/` speaks this protocol; it can also be typed by hand.

| Command | Purpose |
|---|---|
| `PING`, `VERSION`, `STATUS` | liveness / version / full telemetry line |
| `POWER_ON` / `POWER_OFF` | device master power (fan + lighting) |
| `INIT` | arm from BOOT → SAFE_IDLE |
| `RUN LIFT=.. FINAL=.. SEAT=.. HOLD=.. RAMPUP=.. RAMPDOWN=..` | full automated cycle: close door → release lock → spin profile → settle → index to detent → re-lock → open door |
| `ABORT` | controlled ramp-down (allowed while at speed) |
| `HARDSTOP` | immediate stop, latches fault |
| `CLEAR_FAULT` | clear a non-latched fault |
| `DOOR_OPEN` / `DOOR_CLOSE` / `DOOR_STOP` | manual door moves (hall auto-stop) |
| `DOOR_SPEED <0-255>` | door run PWM duty |
| `ROTATE <1-4>` | index the gantry to a tube position |
| `HOME` | re-learn the detent reference |
| `LOCK` / `UNLOCK` | manual lock override (BOOT/SAFE_IDLE only; lock is normally automatic — engaged at rest, released while spinning) |
| `LED R=.. G=.. B=..`, `LED MODE=1` | lighting (solid / rainbow) |
| `AUDIO <n>`, `AUDIO_VOL <0-30>` | play a voice clip / set volume |
| `ESCRAW ...` | raw ESC passthrough (debug builds only, `-DESC_DEBUG_RELAY`) |

A latched fault (`FAULT_LATCHED`) has no software recovery — power-cycle the
device.

## Voice audio (DFPlayer PRO)

The MP3s in `../../CentrifugeVoiceLines/` live on the DFPlayer PRO's internal
128 MB storage. To reload them: connect the DFPlayer's own USB-C to a PC (it
mounts as a flash drive), wipe it, then copy the clips **one at a time in cue
order** — the module addresses files by copy order, not filename. The cue → file
number map is `AUDIO_FILE_*` in `src/config.h`.

## Service / diagnostic firmwares

Each isolates one subsystem for bench testing; build/flash the matching env
(`pio run -d repo/firmware -e <env> -t upload`). They are `#ifdef`-guarded and
contribute nothing to the production binary.

| Env | Source | Tests |
|---|---|---|
| `nano_lock_test` | `src/lock_sweep_test.cpp` | lock servo sweep on D10 |
| `nano_pin_test` | `src/lock_digital_test.cpp` | plain digital toggle on the lock pin |
| `door_test` | `src/door_motor_test.cpp` | door motor manual PWM |
| `door_digi_test` | `src/door_digital_test.cpp` | door motor full-voltage drive |
| `door_move_test` | `src/door_move_test.cpp` | door motor + hall sensors closed-loop |
| `due_serial_test` | `src/serial_test_mode.ino` | Due-prototype-era gated harness (reference) |

Debug build flags on the production env (both **off** for normal use — see
comments in `platformio.ini`): `-DESC_DEBUG_RELAY` (mirror ESC serial to USB +
enable `ESCRAW`), `-DBYPASS_DOOR_INTERLOCK` (force "door closed" to bench-spin
without door magnets).

## ESP32 gotcha worth knowing before editing pin code

On the Nano ESP32 (arduino-esp32 2.x with pin remapping), **core Arduino APIs
(`digitalWrite`, `analogWrite`, `ledcAttachPin`, …) take the logical `D`/`A`
pin and remap internally — never pre-convert with `digitalPinToGPIONumber()`
for those** (double-remap sends the signal to the wrong pad). Libraries that
bypass the Arduino pin layer (Adafruit_NeoPixel, ESP32Servo) need the raw GPIO,
so *those* get `digitalPinToGPIONumber(pin)`. Both patterns are used correctly
in `hardware_guard.cpp` / `led_controller.cpp` — match them.
