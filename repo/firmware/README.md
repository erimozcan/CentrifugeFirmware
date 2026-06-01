# Firmware (Arduino Due)

## Overview
This firmware implements a modular, non-blocking centrifuge control stack with a deterministic 1 kHz scheduler and fixed-point RPM ramps.

Modules:
- `SystemContext` / `PendingCommand` in `context.h`
- `StateMachine`
- `MotorInterface` (ESC UART via `Serial1`)
- `HardwareGuard` (motor enable + lock actuator)
- `CommandInterface` (USB serial command parser)
- `Protocol` (responses + integer parsing)

ESC UART velocity command used by firmware:
- `V=<rpm>\n`
- integer RPM only (no float in tick)
- non-blocking TX; frames are skipped if UART TX buffer is full.

## Pin Configuration
Set these in `src/config.h` to match your wiring:
- `PIN_MOTOR_ENABLE`
- `PIN_LOCK_ACTUATOR`
- `PIN_LOCK_SENSOR`

For full hardware-level wiring (power, UART, lock driver, sensor, safety chain), see:
- `../WIRING.md`

For quick pin-by-pin mapping, see:
- `../PINOUT.md`

Default placeholders:
- `PIN_MOTOR_ENABLE 22`
- `PIN_LOCK_ACTUATOR 23`
- `PIN_LOCK_SENSOR 24`
- `PIN_FAN_DRV_IN1 5`
- `PIN_FAN_DRV_IN2 4`
- `PIN_DOOR_DRV_IN1 6`
- `PIN_DOOR_DRV_IN2 7`
- `PIN_DOOR_OPEN_HALL 30`
- `PIN_DOOR_CLOSED_HALL 31`
- `PIN_TEMP_NTC A0`

## Upload (Arduino Due)
1. Open `firmware/src/main.ino` in Arduino IDE.
2. Select board: `Arduino Due (Programming Port)` or `Arduino Due (Native USB Port)`.
3. Select the correct COM port.
4. Upload.

## Serial Command Examples
Open serial monitor / terminal at `115200` baud and send lines:

```text
1 PING
2 VERSION
3 STATUS
4 INIT
5 LOCK
6 RUN LIFT=120 FINAL=300 SEAT=500 HOLD=2000 RAMPUP=1500 RAMPDOWN=1500
7 ABORT
8 HARDSTOP
9 CLEAR_FAULT
10 UNLOCK
11 DOOR_OPEN
12 DOOR_CLOSE
```

Responses:

```text
<SEQ> OK ...
<SEQ> ERR CODE=<code>
```

`STATUS` fields:
- `STATE`
- `RPM_CMD`
- `RPM1`
- `FAULT`
- `LOCK`
- `ENABLE`
- `DOOR`
- `TEMP_ADC`
- `FAN`

All status values are integer.
