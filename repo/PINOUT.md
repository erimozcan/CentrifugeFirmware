# Pinout (Arduino Due)

This pinout matches the current firmware configuration in:
- `firmware/src/config.h`
- `firmware/src/main.ino`
- `firmware/src/hardware_guard.cpp`

## Core Safety and Lock
| Due Pin | Firmware Symbol | Direction | Connected Element | Notes |
|---|---|---|---|---|
| D22 | `PIN_MOTOR_ENABLE` | Output | Motor enable safety gate | Must disable torque when LOW |
| D23 | `PIN_LOCK_ACTUATOR` | Output | PQ12 lock control input | Drive via proper interface stage |
| D24 | `PIN_LOCK_SENSOR` | Input | Lock confirmation sensor | Firmware treats `HIGH` as locked |

## ODrive S1 UART (Serial1)
| Due Pin | Signal | Direction | ODrive S1 | Notes |
|---|---|---|---|---|
| D18 | `TX1` | Output | UART RX | ASCII protocol, 115200 baud |
| D19 | `RX1` | Input | UART TX | Shared logic GND required |
| GND | GND | - | GND | Common reference |

## DRV8871 #1 (Cooling Fan)
| Due Pin | Firmware Symbol | Direction | DRV8871 Fan Input |
|---|---|---|---|
| D5 | `PIN_FAN_DRV_IN1` | Output | IN1 |
| D4 | `PIN_FAN_DRV_IN2` | Output | IN2 |

## DRV8871 #2 (N20 Door Motor)
| Due Pin | Firmware Symbol | Direction | DRV8871 Door Input |
|---|---|---|---|
| D6 | `PIN_DOOR_DRV_IN1` | Output | IN1 |
| D7 | `PIN_DOOR_DRV_IN2` | Output | IN2 |

## Door Hall Sensors (AP02A / A3144)
| Due Pin | Firmware Symbol | Direction | Sensor Meaning | Default Polarity |
|---|---|---|---|---|
| D30 | `PIN_DOOR_OPEN_HALL` | Input | Door open detect | Active LOW (`DOOR_OPEN_ACTIVE_LOW=1`) |
| D31 | `PIN_DOOR_CLOSED_HALL` | Input | Door closed detect | Active LOW (`DOOR_CLOSED_ACTIVE_LOW=1`) |

Both inputs are configured with pullups in firmware (`INPUT_PULLUP`).

## Temperature (100k NTC Divider)
| Due Pin | Firmware Symbol | Direction | Connected Element | Notes |
|---|---|---|---|---|
| A0 | `PIN_TEMP_NTC` | Input | NTC divider midpoint | 12-bit ADC (`analogReadResolution(12)`) |

Recommended divider:
- `3.3V -> 100k fixed resistor -> A0 -> 100k NTC -> GND`

## USB / Host Link
| Interface | Purpose |
|---|---|
| Due Programming Port USB | Firmware upload + Python CLI serial commands (`Serial`, 115200) |

## Command/Feature Mapping by Pin Group
- ODrive spin commands use `Serial1` on `D18/D19`.
- Door move commands (`DOOR_OPEN`, `DOOR_CLOSE`) drive `D6/D7`.
- Fan auto-control uses `D5/D4`.
- Safety interlock checks use `D30/D31`, `D24`, and `A0`.

## If Sensor Polarity Is Opposite
Edit in `firmware/src/config.h`:
- `DOOR_OPEN_ACTIVE_LOW`
- `DOOR_CLOSED_ACTIVE_LOW`

Set each to `0` if your hall output is active HIGH.
