# Full Wiring Layout (Arduino Due + ODrive + Lock)

## Scope
This document defines what connects to what for the current firmware in:
- `firmware/src/config.h`
- `firmware/src/main.ino`

Current configured control pins:
- `PIN_MOTOR_ENABLE = 22`
- `PIN_LOCK_ACTUATOR = 23`
- `PIN_LOCK_SENSOR = 24`

Current comms:
- Host command link: Due `Serial` over USB at `115200`
- Motor command link: Due `Serial1` to ODrive UART at `115200`

## Safety-Critical Notes
- Arduino Due I/O is **3.3V only**.
- Do not drive actuators directly from Due pins.
- Keep torque-disable capability independent of UART/software.
- Use common signal ground across Due, ODrive logic, and interface boards.
- Use shielded wiring for motor phases and noisy actuator runs.

## System Blocks
1. Host PC (Python CLI/library)
2. Arduino Due (control logic)
3. ODrive (motor power stage)
4. BLDC motor (centrifuge spindle)
5. Lock actuator + driver stage
6. Lock sensor
7. Safety chain (E-stop/interlock path)

## Power Domains
| Domain | Source | Feeds | Notes |
|---|---|---|---|
| Logic 5V/USB | PC USB | Arduino Due | Development/programming power |
| Motor DC bus | Motor PSU | ODrive DC input | Sized for motor current and regen |
| Lock actuator power | Dedicated actuator PSU | Lock actuator driver output | Keep separate from Due GPIO power |

## Signal Wiring (Due-Centric)
| Due Pin | Firmware Name | Direction | Connect To | Electrical Notes |
|---|---|---|---|---|
| D22 | `PIN_MOTOR_ENABLE` | Output | External motor-enable gate input | Gate must fail-safe to disable torque |
| D23 | `PIN_LOCK_ACTUATOR` | Output | Lock driver input | Use MOSFET/relay driver, not direct coil drive |
| D24 | `PIN_LOCK_SENSOR` | Input | Lock sensor output | Firmware reads `HIGH` as `LOCK=1` |
| D18 (TX1) | `Serial1 TX` | Output | ODrive UART RX | 3.3V UART |
| D19 (RX1) | `Serial1 RX` | Input | ODrive UART TX | 3.3V UART |
| GND | Reference | - | ODrive GND + driver logic GND | Required for valid logic levels |

## USB Connections
| Connection | Purpose |
|---|---|
| Due Programming Port USB -> PC | Flash firmware + Python command/telemetry link |

## ODrive + Motor Wiring
| From | To | Notes |
|---|---|---|
| Motor PSU + | ODrive DC+ | Use proper fuse/protection |
| Motor PSU - | ODrive DC- | Low impedance return |
| ODrive M0 A/B/C | Motor U/V/W | Phase order can be corrected in ODrive config |
| Brake resistor | ODrive brake terminals | Required for regenerative energy handling |
| Due D18/D19/GND | ODrive RX/TX/GND | UART control path |

## Motor Enable Hardware Path (Recommended)
Use D22 to drive a **hardware gate** that can remove torque without relying on serial protocol. Typical implementations:
- ODrive enable line gate (if available in your ODrive variant), or
- Safety relay/contactor control in motor power path, or
- External drive inhibit input path.

The gate should be:
- Active only when D22 is asserted.
- Forced to disable by E-stop chain.
- Default disabled on MCU reset/power loss.

## Lock Actuator Wiring
### Required Interface
Due D23 must drive a transistor/driver stage:
- D23 -> driver input (through resistor or optocoupler input stage)
- Driver output -> actuator low side (or relay coil)
- Actuator supply positive -> actuator +
- Driver ground -> common logic/power reference as required by driver design
- Flyback diode across coil if inductive actuator

### Timeout Behavior
When LOCK is commanded:
- Firmware drives actuator output.
- Sensor must confirm lock before timeout.
- If not confirmed in time, firmware enters latched fault path.

## Lock Sensor Wiring
Firmware currently interprets:
- `D24 == HIGH` -> locked (`LOCK=1`)
- `D24 == LOW` -> unlocked (`LOCK=0`)

Recommended sensor output types:
- 3.3V push-pull, or
- Open collector/open drain with pull-up to 3.3V.

Do not feed 5V or 24V directly into D24.

Optional filtering for EMI-prone harnesses:
- Series resistor: ~1k
- Shunt capacitor to GND: 1nF to 10nF

## Safety Chain Wiring (Minimum Practical)
| Safety Element | Hardware Effect |
|---|---|
| E-stop (normally-closed) | Forces motor enable gate OFF |
| Lid interlock (normally-closed) | Inhibits torque path independent of UART |
| Driver/watchdog fault line | Also drops motor enable gate |

Software hard stop should not be your only torque removal mechanism.

## Grounding and EMC Guidance
- Star-ground logic returns where possible.
- Keep motor phase wiring physically separated from sensor/UART harnesses.
- Twist UART pair with ground reference if run length is significant.
- Shield and chassis-bond motor cable per your enclosure practice.
- Add ferrites near noisy boundaries if EMI resets or serial corruption appear.

## Bring-Up Checklist
1. Verify no pin is exposed to >3.3V at Due headers.
2. Verify D22 low truly disables torque path.
3. Verify lock actuator driver toggles from D23 only.
4. Verify lock sensor logic: locked state reads `HIGH` on D24.
5. Verify UART link: Due D18->ODrive RX and D19<-ODrive TX.
6. Power up with rotor unloaded and low RPM limits.
7. Run command checks:
   - `PING`
   - `VERSION`
   - `STATUS`
   - `INIT`
   - `LOCK` then `STATUS`

## Command/State Sanity Checks
- `RUN` only accepted in `SAFE_IDLE`.
- `UNLOCK` only in `SAFE_IDLE` and below `SAFE_UNLOCK_RPM`.
- `HARDSTOP` latches fault path and drops enable state.
- `STATUS` reports: `STATE RPM_CMD RPM1 FAULT LOCK ENABLE`.

## Field Mapping for Diagnostics
| STATUS Field | Meaning |
|---|---|
| `STATE` | Current state-machine state ID |
| `RPM_CMD` | Current command RPM (`rpmInternal / RPM_SCALE`) |
| `RPM1` | Primary RPM feedback placeholder |
| `FAULT` | Latched/current fault code |
| `LOCK` | Lock sensor-confirmed status |
| `ENABLE` | Motor enable output state |

