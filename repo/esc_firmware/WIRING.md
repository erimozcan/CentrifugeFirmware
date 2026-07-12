# Spin subassembly wiring — GT2215/10 + B-G431B-ESC1 (MB1419C) + AS5047P

**Interface: ABI quadrature, NOT SPI.** On this board PB6/PB7/PB8 are the encoder
`A+/B+/Z+` pins (UM2516 Table 4), and the AS5047P emits 1000 PPR / 4000 CPR on its
ABI outputs by default with no register programming. This avoids the bit-banged
SPI that failed last time (MISO read `0xFFFF`). Hardware SPI is not cleanly broken
out on this board, which is why SPI was always the fragile path.

## Source facts (UM2516 Table 4, "Main board STM32G431CB pinout for motor control")

| STM32 pin | UM2516 function | use here |
|---|---|---|
| PB6 | `A+/H1` | encoder **A** |
| PB7 | `B+/H2` | encoder **B** |
| PB8 | `Z+/H3` | encoder **Z** (index) |
| PA8/PA9/PA10 | `TIM1_CH1/2/3` | motor phase high sides |
| PA15 | PWM input (5 V tolerant) | unused |

Connectors: **J7** = 3-phase motor pads (U/V/W); **J5/J6** = battery in (3S–6S);
**J8** = Hall/encoder pads (H1/H2/H3 + 5 V + GND); **J3** = USART2 + PWM(PA15) + GND.

## THE WIRING

### Motor (J7)
| Motor lead | → pad |
|---|---|
| phase 1 | U |
| phase 2 | V |
| phase 3 | W |

Direction is arbitrary — if it spins the wrong way, swap any two leads (or just let
FOC's `sensor_direction` handle it; reported on arm in stage 4).

### Power (J5 / J6)
Bench PSU **12 V** → `+` / `−`. **Current-limit the PSU to ~1–2 A for first spins.**
This is the real safety backstop: the motor is ~0.1 Ω, so at standstill
applied-voltage / R = current, and even ~1 V can pull ~10 A without the limit.

### Encoder AS5047P-TS_EK_AB → ESC (ABI)
| EK P1 pin | signal | → ESC J8 pad | STM32 |
|---|---|---|---|
| P1-1 | VDD (5 V in) | **+5 V** | — |
| P1-8 | GND | **GND** | — |
| P1-10 | **A** | **H1** | PB6 |
| P1-9 | **B** | **H2** | PB7 |
| P1-11 | **I** (index) | **H3** | PB8 |
| P1-4/5/6/7 | SPI (CSn/CLK/MOSI/MISO) | — leave unconnected | — |

**Encoder power = J8's 5 V (correct here).** The EK adapter is designed for 5 V in
(default R1 populated) and regulates it down on-board, so its A/B/I outputs swing
**0–3.3 V** — STM32-safe. Two checks before connecting:
1. Meter J8's supply pad ≈ 5 V (not battery voltage).
2. Confirm the EK board is still in default **5 V** config (R1 populated, R2 empty).
   If it was resoldered to 3.3 V operation, feed it 3.3 V instead.

### Master-controller link (J3, as built)
USART2 on **J3** (3-wire: TX/RX/GND, 3.3 V) goes to the master controller's
`Serial1` (Nano ESP32 D2→ESC RX, ESC TX→D3, common GND).

> **Note (as built):** the ST-LINK daughterboard was **removed from the device**.
> Its MCU actively drives the shared USART2 pins (PA2/PA3) even when idle, which
> blocks any external controller on J3 — a documented board limitation (ST's
> alternative fix is removing resistors R23/R24). Flashing is now done with an
> external ST-LINK on the main board's SWD pads (UM2516 Table 6). During the
> original bring-up below, the daughterboard USB provided both flashing and
> serial.

## Confirm on the board before soldering
- **J8 pad order** — which physical pad is H1/H2/H3/5V/GND (Table 4 gives signals,
  not the left-to-right order).
- Encoder magnet: diametric, centered on the shaft axis, at the datasheet air gap.

## Firmware cross-check
Pins match [src/main.cpp](src/main.cpp): `Encoder encoder = Encoder(PB6, PB7, 1000, PB8)`
= (A=PB6, B=PB7, PPR=1000, index=PB8). Current sense `LowsideCurrentSense(0.003, -64/7, …)`
is forum-confirmed for MB1419C. All voltage caps are ≤ VBUS/2 (6 V) for clean
low-side current sensing.

## Bring-up order (matches the staged firmware)
1. **Flash over USB, bus power OFF.** Expect `READY. Encoder live, motor OFF.`
2. **`1` → stage ENCODER (still no bus power).** Turn shaft by hand; `rev` must
   track turns (1 full turn = 1.000). Garbage/no change ⇒ fix encoder wiring first.
3. **Power the bus** (PSU current-limited low).
4. **`2` → `g` (arm) → `v 1`.** Gentle open-loop spin; confirm phases + that
   encoder `rev` follows the commanded direction. `x` to stop.
5. **`3` → `g` → `v 1`.** Check `Ia/Ib/Ic` look balanced and sane.
6. **`4` → `g`.** FOC aligns (one twitch), prints `zero_electric_angle`. Then
   `a 1` (slow ramp), `v 5` to ease up smoothly, `s` to ramp down, `x` to stop.
7. **Tune** `PID_velocity` / `LPF_velocity.Tf`, then raise `BRINGUP_CURRENT`.
