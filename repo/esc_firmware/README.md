# Spindle ESC firmware (B-G431B-ESC1 + SimpleFOC)

Runs on the ST **B-G431B-ESC1** (STM32G431, rev MB1419C) and owns *all*
closed-loop control of the spindle motor (EMAX GT2215/10, 7 pole pairs):
FOC current/velocity loops, alignment, ramping, and a comms watchdog. The
master controller (`../firmware/`) talks to it over a 3.3 V UART and only
ever expresses intent — it never micromanages the motor.

Shaft feedback: **AS5047P encoder read as ABI quadrature** into TIM4
(PB6/PB7, index PB8). Wiring: [WIRING.md](WIRING.md). The original staged
bring-up procedure is preserved in [BRINGUP.md](BRINGUP.md).

## Protocol (UART 115200)

**High-level verbs** (what the master speaks; whole-word match, takes
priority):

| Verb | Effect |
|---|---|
| `SPIN <rpm>` | arm/align if needed, ramp to target; acks `OK SPIN` once armed |
| `STOP` | ramp to 0, then disarm |
| `ESTOP` | cut torque immediately |
| `PING` → `OK PONG` | heartbeat |
| `STATUS?` | one `ST` line on demand |

First verb received engages "link mode": telemetry streams
`ST rpm=.. tgt=.. state=.. cur=..` at ~10 Hz, and a **watchdog fires after
~750 ms of silence** → autonomous fail-safe ramp-down. The master heartbeats
every 250 ms to keep it fed.

**Bench protocol** (single characters, still available underneath — used for
service and live tuning, including by the master itself, which pushes velocity
PID gains this way at every run start): `1/2/3/4` select stage
(encoder / open-loop / current-sense / closed-loop FOC), `g` arm, `v <rev/s>`,
`a <rev/s²>`, `s` ramp-to-0, `x` disarm, `?` one `STAT` line; live tuning
`k` align-V, `l` volt limit, `c` current limit, `p/i/f` velocity PID/LPF,
`q/j` current PID.

## Critical build facts — do not regress

- **`lib_archive = no`** in `platformio.ini`. Without it SimpleFOC's
  B-G431 hardware support silently fails to link and the motor never spins.
- **`-D SERIAL_TX_BUFFER_SIZE=256`** — otherwise blocking telemetry starves
  the FOC loop and caps top speed.
- **SimpleFOC pinned to 2.4.0.**
- Working motor config (in `src/main.cpp`): foc_current mode,
  current_limit 1.5 A, voltage_limit 5 V (keep ≤ VBUS/2 for clean low-side
  current sense), ALIGN_VOLTAGE 1.4 V (the assembled machine needs ~4.5 A to
  break static friction), velocity PID P=0.05 / I=0.1 / Tf=0.15,
  current PID P=0.3 / I=20, ramp 8 rev/s².

## Source vs. the flash on the shipped device

The source here is a **superset** of what is flashed on the device: it adds an
`INDEX <0-3>` closed-loop tube-positioning verb that was staged but never
flashed (the master instead closes the tube-indexing loop itself from encoder
telemetry — its default `ROTATE_VIA_INDEX`-off build matches the shipped ESC).
Reflashing this source is safe and enables the more precise ESC-side indexing
(flip `ROTATE_VIA_INDEX` on in `../firmware/src/config.h` afterwards).

## Reflashing

The ST-LINK **daughterboard was removed from the device on purpose**: its MCU
actively drives the shared USART2 pins (PA2/PA3) even when idle, which blocks
the master's UART link (a documented board limitation; the ST-recommended
alternative is removing resistors R23/R24). To reflash, wire an external
ST-LINK (the detached daughterboard works) to the SWD pads on the ESC main
board (UM2516 Table 6), then:

```
pio run -d repo/esc_firmware -t upload
```

## Known limits

- At the 12 V bench supply the motor tops out ~5–6k RPM (BEMF vs VBUS/2);
  4000 RPM is the comfortable validated ceiling.
- The open-loop >4000 RPM regime in the source (`CROSSOVER_RPM`..10k) is
  **untuned and unvalidated** — do not use it without containment and the
  intended 24 V supply.

`dashboard/` is a standalone ESC-only development dashboard (FastAPI +
pyserial against the ESC's own USB serial) from the spindle bring-up phase; it
is not part of device operation.
