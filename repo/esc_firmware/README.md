# Spindle ESC firmware (B-G431B-ESC1 + SimpleFOC)

Runs on the ST **B-G431B-ESC1** (STM32G431, rev MB1419C) and owns *all*
closed-loop control of the spindle motor (EMAX GT2215/10, 7 pole pairs):
FOC current/velocity loops, alignment, ramping, and a comms watchdog. The
master controller (`../firmware/`) talks to it over a 3.3 V UART and only
ever expresses intent; it never micromanages the motor.

Shaft feedback: **AS5047P encoder read as ABI quadrature** into TIM4
(PB6/PB7, index PB8). Wiring: [WIRING.md](WIRING.md). The original staged
bring-up procedure is preserved in [BRINGUP.md](BRINGUP.md).

## Protocol (UART 115200)

**High-level verbs** (what the master speaks; whole-word match, takes
priority):

| Verb | Effect |
|---|---|
| `SPIN <rpm>` | arm/align if needed, ramp to target; acks `OK SPIN` once armed |
| `INDEX <0-3>` | closed-loop angle-servo move to a tube detent; holds until `STOP` |
| `HOME` | capture the CURRENT shaft angle as tube 0's detent reference |
| `STOP` | ramp to 0, then disarm |
| `ESTOP` | cut torque immediately |
| `PING` -> `OK PONG` | heartbeat |
| `STATUS?` | one `ST` line on demand |
| `CAL START/STOP/STATUS/APPLY` | manual service calibration (see BRINGUP.md) |
| `PROFILE SPIN/CRAWL` | select the ESC-owned PID profile |
| `TUNE?` | print the active gains |

`INDEX` runs on the CRAWL profile internally (the assembly needs ~4.5 A to break
stiction; the soft spin gains stall a slow servo move) and restores SPIN on disarm.
It refuses while the rotor is turning (>100 RPM) and moves shortest-path in either
direction. Targets are `HOME` reference + n x 90 deg.

First verb received engages "link mode": telemetry streams
`ST rpm=.. tgt=.. state=.. cur=..` at ~10 Hz, and a **watchdog fires after
~750 ms of silence** -> autonomous fail-safe ramp-down. The master heartbeats
every 250 ms to keep it fed.

**Bench protocol** (single characters, still available underneath; used for
service and live tuning, including by the master itself, which pushes velocity
PID gains this way at every run start): `1/2/3/4` select stage
(encoder / open-loop / current-sense / closed-loop FOC), `g` arm, `v <rev/s>`,
`a <rev/s^2>`, `s` ramp-to-0, `x` disarm, `?` one `STAT` line; live tuning
`k` align-V, `l` volt limit, `c` current limit, `p/i/f` velocity PID/LPF,
`q/j` current PID.

## Critical build facts -- do not regress

- **`lib_archive = no`** in `platformio.ini`. Without it SimpleFOC's
  B-G431 hardware support silently fails to link and the motor never spins.
- **`-D SERIAL_TX_BUFFER_SIZE=256`**; otherwise blocking telemetry starves
  the FOC loop and caps top speed.
- **SimpleFOC pinned to 2.4.0.**
- Working motor config (in `src/main.cpp`): foc_current mode,
  current_limit 1.5 A, voltage_limit 5 V (keep <= VBUS/2 for clean low-side
  current sense), ALIGN_VOLTAGE 1.4 V (the assembled machine needs ~4.5 A to
  break static friction), velocity PID P=0.05 / I=0.1 / Tf=0.15,
  current PID P=0.3 / I=20, ramp 8 rev/s^2.

## High-speed sensorless validation

Default builds clamp high-level `SPIN` commands to 4000 RPM and stay in
magnetic-encoder FOC. `-D HIGH_SPEED_SENSORLESS=1` or the
`b_g431_esc1_hs` PlatformIO environment enables observer shadow telemetry and
the sensorless handoff path. The accepted ceiling is **supply-aware**:
`min(10000, KV * (VBUS/2 - margin))` = **~5720 RPM on the 12 V bench**, the full
**10000 RPM only on the 24 V bus** (`-D SUPPLY_VOLTAGE=24.0f`, and the flag must
match the physical supply). This is back-EMF physics, not tuning -- a 12 V bus
cannot drive this 1100 KV motor meaningfully past ~6k. Treat high-speed mode as
bench-validation only until `state=spin-shadow|spin-sensorless`, `obs_rpm`,
`obs_lock`, and `phase_err` have been reviewed at staged speeds.

## Source vs. the flash on the shipped device

As of 2026-07-17 the master's default build has `ROTATE_VIA_INDEX` **on**: tube
indexing is done by this ESC firmware's closed-loop `INDEX` verb (smooth,
bidirectional, ~2 deg landing, active hold while the lock seats). This REQUIRES
an ESC flashed from this source. An ESC still running the old flash (no
`INDEX`/`HOME`) needs `ROTATE_VIA_INDEX` commented back out in
`../firmware/src/config.h`, which restores the legacy Nano-side crawl.

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

- At the 12 V bench supply the motor tops out around 5-6k RPM
  (BEMF vs VBUS/2); 4000 RPM is the comfortable validated ceiling.
- The >4000 RPM regime is gated behind `HIGH_SPEED_SENSORLESS=1` /
  `b_g431_esc1_hs` and uses the local observer handoff path. It is **untuned
  and unvalidated**; do not use it without containment, low current limits,
  staged speed steps, and the intended supply voltage.

`dashboard/` is a standalone ESC-only development dashboard (FastAPI +
pyserial against the ESC's own USB serial) from the spindle bring-up phase; it
is not part of device operation.
