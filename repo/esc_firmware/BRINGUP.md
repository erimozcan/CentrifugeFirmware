# Motor sub-assembly bring-up (B-G431B-ESC1 + GT2215/10 + AS5047P)

> **Historical service document.** This is the staged first-energize procedure
> used during development, kept for reference if the spindle sub-assembly ever
> needs to be re-proven on a bench. The shipped device is past all of this —
> see the [README](README.md) for current facts (e.g. ALIGN_VOLTAGE was later
> raised to 1.4 V for the assembled machine's static friction).

Status as of this checkpoint:
- **Encoder (AS5047P ABI -> TIM4 quadrature): WORKING.** Validated by hand-spin
  (`cnt`/`rev` track rotation). Programmed directly in `TIM4Encoder` (the
  SimpleFOCDrivers `STM32HWEncoder` silently failed to init on this core).
- **Driver + current sense + motor init: WORKING** (`INIT driver=1 cs=1 motor=1`).
- **Motor NOT yet energized.** Stages 2-4 below are the first energize; do them
  with the bench staged and a hand on the PSU.

## Hard-won fixes already baked in (do not regress)

1. **`lib_archive = no` in platformio.ini** — THE critical one. SimpleFOC's
   STM32/B-G431 hardware support overrides *weak* generic symbols
   (`_configure6PWM`, `_configureADCLowSide`). Built as a static `.a` archive,
   those strong overrides don't link, so the driver + low-side CS silently fall
   back to generic stubs ("driver not init" / "Low-side cs not supported") and
   the motor can never spin. `lib_archive=no` makes the strong symbols win.
2. **`loopFOC()` runs in EVERY armed stage** (not just closed-loop). In SimpleFOC
   2.4.0 `move()`/`velocityOpenloop()` only set the target + advance the open-loop
   angle; `loopFOC()` is the only thing that drives the phases. Without this,
   open-loop stages 2/3 are silently dead.
3. **`motor.enable()` BEFORE `initFOC()`** in `arm()`. `initFOC()` aligns by
   driving the phases; a disabled driver forces duty=0 -> alignment at 0 V ->
   garbage/failed `zero_electric_angle`.
4. **`ALIGN_VOLTAGE = 0.15 V`** (was 1.0). On the ~0.1 ohm motor, 1.0 V align
   commands ~7-10 A; 0.15 V ~= 1 A. There is NO current regulation during align,
   so the PSU current limit is the only backstop.
5. **Voltage torque mode + `currentSense.skip_align = true`** for bring-up:
   robust, smooth, no current-PID windup, and no powered CS-align that could flip
   the known-good -64/7 mapping. (Switch to `foc_current`, P-first, later.)

## Bench sequence (run when back at the bench)

PSU: **12 V, current limit ~1.5-2 A** the whole session (primary overcurrent
backstop for the 0.1 ohm winding). Motor clamped, no rotor/load, hand on PSU.
Commands are sent over the 115200 serial link (Claude drives them via the
two-way controller, or type them yourself).

| # | Send | Expect | Abort if |
|---|---|---|---|
| 1 | `1`, hand-turn exactly 1 rev | `rev` changes by **1.000** | `~1.024` -> AS5047P in binary mode; set `ENCODER_PPR=1024`, reflash |
| 2 | bus ON | PSU rests near 0 A | jumps to Ilim at rest -> short, cut power |
| 3 | `2` `g` `v 1` | smooth open-loop spin, fraction of an amp, `rev` follows | no motion / violent cog / PSU pegged -> `x`, recheck |
| 4 | `v 0.5` | mech speed = electrical/7 (confirms POLE_PAIRS=7) | ratio not ~1/7 -> `x`, fix pole pairs |
| 5 | `3` `g` `v 0` then `v 1` | rest: Ia/Ib/Ic ~0; spin: 3 balanced sinusoids; `Idc` ~ PSU amps | nonzero at standstill / one phase ~0 / Idc off ~2x -> `x` |
| 6 | `4` `g` | one twitch; prints finite `zero_electric_angle` + `sensor_direction` (±1, never 0); no `ERR initFOC FAILED` | fail/0/UNKNOWN, or PSU pinned through whole align -> `x`; if "no twitch" raise ALIGN_VOLTAGE to 0.20-0.30 (and PSU Ilim), do NOT go back to 1.0 |
| 7 | observe (holds v=0) | shaft holds still, small bounded `Idc`, no buzz | Idc pegs at limit while vel~0 (locked/inverted) -> `x` |
| 8 | `a 1` then `v 5` | `tgt` rises at 1 rev/s^2, `vel` tracks with small lag, smooth | runaway / oscillation / tgt >> vel -> `x`; tune PID_velocity.P toward 0.1, LPF_velocity.Tf 0.02-0.05 |
| 9 | `s`, then `v 20`,`v 50`,...`v 67`+ | smooth bounded ramps toward >4000 RPM | any runaway/oscillation/winding warmth -> `s` then `x`, cut PSU |

## Notes / open items for higher speed
- **6 V (VBUS/2) cap** is required for clean low-side CS but limits top speed; a
  1100 KV motor's BEMF approaches the modulation limit well below 13k RPM.
  For higher RPM raise the **bus voltage**, do NOT relax the VBUS/2 cap.
  `MOTOR_VOLT_LIMIT` starts at 3 V; raise toward VBUS/2 as you go faster.
- `zero_electric_angle` is NOT reproducible across power cycles (incremental
  encoder, Z/PB8 unused). Per-boot `initFOC` is correct; never hardcode it.
  `foc_ready` latches after first arm -> RESET the board between stage-4 trials.
- Recovery after any abort: `x` then `0`; reset board before re-arming stage 4.
