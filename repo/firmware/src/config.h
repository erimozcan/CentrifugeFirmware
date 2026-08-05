#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

#define SERIAL_BAUD 115200
#define ESC_UART_BAUD 115200
#define ESC_UART_CMD_MAX_LEN 24U

// Due->ESC link pacing. The state machine calls sendVelocitySetpoint() every
// tick (~1 kHz); we must NOT emit a verb that fast or we saturate the UART.
// ESC_TX_MIN_MS caps setpoint updates while spinning (~25 Hz is plenty smooth);
// ESC_TX_HEARTBEAT_MS keeps the ESC's comms watchdog (~750 ms) fed when the
// setpoint is steady or the system is idle. Both must stay < the ESC watchdog.
#define ESC_TX_MIN_MS 40U
#define ESC_TX_HEARTBEAT_MS 250U

#define TICK_INTERVAL_US 1000U
#define MAX_TICK_CATCHUP 4U

#define RPM_SCALE 1000
// Closed-loop operating ceiling the Nano will command. Default builds stay at the
// validated 4000 RPM encoder-FOC ceiling. Only high-speed validation builds may
// command the ESC's sensorless observer handoff path.
#ifndef HIGH_SPEED_SENSORLESS
#define HIGH_SPEED_SENSORLESS 0
#endif
#if HIGH_SPEED_SENSORLESS
// 12000 = the 24 V bus's practical physical max (supply-bounded 12320 ideal minus
// IR/load losses; see MAX_SPIN_RPM in the ESC). Raised from 10000 (2026-07-28)
// after 10k was validated on the device; 10k-12k needs incremental validation.
#define MAX_RPM_BAREBONES 12000
#else
#define MAX_RPM_BAREBONES 4000
#endif
#define SAFE_UNLOCK_RPM 50

// Lock crush guard (2026-08-04): the FULL lock throw is only ever commanded when the
// ESC-reported shaft angle sits within DETENT_VERIFY_TOL_REV of a learned detent, on
// telemetry no older than DETENT_POS_FRESH_MS. Half a tube slot is 0.125 rev, so ~20 deg
// (the lock taper's capture range) can never pass at a bucket. Motivation: a mid-cycle
// 5 V brownout reboot (lock-actuator stall; see the audio notes below) used to re-extend
// the lock at boot wherever the gantry stopped -- once, onto a bucket. The manual LOCK
// override (bench rig, no ESC telemetry) bypasses the guard.
#define DETENT_VERIFY_TOL_REV  0.055f
#define DETENT_POS_FRESH_MS    1500U
// Master boot: wait this long for the ESC to report an explicitly-set home reference
// (the ESC rides the 24 V bus, so its reference SURVIVES a master brownout reboot and
// must be adopted, not blindly re-captured at whatever mid-cycle angle the gantry was
// left). Fall back to capturing at the current angle on a fresh full power-up or an
// older ESC flash that doesn't report home telemetry.
#define HOME_ADOPT_WAIT_MS     3000U

// ===========================================================================
// Pin map. Two targets share this firmware:
//   * Arduino Nano ESP32 (ESP32-S3) -- the FINAL master controller.
//   * Arduino Due                    -- the prototype used for bring-up.
// Selected by the core's ARDUINO_ARCH_ESP32 macro. Full rationale + wiring in
// repo/firmware/PINMAP_NANO_ESP32.md. Dx/Ax named constants are used on the
// Nano so they resolve correctly regardless of the IDE pin-numbering mode.
// ===========================================================================
#if defined(ARDUINO_ARCH_ESP32)

// ---- Arduino Nano ESP32 (ESP32-S3) ----------------------------------------
// ESC link: Serial1 (UART1) remapped to these pins (3.3 V, crossover, GND).
#define PIN_ESC_UART_TX      D2    // GPIO5  -> ESC RX
#define PIN_ESC_UART_RX      D3    // GPIO6  <- ESC TX

// DFPlayer PRO (DFR0768): Serial2 (UART2) @ 115200 baud, AT-command protocol.
// Direct 3.3 V (no series resistor). The PRO has NO BUSY pin (status is via AT
// query) so A4 stays free. Driver: audio_controller.{h,cpp} (fire-and-forget
// AT writes; the DFRobot_DF1201S library is used only for one-time setup).
#define PIN_DFPLAYER_TX      D4    // GPIO7  -> DF RX
#define PIN_DFPLAYER_RX      D5    // GPIO8  <- DF TX
#define HAS_AUDIO            1

// Door motor via DRV8871 #1 (kick + PWM slow-run; see updateDoorMotor()).
#define PIN_DOOR_DRV_IN1     D6    // GPIO9
#define PIN_DOOR_DRV_IN2     D7    // GPIO10
// Door PWM (analogWrite) frequency. 20 kHz keeps it out of the audible band. analogWrite
// on this core takes the RAW GPIO (see door_motor_test.cpp / updateDoorMotor()).
#define DOOR_PWM_FREQ_HZ     20000

// Fan via DRV8871 #2.
#define PIN_FAN_DRV_IN1      D8    // GPIO17
#define PIN_FAN_DRV_IN2      D9    // GPIO18

// Lock actuator: Actuonix PQ12-63-6-R RC servo (1-2 ms pulse, own controller).
#define PIN_LOCK_ACTUATOR    D10   // GPIO21 (servo signal)
#define LOCK_IS_SERVO        1
#define LOCK_PULSE_MIN_US      1000
#define LOCK_PULSE_MAX_US      2000
// Per Actuonix PQ12-R datasheet (Rev D): 1.0 ms = fully EXTEND, 2.0 ms = fully RETRACT,
// position linear in pulse width between them. On this build the mechanics are:
// EXTENDED = locked, RETRACTED = unlocked. SWEEP is the partial extend that puts the
// pin's sweeper extrusion into the buckets' path for the post-run knockdown turn
// (see STATE_RUN_SWEEP_*). (A 95% locked throw was tried 2026-08-01: not far enough.)
// Pulse -> travel is linear: extend % = (2000 - pulse) / 10.
#define LOCK_PULSE_LOCKED_US   1000  // 1.0 ms = full extend = LOCKED
#define LOCK_PULSE_UNLOCKED_US 2000  // 2.0 ms = retract = UNLOCKED
#define LOCK_PULSE_SWEEP_US    1700  // 30% extend = bucket-sweep hold (50% then 40%
                                     // both reached too far; bench 2026-08-05).
                                     // Tune live with LOCK_PCT <0-100>, then set here.

// WS2812 addressable LED strip on D11 (GPIO38, via RMT). Adafruit_NeoPixel.
#define PIN_LED_STRIP        D11   // GPIO38 (data, via RMT)
#define LED_COUNT            30    // strip length (<=30 on this build)
#define HAS_LED_STRIP        1

// WiFi operator console: the ESP32 serves the UI as its own hotspot. Join this network
// from a laptop/iPad, then open http://192.168.4.1/ . (Enabled via WITH_WEB_ASSETS build
// flag, which also links the embedded UI; see net_server.*.)
#define HAS_WIFI             1
#define WIFI_AP_SSID         "Centrifuge"
#define WIFI_AP_PASS         "spin1234"     // >= 8 chars for WPA2; change to taste

// Door hall sensors (MUST be powered at 3.3 V -- S3 GPIO not 5 V tolerant).
#define PIN_DOOR_OPEN_HALL   A0    // GPIO1
#define PIN_DOOR_CLOSED_HALL A1    // GPIO2

// NTC temp -- dropped for now; ADC1-safe slot kept reserved for its return.
#define PIN_TEMP_NTC         A3    // GPIO4 (ADC1_CH3)
#define HAS_NTC              0     // no thermistor fitted -> report a "cool" value

// No dedicated ESC hardware-enable line and no lock-feedback sensor are wired
// on the Nano (ESC is armed over UART; PQ12-R has no position output). These
// keep the shared guard/state-machine code portable but are non-functional:
//   - PIN_MOTOR_ENABLE: spare indicator GPIO only.
//   - PIN_LOCK_SENSOR : OPEN-LOOP -- floats HIGH via pullup => lock always reads
//     "confirmed". Replace with a limit switch or command-trust logic in Phase E.
#define PIN_MOTOR_ENABLE     D12   // GPIO47 (spare/indicator)
#define PIN_LOCK_SENSOR      A5    // GPIO12 (spare input; see note)
#define PIN_DOOR_ENC_A       A6    // GPIO13 (spare; door encoder unused)
#define PIN_DOOR_ENC_B       A7    // GPIO14 (spare; door encoder unused)

#else

// ---- Arduino Due (prototype) ----------------------------------------------
#define PIN_MOTOR_ENABLE 22
#define PIN_LOCK_ACTUATOR 23
#define PIN_LOCK_SENSOR 24
#define PIN_FAN_DRV_IN1 5
#define PIN_FAN_DRV_IN2 4
#define PIN_DOOR_DRV_IN1 6
#define PIN_DOOR_DRV_IN2 7
#define PIN_DOOR_OPEN_HALL 30
#define PIN_DOOR_CLOSED_HALL 31
#define PIN_DOOR_ENC_A 8
#define PIN_DOOR_ENC_B 9
#define PIN_TEMP_NTC A0
#define HAS_NTC 1
#define HAS_LED_STRIP 0   // no WS2812 driver on the Due prototype
#define HAS_AUDIO 0       // no DFPlayer wired on the Due prototype
#define HAS_WIFI 0        // Due has no WiFi
// The Due drives the lock with a plain digital output (no servo), but the SHARED state
// machine still resolves a servo pulse before handing it to the guard, which ignores it
// on this target. Define the same endpoints so that arithmetic compiles and stays
// meaningful if a servo is ever fitted to the prototype.
#define LOCK_PULSE_LOCKED_US   1000
#define LOCK_PULSE_UNLOCKED_US 2000
#define LOCK_PULSE_SWEEP_US    1700

#endif

// Placeholder ADC value reported when no NTC is fitted: above FAN_OFF and well
// above CRITICAL_LOW, so the fan stays off and over-temp never false-trips.
#define NTC_ADC_NO_SENSOR 2500

// ===========================================================================
// Audio (DFPlayer PRO) -- spoken-prompt cues tied to machine events. The driver
// (audio_controller.*) is a pure OBSERVER of SystemContext: it watches for state/
// door/lock/power edges each loop and plays the matching clip. See PINMAP notes.
// ===========================================================================
#define AUDIO_UART_BAUD  115200
#define AUDIO_VOLUME     20      // startup volume, 0-30 (override live: AUDIO_VOL <n>)

// Module init timing. The DFPlayer PRO takes ~2-3 s to boot after power-up, and its
// 5 V rail may come up after the Nano (USB-powered). So: wait, retry a few times at
// boot, and if it still hasn't answered, keep retrying while the machine is IDLE
// (never during a spin -- df.begin() blocks up to ~1 s on a missing ACK). This makes
// audio self-heal the moment the module gets power, with no reflash.
#define AUDIO_INIT_BOOT_DELAY_MS 1500U  // wait for the module to boot before the 1st AT
#define AUDIO_INIT_BOOT_RETRIES  4U     // attempts during setup()
#define AUDIO_INIT_RETRY_GAP_MS  300U   // gap between the boot-time attempts
#define AUDIO_INIT_IDLE_RETRY_MS 3000U  // then re-probe this often while idle until ready

// The DFPlayer PRO does NOT power-down-save its play mode: every module power-up
// (including a brownout reboot if the shared 5 V rail droops under a lock-actuator
// stall) silently reverts it to mode 2 = "repeat all". In that mode ANY clip rolls
// straight on through every file on the flash, in copy order, forever -- the
// "reads out all the voice lines nonstop" failure. So single-shot mode (3) is
// re-asserted this often for the machine's whole life, not just once at init.
#define AUDIO_MODE_REASSERT_MS 5000U
// The module can drop an AT command that arrives while the previous one is still
// being processed (a PLAYNUM sent back-to-back with a PLAYMODE was silently eaten ->
// no sound at all). Every raw command is therefore given this much clear bus before
// a queued clip is released.
#define AUDIO_CMD_SPACING_MS 250U

// Minimum spacing between consecutive clips. Cues are queued and released one every
// this interval so a burst of events (e.g. end of run: locking -> door opening -> run
// complete, all within ~1 s) doesn't cut each clip off. Set a bit above the longest
// run-cycle clip (~1.5 s) so each finishes with a small gap.
#define AUDIO_CUE_GAP_MS 1700U

// The gantry-lock cue is edge-triggered on ctx.lockActuatorCommanded, which the rpm
// safety net can flip rapidly as the measured RPM jitters across SAFE_UNLOCK_RPM (e.g.
// while a rotor coasts down after an E-STOP). Only announce a lock/unlock once its state
// has been STABLE this long, so that jitter never spams cues.
#define AUDIO_LOCK_CUE_DEBOUNCE_MS 500U

// Cue -> file NUMBER. On the DFPlayer PRO a file's number is the order it was COPIED
// onto the module's flash (NOT its name). These match the CentrifugeVoiceLines/ clip
// set (filenames 01..10). If a clip plays for the wrong event, just swap the number
// here -- verify each one live with: AUDIO <n>. A value of 0 = no clip (cue silent).
#define AUDIO_FILE_AT_SPEED      1   // "at speed"
#define AUDIO_FILE_DOOR_CLOSING  2   // "door closing"
#define AUDIO_FILE_DOOR_OPENING  3   // "door opening"
#define AUDIO_FILE_LOCKING       4   // "locking gantry"
#define AUDIO_FILE_POWER_ON      5   // "power on"
#define AUDIO_FILE_RUN_COMPLETE  6   // "run complete"
// Per-tube "tube N selected" clips (announced when a ROTATE to that tube starts).
#define AUDIO_FILE_TUBE1         8   // "tube one selected"
#define AUDIO_FILE_TUBE2         10  // "tube two selected"
#define AUDIO_FILE_TUBE3         9   // "tube three selected"
#define AUDIO_FILE_TUBE4         7   // "tube four selected"
// Events with no recorded clip yet -> 0 keeps them silent (playFile ignores 0).
#define AUDIO_FILE_UNLOCKING     0
#define AUDIO_FILE_SPINNING_UP   0
#define AUDIO_FILE_SLOWING       0
#define AUDIO_FILE_FAULT         0

#define DOOR_OPEN_ACTIVE_LOW 1
#define DOOR_CLOSED_ACTIVE_LOW 1

#define LOCK_ENGAGE_TIMEOUT_MS 1500U
#define PRE_RUN_VERIFY_MS 200U

// Automated full-cycle run choreography dwell times (the door close/open steps use
// DOOR_MOVE_TIMEOUT_MS). The gantry lock is open-loop (no position sensor), so these give
// the actuator time to physically move before the next step. STROKE MATH (2026-08-04):
// the PQ12-63's full 20 mm stroke takes ~1.6 s no-load at its 15 mm/s max -- and the
// shared 5 V rail runs the 6 V unit ~17% slow -- so the old 800 ms dwells started the
// spin (and opened the door) with the pin only ~60% of the way through its travel.
#define RUN_RELEASE_MS  2500U   // let the lock FULLY RETRACT (full 20 mm) before spinning
#define RUN_SETTLE_MS   1500U   // dwell at 0 rpm so the rotor fully coasts to rest
#define RUN_ENGAGE_MS   2500U   // let the lock FULLY EXTEND (seat) before opening the door

// Post-run bucket-sweep pass: the buckets are too light to fold back down after a
// spin, so before the final lock (door still closed) the lock extends HALFWAY --
// putting its sweeper extrusion into the buckets' path -- and the gantry does one
// slow full turn, knocking every bucket flat as it passes. The turn stops a hair
// short of the full rev so the re-index approaches the detent cleanly (forward on
// the legacy crawl; either way for ESC INDEX), then the normal re-index + full
// lock + door-open follow.
#define SWEEP_EXTEND_MS  1500U   // let the lock reach the sweep hold (~8 mm, well under a
                                 // second at the 5 V rail's ~12.5 mm/s) before the turn
#define SWEEP_TURN_RPM    30     // knockdown crawl speed (crawl profile; kept under
                                 // SAFE_UNLOCK_RPM so the rpm net never drops the hold)
#define SWEEP_REV_TARGET  0.96f  // stop ~15 deg short of the full rev; the re-index
                                 // covers the remainder and lands the detent
// The sweeper is SUPPOSED to touch buckets at SWEEP_TURN_RPM, so the plain lock safety
// net (SAFE_UNLOCK_RPM = 50) is the wrong bar for it: measured RPM jitters around the
// 30 RPM crawl and a momentary reading over 50 would retract the sweeper mid-turn --
// silently skipping buckets with no fault to show for it. This backstop still retracts
// if the rotor ever really runs away, but sits well clear of the crawl.
#define SWEEP_ABORT_RPM   150

// Gantry tube indexing (ROTATE). Reuses RUN_RELEASE_MS/RUN_ENGAGE_MS for the lock dwells.
#define TUBE_COUNT              4U      // 4 tube positions, 90 deg apart (matches the ESC)
#define ROTATE_MOVE_TIMEOUT_MS 20000U   // max time to reach the tube -> else fault. Covers the
                                        // ESC crawl's settle dwells + stiction-kick retries AND
                                        // the first-index-after-boot case (FOC align ~3-5 s
                                        // precedes the move) -- bench-measured 2026-07-17

// Rotate implementation. DEFAULT (flag ON, 2026-07-17) = the ESC's native closed-loop
// INDEX verb: a SimpleFOC angle-mode servo move to the tube detent -- shortest path
// (bidirectional), smooth decel, ~2 deg landing, active hold until the lock seats.
// Requires the updated ESC firmware (INDEX + HOME verbs). Comment the flag OUT to fall
// back to the legacy Nano-side crawl (SPIN + STAT-angle polling, forward-only) for an
// ESC still running the old flash.
#define ROTATE_VIA_INDEX

// Nano-side velocity-feedback rotate tunables (direct 1:1 drive: 90 deg = 0.25 rev).
// The Nano crawls the gantry forward to an ABSOLUTE detent, read from the encoder angle
// the ESC reports (STAT ang=). Detents are learned relative to a homed reference.
//
// The assembly has heavy stiction (~4.5 A to break away). The spin's velocity PID is tuned
// SOFT (P=0.05) so at a slow crawl it barely commands current -> stalls. So during the
// rotate we (a) boost the velocity P for real low-speed torque, and (b) run a TWO-SPEED
// crawl: fast to break away + cover ground, slow for a precise final approach. Gains are
// restored (applyEscTuning) when the move ends.
// PRECISION MODEL: the crawl is FORWARD-ONLY (no reverse), so when it stops it COASTS PAST
// the target by some distance C. It stops when it gets within ARRIVE_TOL *before* the detent,
// so the landing error = C - ARRIVE_TOL. To hit the detent: (1) keep C small + consistent by
// approaching slowly (low SLOW_RPM), and (2) lead the stop by ~C via ARRIVE_TOL. We bias the
// stop EARLY (ARRIVE_TOL a bit above C) so any error is a small UNDERSHOOT (the lock taper can
// still pull it in) rather than an overshoot (which it can't back out of).
// BENCH TUNING: still overshoots -> raise ARRIVE_TOL and/or lower SLOW_RPM. Undershoots or
// stalls short -> lower ARRIVE_TOL and/or raise SLOW_RPM (stiction can stall a too-slow crawl).
#define ROTATE_VEL_P            1.5f    // boosted velocity PID P during the crawl (torque)
#define ROTATE_FAST_RPM         40      // coarse speed: break stiction + cover distance. Kept low
                                        // enough that it can decelerate within COARSE_TOL and NOT
                                        // overshoot the detent (at 80 the decel ran ~60 deg > zone).
#define ROTATE_SLOW_RPM         15      // fine speed near the detent: lower = less coast + variance
                                        // (was 20; drop toward ~10 if it still overshoots, raise if it stalls)
#define ROTATE_COARSE_TOL_REV   0.11f   // start the slow approach earlier (~40 deg out) so it is
                                        // fully settled at SLOW speed well before the detent
#define ROTATE_ARRIVE_TOL_REV   0.03f   // stop ~11 deg BEFORE the detent to absorb the coast (was
                                        // 0.02/~7 deg = landed past it); lands on/just-short of the detent
#define ROTATE_POLL_MS          25U     // poll the ESC angle more often -> detect arrival sooner ->
                                        // less latency-driven overshoot (was 40)
#define ROTATE_SETTLE_MS        350U    // angle stable this long after stopping = arrived
#define ROTATE_STABLE_REV       0.004f  // rev change below this counts as "not moving"
// Door-move watchdog: if the target hall never goes active, the motor is stopped and
// the move is abandoned. Generous during bring-up because the door is not attached yet
// and the endpoint magnets are presented by hand. Tighten once the door is mounted.
#define DOOR_MOVE_TIMEOUT_MS 6000U

// Door motor drive shaping (shared Due/Nano): a brief KICK to break the N20 worm-gear
// static friction, then settle to a slow PWM run. All duties are 0-255.
#define DOOR_KICK_MS      90U    // kick duration (shorter = less lunge at start)
#define DOOR_KICK_DUTY    170    // kick strength (~67%); raised from 150 (2026-08-03,
                                 // more door strength request) -- lower = gentler kick
#define DOOR_PWM_DUTY     220    // run duty (~86%) after the kick; raised from 192
                                 // (same request). DOOR_SPEED verb still overrides live.
                                 // Duty 255 == the N20's full 12 V rating (VM-rescaled),
                                 // so ~15% headroom remains before the motor's ceiling.
// Motor direction vs the door mechanism. Set to 1 if OPEN/CLOSE drive the wrong way
// (swaps the DRV8871 outputs in firmware instead of rewiring OUT1/OUT2).
#define DOOR_DIR_INVERT   1

// Door-motor supply scaling. The N20 door motor (BOM M2) is a 12 V-max part; on the
// 24 V single-bus rework its DRV8871 VM rides the 24 V rail directly (no room for the
// BOM's P2 12 V buck). The 20 kHz slow-decay H-bridge PWM into the winding inductance
// is electrically a buck stage, so updateDoorMotor() rescales every duty above (all
// calibrated on the original 12 V rail) to keep the motor's AVERAGE voltage at its
// 12 V rating -- commanded duty 255 means "12 V", never full VM. The full-VM bench
// tests (door_digi_test / door_test / door_move_test) refuse to build while this is
// above the motor rating. Set back to 12 if VM is ever rewired to a 12 V rail.
#define DOOR_VM_VOLTAGE     24
#define DOOR_MOTOR_RATED_V  12

// Fan control: normal firmware drives the fan full-speed only while there is
// motor activity, then holds it on for a cooldown. POWER_OFF still forces it off.
#define FAN_COOLDOWN_MS           60000U
#define FAN_RPM_ACTIVE_THRESHOLD  100
#define FAN_RUNS_DURING_ROTATE    1
#define FAN_RUNS_DURING_ESC_CAL   1

// NTC (100k divider) thresholds in raw ADC counts (12-bit ADC)
// Lower ADC == hotter for pullup-divider wiring.
#define NTC_ADC_FAN_ON 1400
#define NTC_ADC_FAN_OFF 1700
#define NTC_ADC_CRITICAL_LOW 1100

#endif
