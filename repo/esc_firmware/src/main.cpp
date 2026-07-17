// ===========================================================================
// Centrifuge spindle ESC firmware  -  B-G431B-ESC1 (MB1419C) + SimpleFOC
//   Motor:   EMAX GT2215/10, 1100 KV, 14-pole  -> 7 pole pairs
//   Encoder: AS5047P-TS_EK_AB, read via its ABI QUADRATURE output (A/B/Z),
//            NOT SPI. PB6/PB7/PB8 on this board are the encoder A+/B+/Z+ pins
//            (UM2516 Table 4). The AS5047P emits 1000 PPR / 4000 CPR by default
//            with zero register programming -- so there is no fragile bit-banged
//            SPI to fail (the previous bring-up died on SPI reading 0xFFFF).
//
// PHILOSOPHY: flash ONCE, then bring the system up stage by stage over serial.
// Nothing energizes the motor until you explicitly `arm`. Each stage proves one
// thing the next stage depends on:
//
//   stage 1  ENCODER     (no bus power)  turn shaft by hand; 1 turn == 1.000 rev
//   stage 2  OPENLOOP    (bus power)     gentle spin, no feedback; checks phases
//   stage 3  CURRENT     (bus power)     print phase currents; checks cal
//   stage 4  CLOSEDLOOP  (bus power)     FOC velocity + smooth bounded ramp
//
// SAFETY for this 0.1 ohm motor: at standstill, applied voltage / R = current,
// so even ~1 V open-loop pulls ~10 A. The PSU current limit (set it to ~1-2 A)
// is the real backstop. Voltages here are deliberately tiny; raise slowly.
// ===========================================================================
#include <Arduino.h>

#ifndef ENCODER_DIAG   // [env:diag] builds src/diag.cpp instead of this file
#include <SimpleFOC.h>
#include <stdlib.h>            // atof
#include <string.h>            // strcmp
#include "sensorless_observer.h"
// Quadrature is decoded by TIM4 in silicon -> zero CPU load at any RPM (an
// interrupt-based Encoder would saturate the core past ~4-5k RPM at 4000 CPR).
// We program TIM4 directly (below) instead of the SimpleFOCDrivers STM32HWEncoder,
// whose init() silently failed on this board/core combo.

// ----------------------------- tunables ------------------------------------
#ifndef HIGH_SPEED_SENSORLESS
#define HIGH_SPEED_SENSORLESS 0
#endif

#define POLE_PAIRS         7        // GT2215/10: 14 magnets / 2
#ifndef SUPPLY_VOLTAGE
#define SUPPLY_VOLTAGE     12.0f    // bench PSU (3S equivalent). For the 24 V bus build
                                    // with `-D SUPPLY_VOLTAGE=24.0f` -- every voltage cap and
                                    // the reachable-RPM clamp below scale from this.
#endif

// Low-side current sense ONLY reads cleanly below ~VBUS/2 duty, so every voltage
// ceiling here is capped at VBUS/2. (Confirmed for MB1419C on the SimpleFOC forum.)
#define VBUS_HALF          (SUPPLY_VOLTAGE * 0.5f)   // 6.0 V on a 12 V bus

// On a ~0.1 ohm motor, applied volts / (~1.5*R) = amps, with NO current regulation
// during alignment -- so the PSU current limit (~1.5-2 A) is the only backstop and
// these voltages must stay tiny. 0.15 V -> ~1 A align; 0.3 V open-loop; 3 V closed
// (raise toward VBUS/2=6 V only once spinning smoothly and you need more RPM).
// Measured: 0.30 V open-loop -> ~0.95 A, so effective R ~0.32 ohm (higher than the
// 0.1 ohm nameplate). 0.15 V align gave only ~0.5 A -> too little torque to seat the
// rotor -> bad zero_electric_angle + PP check fail + closed-loop judder/overcurrent.
// Tunable live with `k <V>` (align) and `l <V>` (closed-loop limit).
// Tuned for the FULL centrifuge in its housing: the assembly has high static
// friction (stiction) -- it needs ~4.5 A to break away from standstill, then only
// ~2-3 A to keep spinning (measured on the bench, 5 A PSU). The bare-motor values
// (0.6 V align / 1.5 A) could not seat or spin the loaded shaft. Live: `k` / `c`.
#define ALIGN_VOLTAGE      1.40f    // FOC alignment push (~4.4 A; needs PSU Ilim ~5 A)
#define OPENLOOP_VOLTAGE   0.30f    // stage-2/3 open-loop voltage cap (voltage torque mode)
#define BRINGUP_CURRENT    5.0f     // closed-loop CURRENT limit (foc_current) -- enough to
                                    // break the loaded shaft loose; the real safety cap. Live: `c`.
#define MOTOR_VOLT_LIMIT   5.0f     // closed-loop voltage ceiling: high enough for 4000 RPM
                                    // back-EMF (~3.6 V). Safe because foc_current bounds the
                                    // current; voltage just follows what BEMF needs. (<= VBUS/2)

// ===== High-speed profile: encoder FOC <-> sensorless PLL ====================
// Closed-loop FOC (AS5047P encoder + current sense) runs 0..CROSSOVER_RPM. With
// HIGH_SPEED_SENSORLESS=1, the observer shadows encoder FOC near the crossover,
// then supplies the high-speed electrical angle after it proves lock. Default
// builds keep the production 4000 RPM ceiling and never enter high-speed mode.
//
// !!! REACHABLE TOP SPEED IS BOUNDED BY THE SUPPLY !!! back-EMF ~ rpm/KV, and the
// phases can only be driven to ~VBUS/2. At 12 V that caps ~6k RPM; 10k needs a 6S
// (~24 V) pack -> set SUPPLY_VOLTAGE=24 and reflash. Everything below is relative
// to SUPPLY_VOLTAGE so it scales automatically.
//
// !!! UNTUNED ABOVE THE CROSSOVER !!! the sensorless observer + handoff have NOT
// been validated on hardware. Validate incrementally with containment/current
// limit before real use.
// Validate incrementally on the final supply, with containment, before real use.
#define MOTOR_KV           1100.0f  // GT2215/10: 1100 RPM per volt of back-EMF
#define CROSSOVER_RPM      4000.0f  // closed-loop below, open-loop above
#define CROSSOVER_HYST_RPM  300.0f  // hysteresis so it doesn't chatter at the boundary
#define OL_BEMF_MARGIN      0.8f    // open-loop drives back-EMF + this margin (keeps current low)
#define OL_VOLT_CAP        VBUS_HALF // open-loop voltage ceiling (== driver limit; raise supply for more)
#if HIGH_SPEED_SENSORLESS
// Commandable ceiling is SUPPLY-AWARE: the design target is 10k, but the drive can only
// hold sync while BEMF + OL_BEMF_MARGIN fits under OL_VOLT_CAP. Clamping the accepted
// target here (instead of letting it sail past and fault) makes the physics explicit:
//   12 V -> KV*(6.0-0.8)  = 5720 RPM   |   24 V -> KV*(12.0-0.8) = 12320 -> clamped 10000.
#define MAX_SPIN_RPM_DESIGN 10000.0f
#define MAX_SPIN_RPM_SUPPLY (MOTOR_KV * (OL_VOLT_CAP - OL_BEMF_MARGIN))
#define MAX_SPIN_RPM        (MAX_SPIN_RPM_SUPPLY < MAX_SPIN_RPM_DESIGN ? MAX_SPIN_RPM_SUPPLY : MAX_SPIN_RPM_DESIGN)
#else
#define MAX_SPIN_RPM      CROSSOVER_RPM
#endif
#define SENSORLESS_SHADOW_RPM       (CROSSOVER_RPM - 500.0f)
#define SENSORLESS_LOCK_DWELL_MS    300.0f
#define SENSORLESS_LOCK_ERR_RAD     (15.0f * _PI / 180.0f)
#define SENSORLESS_LOCK_RPM_ERR     300.0f
#define SENSORLESS_MIN_BEMF_V       1.0f
#define SENSORLESS_TRIAL_CURRENT_A  5.0f
#define MOTOR_PHASE_RESISTANCE_OHM  0.32f
#define MOTOR_PHASE_INDUCTANCE_H    0.000010f

// ===== Gantry indexing (tube positions) =====================================
// The gantry is DIRECT-driven (1:1), so 90 deg of the motor shaft = one tube position.
// Closed-loop angle control (SimpleFOC MotionControlType::angle) drives to an ABSOLUTE
// tube angle using the AS5047P. Tube 0 = the shaft angle at power-on (the operator homes
// the gantry to a detent first); tubes 1-3 at +90/180/270 deg. The mechanical detent +
// lock taper pin give the final precision within a detent. Verb: `INDEX <0-3>`.
#define TUBE_COUNT        4
#define TUBE_STEP_RAD     (REV / TUBE_COUNT)    // 90 deg per tube (direct 1:1 drive)
#define INDEX_VEL_LIMIT   (1.0f * REV)          // slow, controlled index move (1 rev/s)
#define INDEX_TOL_RAD     0.035f                // ~2 deg arrival tolerance
#define INDEX_P_ANGLE     10.0f                 // position-loop P gain (gentle)
#define MOTOR_VEL_LIMIT   (200.0f * REV)        // spin velocity ceiling (restored after index)

// MB1419C current sense: 3 mOhm shunt, gain -64/7 (forum-confirmed for Rev C).
#define SHUNT_OHMS         0.003f
#define CS_GAIN           (-64.0f / 7.0f)

// AS5047P ABI: 1000 pulses/rev default. SimpleFOC wants PPR; it x4 internally.
#define ENCODER_PPR        1000

static const float REV = _2PI;     // rad per revolution
// ---------------------------------------------------------------------------

// ----- Encoder: TIM4 quadrature, programmed directly ------------------------
// A -> PB6 (TIM4_CH1, AF2), B -> PB7 (TIM4_CH2, AF2). x4 decoding, the counter
// wraps once per mechanical revolution (ARR = CPR-1) so getSensorAngle() maps
// straight to [0, 2pi) and the SimpleFOC Sensor base handles rotation counting.
#define ENCODER_CPR  (4u * ENCODER_PPR)

class TIM4Encoder : public Sensor {
 public:
  void init() override {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;              // harmless on the encoder's push-pull outputs
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &g);

    TIM4->CR1   = 0;
    TIM4->CCMR1 = 0x3131;                   // CC1S/CC2S=TI, IC1F/IC2F=8-sample filter
    TIM4->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM4->SMCR  = 0x0003;                   // SMS=011: encoder mode 3 (count both TI1+TI2 = x4)
    TIM4->PSC   = 0;
    TIM4->ARR   = ENCODER_CPR - 1u;
    TIM4->CNT   = 0;
    TIM4->CR1  |= TIM_CR1_CEN;
    Sensor::init();                         // seed angle/velocity tracking
  }
  float getSensorAngle() override {
    return _2PI * (float)(TIM4->CNT) / (float)ENCODER_CPR;
  }
};

TIM4Encoder encoder;

BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver6PWM driver =
    BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
LowsideCurrentSense currentSense =
    LowsideCurrentSense(SHUNT_OHMS, CS_GAIN, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// ----- stage / run state ---------------------------------------------------
enum Stage { IDLE = 0, ENCODER = 1, OPENLOOP = 2, CURRENT = 3, CLOSEDLOOP = 4 };
enum SpinControlMode { ENCODER_FOC = 0, OBSERVER_SHADOW = 1, SENSORLESS_PLL = 2, FAULT_RAMPDOWN = 3 };
static Stage stage = IDLE;
static bool  armed = false;          // motor energized? (stages 2-4 require arm)
static bool  foc_ready = false;      // initFOC() (alignment) done once
static SpinControlMode spin_mode = ENCODER_FOC;
static uint32_t sensorless_last_us = 0;
static uint32_t sensorless_enter_ms = 0;
static bool sensorless_loss_reported = false;
static float g_observer_phase_err = 0.0f;
static float g_observer_rpm = 0.0f;
static float g_observer_bemf = 0.0f;
static float g_last_current_a = 0.0f;
static SensorlessObserver observer;

struct PidProfile {
  const char *name;
  float velocityP;
  float velocityI;
  float velocityTf;
  float currentP;
  float currentI;
};

static const PidProfile SPIN_PROFILE  = {"spin",  0.05f, 0.10f, 0.15f, 0.3f, 20.0f};
static const PidProfile CRAWL_PROFILE = {"crawl", 1.50f, 0.10f, 0.15f, 0.3f, 20.0f};
static const PidProfile *active_pid_profile = &SPIN_PROFILE;

enum CalibrationState {
  CAL_IDLE = 0,
  CAL_ALIGN,
  CAL_ENCODER_CHECK,
  CAL_CURRENT_CHECK,
  CAL_LOW_SPEED_PID,
  CAL_OBSERVER_SHADOW,
  CAL_DONE,
  CAL_FAILED
};

static CalibrationState cal_state = CAL_IDLE;
static uint32_t cal_state_ms = 0;
static uint16_t cal_encoder_start = 0;
static uint8_t cal_step = 0;
static float cal_target_rpm = 0.0f;
static float cal_max_abs_rpm_err = 0.0f;
static float cal_sum_abs_rpm_err = 0.0f;
static float cal_max_current = 0.0f;
static float cal_saved_ramp_accel = 5.0f * REV;
static uint16_t cal_samples = 0;
static char cal_error[24] = "none";

static const float CAL_RPM_STEPS[] = {500.0f, 1000.0f, 2000.0f, 3000.0f, 4000.0f};
static const uint8_t CAL_RPM_STEP_COUNT = sizeof(CAL_RPM_STEPS) / sizeof(CAL_RPM_STEPS[0]);
#define CAL_LOCK_RELEASE_MS        1500U  // dwell before align: master retracts the gantry lock
#define CAL_ALIGN_TIMEOUT_MS       6000U  // includes the lock-release dwell
#define CAL_ENCODER_CHECK_MS        350U
#define CAL_CURRENT_CHECK_MS        700U
#define CAL_PID_STEP_MS            1200U
#define CAL_OBSERVER_SHADOW_MS     1200U
#define CAL_ENCODER_STILL_COUNTS    200
#define CAL_RPM_ERROR_LIMIT         800.0f

// ----- gantry indexing (angle-control position move) ------------------------
static bool  index_mode    = false;  // true = closed-loop angle move/hold to a tube
static bool  index_arrived = false;  // reached the target angle (within tolerance)
static int   index_tube    = 0;      // last commanded tube (0..TUBE_COUNT-1)
static float index_target  = 0.0f;   // target cumulative angle (rad)

static float cmd_vel   = 0.0f;       // requested final velocity (rad/s)
static float ramp_vel  = 0.0f;       // ramped setpoint actually commanded
static float ramp_accel = 5.0f * REV;// default 5 rev/s^2 -- the "very smooth" knob
static uint32_t last_ramp_us = 0;
static uint32_t last_tele_ms = 0;
static uint32_t last_st_ms   = 0;    // ST stream cadence (link mode, ~10 Hz)
static uint32_t g_loop_hz = 0;       // measured loop()/FOC rate -- must be >> electrical Hz

// ----- high-level Due-facing link (SPIN/STOP/ESTOP/PING/STATUS?) ------------
// The Due drives the ESC with high-level verbs over UART; the bench single-char
// commands above stay available for debugging. "link mode" engages on the first
// verb and switches telemetry from the verbose STAT line to a compact ST stream.
static bool     link_mode       = false;  // a Due-facing verb has been seen
static uint32_t last_link_ms    = 0;      // last valid verb / PING (watchdog feed)
static bool     stop_then_disarm = false; // STOP/watchdog: cut torque once stopped
static bool     wd_tripped      = false;  // watchdog fired (latched until next verb)
static const uint32_t LINK_WD_MS = 750;   // comms watchdog timeout (~0.5-1 s spec)

// ----- helpers -------------------------------------------------------------
static const char *spinModeName() {
  switch (spin_mode) {
    case ENCODER_FOC:     return "spin-enc";
    case OBSERVER_SHADOW: return "spin-shadow";
    case SENSORLESS_PLL:  return "spin-sensorless";
    case FAULT_RAMPDOWN:  return "spin-fault-ramp";
    default:              return "spin-unknown";
  }
}

static bool sensorlessModeActive() {
  return spin_mode == SENSORLESS_PLL || spin_mode == FAULT_RAMPDOWN;
}

static const char *calStateName() {
  switch (cal_state) {
    case CAL_IDLE:            return "idle";
    case CAL_ALIGN:           return "align";
    case CAL_ENCODER_CHECK:   return "encoder-check";
    case CAL_CURRENT_CHECK:   return "current-check";
    case CAL_LOW_SPEED_PID:   return "low-speed-pid";
    case CAL_OBSERVER_SHADOW: return "observer-shadow";
    case CAL_DONE:            return "done";
    case CAL_FAILED:          return "failed";
    default:                  return "unknown";
  }
}

static bool calibrationRunning() {
  return cal_state != CAL_IDLE && cal_state != CAL_DONE && cal_state != CAL_FAILED;
}

static bool calibrationMotionActive() {
  return calibrationRunning();
}

static void abortCalibration(const char *reason, CalibrationState finalState) {
  ramp_accel = cal_saved_ramp_accel;
  cal_target_rpm = 0.0f;
  cal_step = 0;
  strncpy(cal_error, reason, sizeof(cal_error) - 1U);
  cal_error[sizeof(cal_error) - 1U] = '\0';
  cal_state = finalState;
  cal_state_ms = millis();
}

static void applyPidProfile(const PidProfile &profile) {
  active_pid_profile = &profile;
  motor.PID_velocity.P = profile.velocityP;
  motor.PID_velocity.I = profile.velocityI;
  motor.PID_velocity.output_ramp = 200.0f;
  motor.LPF_velocity.Tf = profile.velocityTf;
  motor.PID_current_q.P = profile.currentP;
  motor.PID_current_q.I = profile.currentI;
  motor.PID_current_q.limit = MOTOR_VOLT_LIMIT;
  motor.PID_current_d.P = profile.currentP;
  motor.PID_current_d.I = profile.currentI;
  motor.PID_current_d.limit = MOTOR_VOLT_LIMIT;
}

static void printTune() {
  Serial.print("TUNE profile="); Serial.print(active_pid_profile->name);
  Serial.print(" velP=");        Serial.print(motor.PID_velocity.P, 4);
  Serial.print(" velI=");        Serial.print(motor.PID_velocity.I, 4);
  Serial.print(" velTf=");       Serial.print(motor.LPF_velocity.Tf, 4);
  Serial.print(" curP=");        Serial.print(motor.PID_current_q.P, 4);
  Serial.print(" curI=");        Serial.print(motor.PID_current_q.I, 4);
  Serial.print(" ilim=");        Serial.print(motor.current_limit, 3);
  Serial.println(" save=0");
}

static void disarm(const char *why) {
  motor.disable();
  armed = false;
  cmd_vel = ramp_vel = 0.0f;
  // Always leave the motor in closed-loop VELOCITY config so the next arm starts there
  // (a disarm from the open-loop or indexing regime must not strand those settings).
  spin_mode = ENCODER_FOC;
  sensorless_loss_reported = false;
  index_mode = false;
  applyPidProfile(SPIN_PROFILE);       // undo an index move's CRAWL gains
  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.voltage_limit = MOTOR_VOLT_LIMIT;
  motor.velocity_limit = MOTOR_VEL_LIMIT;   // restore the spin ceiling (index lowers it)
  Serial.print("DISARMED ("); Serial.print(why); Serial.println(")");
}

static void printStat() {
  // Displayed velocity is averaged over the telemetry interval. The Sensor's own
  // getVelocity() is quantization-noisy because loopFOC() updates it at kHz rates
  // (few encoder counts per update at low speed). Do NOT call encoder.update() here
  // -- loop()/loopFOC() own it; an extra update would corrupt the velocity the
  // closed-loop PID reads.
  static float    last_rev = 0.0f;
  static uint32_t last_us  = 0;
  float now_rev = encoder.getAngle() / REV;                 // cumulative revs
  uint32_t now_us = micros();
  float dt = (now_us - last_us) * 1e-6f;
  float tele_vel = (last_us != 0 && dt > 1e-4f) ? (now_rev - last_rev) / dt : 0.0f;
  last_rev = now_rev;
  last_us  = now_us;

  Serial.print("STAT stage="); Serial.print((int)stage);
  Serial.print(" armed=");     Serial.print(armed ? 1 : 0);
  Serial.print(" rev=");       Serial.print(now_rev, 3);
  Serial.print(" ang=");       Serial.print(encoder.getMechanicalAngle(), 3); // [0,2pi)
  Serial.print(" cnt=");       Serial.print((uint32_t)(TIM4->CNT));          // raw TIM4 counter
  Serial.print(" hz=");        Serial.print(g_loop_hz);                       // FOC loop rate
  Serial.print(" vel=");       Serial.print(tele_vel, 3);                     // rev/s, interval-averaged
  Serial.print(" tgt=");       Serial.print(ramp_vel / REV, 3);
  Serial.print(" cmd=");       Serial.print(cmd_vel / REV, 3);
  Serial.print(" mode=");      Serial.print(spinModeName());
  Serial.print(" cal=");       Serial.print(calStateName());
  Serial.print(" cal_step=");  Serial.print(cal_step);
  Serial.print(" obs_rpm=");   Serial.print(g_observer_rpm, 0);
  Serial.print(" obs_lock=");  Serial.print(observer.locked() ? 1 : 0);
  Serial.print(" phase_err="); Serial.print(g_observer_phase_err * 180.0f / _PI, 1);
  Serial.print(" bemf=");      Serial.print(g_observer_bemf, 2);
  if (stage == CURRENT || stage == CLOSEDLOOP) {
    PhaseCurrent_s c = currentSense.getPhaseCurrents();
    float mag = currentSense.getDCCurrent();
    Serial.print(" Ia="); Serial.print(c.a, 2);
    Serial.print(" Ib="); Serial.print(c.b, 2);
    Serial.print(" Ic="); Serial.print(c.c, 2);
    Serial.print(" Idc="); Serial.print(mag, 2);
  }
  Serial.println();
}

// Move ramp_vel toward cmd_vel by at most ramp_accel*dt -> bounded acceleration.
static void updateRamp() {
  uint32_t now = micros();
  float dt = (now - last_ramp_us) * 1e-6f;
  last_ramp_us = now;
  if (dt <= 0 || dt > 0.1f) return;             // ignore absurd dt
  float step = ramp_accel * dt;
  if      (ramp_vel < cmd_vel) ramp_vel = _constrain(ramp_vel + step, ramp_vel, cmd_vel);
  else if (ramp_vel > cmd_vel) ramp_vel = _constrain(ramp_vel - step, cmd_vel, ramp_vel);
}

// ----- dual-mode (closed-loop <-> open-loop) --------------------------------
// Open-loop applied voltage vs speed: just above back-EMF so current stays small
// (I = (Vapplied - BEMF)/R). Capped at OL_VOLT_CAP (== driver ceiling).
static float olVoltage(float rpm) {
  float v = rpm / MOTOR_KV + OL_BEMF_MARGIN;
  return _constrain(v, OL_BEMF_MARGIN, OL_VOLT_CAP);
}

// Hand off encoder FOC -> sensorless PLL. Seed the observer from the calibrated
// encoder angle, then let its PLL carry the electrical angle at high speed.
static void enterSensorlessMode(float encoderElectricalAngle, float encoderRpm) {
  spin_mode = SENSORLESS_PLL;
  sensorless_loss_reported = false;
  observer.resetFromElectrical(encoderElectricalAngle, encoderRpm);
  sensorless_last_us = micros();
  sensorless_enter_ms = millis();
  Serial.println("MODE sensorless PLL (> crossover)");
}

// Hand off sensorless -> encoder FOC. The encoder is kept fresh during sensorless
// operation so FOC can re-lock below the crossover.
static void enterClosedLoopMode() {
  spin_mode = ENCODER_FOC;
  sensorless_loss_reported = false;
  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.voltage_limit = MOTOR_VOLT_LIMIT;
  Serial.println("MODE encoder FOC (< crossover)");
}

static void enterFaultRampdown(const char *why) {
  if (!sensorless_loss_reported) {
    Serial.print("MODE sensorless fault rampdown: ");
    Serial.println(why);
    sensorless_loss_reported = true;
  }
  spin_mode = FAULT_RAMPDOWN;
  cmd_vel = 0.0f;
}

// Configure the motor for a stage and (for powered stages) energize it.
static void enterStage(Stage s) {
  // Always start from a safe, de-energized state.
  disarm("stage change");
  stage = s;
  ramp_vel = cmd_vel = 0.0f;
  last_ramp_us = micros();

  switch (s) {
    case IDLE:
      Serial.println("STAGE 0 IDLE  -- motor off.");
      break;
    case ENCODER:
      Serial.println("STAGE 1 ENCODER -- bus power OFF. Turn shaft by hand:");
      Serial.println("  expect rev to track turns (1 full turn == 1.000), smooth vel.");
      Serial.println("  if rev counts backward, A/B are swapped (or that's our +dir).");
      break;
    case OPENLOOP:
      motor.controller = MotionControlType::velocity_openloop;
      motor.torque_controller = TorqueControlType::voltage;
      motor.voltage_limit = OPENLOOP_VOLTAGE;   // tiny: 0.1 ohm motor
      Serial.println("STAGE 2 OPENLOOP -- bus power ON, PSU current-limited ~1-2 A.");
      Serial.println("  `arm`, then `v 1` for a gentle spin. Watch encoder rev follow.");
      break;
    case CURRENT:
      motor.controller = MotionControlType::velocity_openloop;
      motor.torque_controller = TorqueControlType::voltage;
      motor.voltage_limit = OPENLOOP_VOLTAGE;
      Serial.println("STAGE 3 CURRENT -- `arm` then `v 0`..`v 2`; STAT prints Ia/Ib/Ic.");
      Serial.println("  at rest Ia/Ib/Ic ~ 0; they should look balanced & sane spinning.");
      break;
    case CLOSEDLOOP:
      motor.controller = MotionControlType::velocity;
      motor.torque_controller = TorqueControlType::foc_current;  // current-bounded at any speed
      motor.voltage_limit = MOTOR_VOLT_LIMIT;
      foc_ready = false;             // re-align on the next arm (tuning convenience)
      Serial.print("STAGE 4 CLOSEDLOOP (foc_current) -- arm re-aligns. align="); Serial.print(motor.voltage_sensor_align, 2);
      Serial.print("V Ilim="); Serial.print(motor.current_limit, 2); Serial.println("A");
      Serial.println("  `c <A>`=current limit, `q <P>`=current P, `p/i/f`=vel PID, `a`/`v`/`s`/`x`.");
      break;
  }
}

static void arm() {
  if (stage == IDLE || stage == ENCODER) {
    Serial.println("ERR nothing to arm in this stage");
    return;
  }
  last_ramp_us = micros();
  // The driver MUST be live before initFOC(): alignSensor() drives the phases via
  // setPhaseVoltage(), and a disabled driver forces duty=0 -> alignment runs at 0 V
  // and produces a garbage/failed zero_electric_angle. enable() first.
  motor.enable();
  if (stage == CLOSEDLOOP && !foc_ready) {
    Serial.println("INIT aligning FOC (motor twitches once -- keep it clamped)...");
    if (!motor.initFOC()) {              // initFOC() disables the driver on failure
      motor.disable();
      Serial.println("ERR initFOC FAILED");
      return;
    }
    foc_ready = true;
    Serial.print("INIT zero_electric_angle="); Serial.println(motor.zero_electric_angle, 4);
    Serial.print("INIT sensor_direction=");    Serial.println((int)motor.sensor_direction);
  }
  armed = true;
  Serial.println("ARMED");
}

// ----- high-level Due-facing verb handlers ---------------------------------
// Compact telemetry the Due parses: actual RPM, target RPM, link state, current.
static void printST() {
  // Interval-averaged measured velocity (same method as printStat()); its own
  // statics so it doesn't fight printStat()'s averaging. Read-only on the encoder.
  static float    last_rev = 0.0f;
  static uint32_t last_us  = 0;
  float now_rev = encoder.getAngle() / REV;
  uint32_t now_us = micros();
  float dt = (now_us - last_us) * 1e-6f;
  float vel = (last_us != 0 && dt > 1e-4f) ? (now_rev - last_rev) / dt : 0.0f;
  last_rev = now_rev;
  last_us  = now_us;

  float rpm     = vel * 60.0f;
  float tgt_rpm = (cmd_vel / REV) * 60.0f;
  float cur = (armed && stage == CLOSEDLOOP && !index_mode) ? g_last_current_a : 0.0f;
  const char *st = !armed ? "idle"
                          : index_mode ? (index_arrived ? "indexed" : "indexing")
                          : (stop_then_disarm ? "stopping" : spinModeName());

  Serial.print("ST rpm=");   Serial.print(rpm, 0);
  Serial.print(" tgt=");     Serial.print(tgt_rpm, 0);
  Serial.print(" state=");   Serial.print(st);
  Serial.print(" hz=");      Serial.print(g_loop_hz);
  Serial.print(" cur=");     Serial.print(cur, 2);
  Serial.print(" cal=");     Serial.print(calStateName());
  Serial.print(" cal_step="); Serial.print(cal_step);
  Serial.print(" cal_target="); Serial.print(cal_target_rpm, 0);
  Serial.print(" cal_err="); Serial.print(cal_error);
  Serial.print(" obs_rpm="); Serial.print(g_observer_rpm, 0);
  Serial.print(" obs_lock="); Serial.print(observer.locked() ? 1 : 0);
  Serial.print(" phase_err="); Serial.print(g_observer_phase_err * 180.0f / _PI, 1);
  Serial.print(" bus=");     Serial.print(SUPPLY_VOLTAGE, 1);
  Serial.print(" ilim=");    Serial.print(motor.current_limit, 2);
  Serial.print(" pos=");     Serial.print(encoder.getMechanicalAngle() * 180.0f / _PI, 1);  // deg [0,360)
  Serial.print(" tube=");    Serial.print(index_tube);
  Serial.println();
}

static void linkSpin(float rpm) {
  link_mode = true; last_link_ms = millis(); wd_tripped = false;
  if (rpm < 0) rpm = 0;
  if (rpm > MAX_SPIN_RPM) rpm = MAX_SPIN_RPM;   // dual-mode ceiling (supply-bounded in reality)
  // Ensure we are in current-bounded closed loop and aligned/armed. enterStage()
  // disarms + forces a re-align, so only call it if we're not already there.
  if (stage != CLOSEDLOOP) enterStage(CLOSEDLOOP);
  if (!armed) arm();                   // aligns on first arm; needs the 12 V bus
  if (!armed) { Serial.println("ERR SPIN not armed (bus power? align failed)"); return; }
  stop_then_disarm = false;
  cmd_vel = (rpm / 60.0f) * REV;       // RPM -> rad/s; updateRamp() ramps to it
  Serial.print("OK SPIN "); Serial.println(rpm, 0);
}

static void linkStop() {
  link_mode = true; last_link_ms = millis(); wd_tripped = false;
  if (calibrationRunning() || cal_state == CAL_DONE) {
    abortCalibration("stop", CAL_IDLE);
    disarm("cal stop");
    Serial.println("OK STOP");
    return;
  }
  if (index_mode) {
    disarm("index stop");              // leaves angle-hold; lock now holds the gantry
    Serial.println("OK STOP");
    return;
  }
  cmd_vel = 0.0f;
  stop_then_disarm = true;             // disarm once the ramp reaches 0
  Serial.println("OK STOP");
}

// Detent reference for tube indexing: the mechanical angle of tube 0's detent. Defaults
// to the power-on shaft angle (the operator boots with the gantry homed at a detent);
// re-captured any time the master sends HOME with the gantry parked at a detent.
static float home_ref_rad = 0.0f;

// HOME: capture the CURRENT shaft angle as tube 0's detent reference. Sent by the master
// at boot and from the UI's "Set home" (gantry parked at a detent, rotor stopped).
static void linkHome() {
  link_mode = true; last_link_ms = millis(); wd_tripped = false;
  home_ref_rad = _normalizeAngle(encoder.getAngle());
  Serial.print("OK HOME "); Serial.println(home_ref_rad, 4);
}

// Closed-loop angle move to a tube position (0..TUBE_COUNT-1). The gantry MUST be
// unlocked before this (the Due sequences that). Takes the shortest path; on arrival it
// holds the angle until the Due locks the gantry and sends STOP to disarm.
static void linkIndex(int tube) {
  link_mode = true; last_link_ms = millis(); wd_tripped = false;
  tube = ((tube % TUBE_COUNT) + TUBE_COUNT) % TUBE_COUNT;
  index_tube = tube;
  // Refuse to enter a position move while the rotor is actually turning -- an index
  // is only ever valid from rest (the master sequences lock release around it).
  if (fabsf((encoder.getVelocity() / REV) * 60.0f) > 100.0f) {
    Serial.println("ERR INDEX rotor moving");
    return;
  }
  if (stage != CLOSEDLOOP) enterStage(CLOSEDLOOP);
  if (!armed) arm();                   // aligns on the first arm (needs bus power)
  if (!armed) { Serial.println("ERR INDEX not armed (bus power? align failed)"); return; }

  // Shortest move to the tube's ABSOLUTE detent angle (home_ref_rad = tube 0's detent).
  float target_mech = _normalizeAngle(home_ref_rad + (float)tube * TUBE_STEP_RAD);
  float cur = encoder.getAngle();                  // cumulative rad
  float delta = target_mech - _normalizeAngle(cur);
  while (delta >  _PI) delta -= _2PI;              // shortest path, [-pi, pi]
  while (delta < -_PI) delta += _2PI;
  index_target  = cur + delta;
  index_arrived = false;
  index_mode    = true;
  stop_then_disarm = false;
  // The assembly needs ~4.5 A to break stiction; the soft spin velocity gains barely
  // command current at index speeds. Run the move on the CRAWL profile (disarm()
  // restores SPIN) so the position servo actually has low-speed torque.
  applyPidProfile(CRAWL_PROFILE);
  motor.controller = MotionControlType::angle;      // position servo
  motor.velocity_limit = INDEX_VEL_LIMIT;           // slow, controlled index
  Serial.print("OK INDEX "); Serial.println(tube);
}

static void linkEstop() {
  link_mode = true; last_link_ms = millis(); wd_tripped = false;
  if (calibrationRunning() || cal_state == CAL_DONE) {
    abortCalibration("estop", CAL_FAILED);
  }
  stop_then_disarm = false;
  disarm("ESTOP");                     // immediate torque cut
  Serial.println("OK ESTOP");
}

static void linkPing() {
  link_mode = true; last_link_ms = millis(); wd_tripped = false;
  Serial.println("OK PONG");
}

static bool argEquals(const char *arg, const char *want) {
  while (*arg == ' ') arg++;
  while (*want != '\0') {
    char a = *arg++;
    if (a >= 'a' && a <= 'z') a = (char)(a - ('a' - 'A'));
    if (a != *want++) return false;
  }
  return *arg == '\0' || *arg == ' ' || *arg == '?';
}

static void resetCalMetrics() {
  cal_max_abs_rpm_err = 0.0f;
  cal_sum_abs_rpm_err = 0.0f;
  cal_max_current = 0.0f;
  cal_samples = 0;
}

static void setCalState(CalibrationState state) {
  cal_state = state;
  cal_state_ms = millis();
  resetCalMetrics();
  if (state == CAL_ENCODER_CHECK) {
    cal_encoder_start = (uint16_t)TIM4->CNT;
  }
  if (state == CAL_LOW_SPEED_PID && cal_step < CAL_RPM_STEP_COUNT) {
    cal_target_rpm = CAL_RPM_STEPS[cal_step];
    cmd_vel = (cal_target_rpm / 60.0f) * REV;
  }
  if (state == CAL_OBSERVER_SHADOW) {
    cal_target_rpm = CROSSOVER_RPM;
    cmd_vel = (cal_target_rpm / 60.0f) * REV;
  }
  if (state == CAL_DONE) {
    cal_target_rpm = 0.0f;
    cmd_vel = 0.0f;
  }
}

static void calFail(const char *reason) {
  strncpy(cal_error, reason, sizeof(cal_error) - 1U);
  cal_error[sizeof(cal_error) - 1U] = '\0';
  ramp_accel = cal_saved_ramp_accel;
  disarm("cal failed");
  cal_state = CAL_FAILED;
  cal_state_ms = millis();
  Serial.print("ERR CAL "); Serial.println(cal_error);
}

static void finishCalibration() {
  strncpy(cal_error, "none", sizeof(cal_error) - 1U);
  cal_error[sizeof(cal_error) - 1U] = '\0';
  ramp_accel = cal_saved_ramp_accel;
  setCalState(CAL_DONE);
  Serial.println("OK CAL DONE");
}

static void printCalStatus() {
  float rpm = (encoder.getVelocity() / REV) * 60.0f;
  Serial.print("CAL state=");      Serial.print(calStateName());
  Serial.print(" step=");          Serial.print(cal_step);
  Serial.print(" rpm=");           Serial.print(rpm, 0);
  Serial.print(" target=");        Serial.print(cal_target_rpm, 0);
  Serial.print(" cur=");           Serial.print(g_last_current_a, 2);
  Serial.print(" loop_hz=");       Serial.print(g_loop_hz);
  Serial.print(" zero=");          Serial.print(motor.zero_electric_angle, 4);
  Serial.print(" dir=");           Serial.print((int)motor.sensor_direction);
  Serial.print(" obs_rpm=");       Serial.print(g_observer_rpm, 0);
  Serial.print(" obs_lock=");      Serial.print(observer.locked() ? 1 : 0);
  Serial.print(" phase_err=");     Serial.print(g_observer_phase_err * 180.0f / _PI, 1);
  Serial.print(" max_err=");       Serial.print(cal_max_abs_rpm_err, 0);
  Serial.print(" max_cur=");       Serial.print(cal_max_current, 2);
  Serial.print(" profile=");       Serial.print(active_pid_profile->name);
  Serial.print(" err=");           Serial.print(cal_error);
  Serial.println(" save=0");
}

static void startCalibration() {
  link_mode = true; last_link_ms = millis(); wd_tripped = false;
  if (calibrationMotionActive()) {
    Serial.println("ERR CAL busy");
    return;
  }
  cal_saved_ramp_accel = ramp_accel;
  ramp_accel = 20.0f * REV;
  applyPidProfile(SPIN_PROFILE);
  if (stage != CLOSEDLOOP) enterStage(CLOSEDLOOP);
  cal_step = 0;
  cal_target_rpm = 0.0f;
  strncpy(cal_error, "none", sizeof(cal_error) - 1U);
  cal_error[sizeof(cal_error) - 1U] = '\0';
  observer.resetFromElectrical(0.0f, 0.0f);
  setCalState(CAL_ALIGN);
  Serial.println("OK CAL START");
}

static void stopCalibration() {
  link_mode = true; last_link_ms = millis(); wd_tripped = false;
  abortCalibration("none", CAL_IDLE);
  disarm("cal stop");
  Serial.println("OK CAL STOP");
}

static void handleCalCommand(const char *arg) {
  if (argEquals(arg, "START"))  { startCalibration(); return; }
  if (argEquals(arg, "STOP"))   { stopCalibration();  return; }
  if (argEquals(arg, "STATUS")) { link_mode = true; last_link_ms = millis(); wd_tripped = false; printCalStatus(); return; }
  if (argEquals(arg, "APPLY"))  { applyPidProfile(SPIN_PROFILE); Serial.println("OK CAL APPLY profile=spin save=0"); return; }
  Serial.println("ERR CAL verb");
}

static void handleProfileCommand(const char *arg) {
  link_mode = true; last_link_ms = millis(); wd_tripped = false;
  if (argEquals(arg, "SPIN")) {
    applyPidProfile(SPIN_PROFILE);
    Serial.println("OK PROFILE SPIN");
    return;
  }
  if (argEquals(arg, "CRAWL")) {
    applyPidProfile(CRAWL_PROFILE);
    Serial.println("OK PROFILE CRAWL");
    return;
  }
  Serial.println("ERR PROFILE verb");
}

static void serviceCalibration() {
  uint32_t now = millis();
  if (cal_state == CAL_IDLE || cal_state == CAL_FAILED) {
    return;
  }

  if (cal_state == CAL_ALIGN) {
    // Give the master time to RETRACT the gantry lock before alignment torques the
    // shaft (it releases the lock as soon as telemetry shows cal active). Aligning
    // against the engaged taper pin grinds it and corrupts zero_electric_angle.
    if ((uint32_t)(now - cal_state_ms) < CAL_LOCK_RELEASE_MS) {
      return;
    }
    if (!armed) {
      arm();
      if (!armed) {
        if ((uint32_t)(now - cal_state_ms) > CAL_ALIGN_TIMEOUT_MS) {
          calFail("align-timeout");
        }
        return;
      }
      ramp_vel = 0.0f;
      cmd_vel = 0.0f;
      setCalState(CAL_ENCODER_CHECK);
    }
  }

  if (!armed) {
    return;
  }

  motor.loopFOC();
  updateRamp();
  motor.move(ramp_vel);

  PhaseCurrent_s c = currentSense.getPhaseCurrents();
  (void)c;
  g_last_current_a = fabsf(currentSense.getDCCurrent());
  float rpm = (encoder.getVelocity() / REV) * 60.0f;
  float abs_err = fabsf(rpm - cal_target_rpm);
  if (g_last_current_a > cal_max_current) cal_max_current = g_last_current_a;
  if (abs_err > cal_max_abs_rpm_err) cal_max_abs_rpm_err = abs_err;
  cal_sum_abs_rpm_err += abs_err;
  cal_samples++;

  uint32_t elapsed = now - cal_state_ms;
  switch (cal_state) {
    case CAL_ENCODER_CHECK: {
      cmd_vel = 0.0f;
      if (elapsed >= CAL_ENCODER_CHECK_MS) {
        int32_t diff = (int32_t)((uint16_t)TIM4->CNT) - (int32_t)cal_encoder_start;
        if (diff < 0) diff = -diff;
        // TIM4 wraps at ENCODER_CPR (ARR = CPR-1): a rotor parked near the wrap point
        // jitters between ~0 and ~CPR-1, so compare the CIRCULAR distance.
        if (diff > (int32_t)(ENCODER_CPR / 2u)) diff = (int32_t)ENCODER_CPR - diff;
        if (diff > CAL_ENCODER_STILL_COUNTS) {
          calFail("encoder-moving");
        } else if ((int)motor.sensor_direction == 0) {
          calFail("encoder-dir");
        } else {
          setCalState(CAL_CURRENT_CHECK);
        }
      }
      break;
    }
    case CAL_CURRENT_CHECK:
      cmd_vel = 0.0f;
      if (elapsed >= CAL_CURRENT_CHECK_MS) {
        if (cal_max_current > motor.current_limit + 0.75f) {
          calFail("current-high");
        } else {
          cal_step = 0;
          setCalState(CAL_LOW_SPEED_PID);
        }
      }
      break;
    case CAL_LOW_SPEED_PID:
      if (elapsed > (CAL_PID_STEP_MS / 2U) &&
          cal_samples > 5U &&
          cal_max_abs_rpm_err > CAL_RPM_ERROR_LIMIT) {
        calFail("pid-track");
        break;
      }
      if (elapsed >= CAL_PID_STEP_MS) {
        cal_step++;
        if (cal_step < CAL_RPM_STEP_COUNT) {
          setCalState(CAL_LOW_SPEED_PID);
        } else {
#if HIGH_SPEED_SENSORLESS
          setCalState(CAL_OBSERVER_SHADOW);
#else
          finishCalibration();
#endif
        }
      }
      break;
    case CAL_OBSERVER_SHADOW: {
      float encoder_elec = motor.electricalAngle();
      uint32_t nowus = micros();
      float dt = (nowus - sensorless_last_us) * 1e-6f;
      sensorless_last_us = nowus;
      // Same rule as the run-time shadow: the observer gets the voltages FOC actually
      // applied (motor.voltage), not the open-loop schedule.
      observer.update(c.a, c.b, c.c, motor.voltage.q, motor.voltage.d, encoder_elec,
                      encoder_elec, rpm, dt);
      g_observer_rpm = observer.rpmMechanical();
      g_observer_phase_err = observer.phaseErrorToEncoder();
      g_observer_bemf = observer.bemfVolts();
      spin_mode = OBSERVER_SHADOW;
      if (elapsed >= CAL_OBSERVER_SHADOW_MS) {
        finishCalibration();
      }
      break;
    }
    case CAL_DONE:
      cmd_vel = 0.0f;
      if (fabsf(ramp_vel) < 0.1f) {
        disarm("cal done");
      }
      break;
    default:
      break;
  }
}

// Returns true if the line was a high-level verb (and was handled).
static bool handleHighLevel(const char *line) {
  char tok[12];
  size_t i = 0;
  while (line[i] != '\0' && line[i] != ' ' && i < sizeof(tok) - 1) {
    char ch = line[i];
    if (ch >= 'a' && ch <= 'z') ch = (char)(ch - ('a' - 'A'));
    tok[i++] = ch;
  }
  tok[i] = '\0';
  const char *arg = line + i;
  while (*arg == ' ') arg++;

  if (strcmp(tok, "SPIN") == 0)    { linkSpin(atof(arg)); return true; }
  if (strcmp(tok, "INDEX") == 0)   { linkIndex(atoi(arg)); return true; }
  if (strcmp(tok, "HOME") == 0)    { linkHome();          return true; }
  if (strcmp(tok, "STOP") == 0)    { linkStop();          return true; }
  if (strcmp(tok, "ESTOP") == 0)   { linkEstop();         return true; }
  if (strcmp(tok, "PING") == 0)    { linkPing();          return true; }
  if (strcmp(tok, "CAL") == 0)     { handleCalCommand(arg); return true; }
  if (strcmp(tok, "PROFILE") == 0) { handleProfileCommand(arg); return true; }
  if (strcmp(tok, "TUNE?") == 0 ||
      strcmp(tok, "TUNE") == 0)    { link_mode = true; last_link_ms = millis();
                                     wd_tripped = false; printTune(); return true; }
  if (strcmp(tok, "STATUS?") == 0 ||
      strcmp(tok, "STATUS") == 0)  { link_mode = true; last_link_ms = millis();
                                     wd_tripped = false; printST(); return true; }
  return false;
}

static void handleLine(char *line) {
  // High-level Due verbs (whole-word) take priority; everything else falls
  // through to the single-char bench commands below.
  if (handleHighLevel(line)) return;

  // first token is the command, rest is an optional float arg
  char c = line[0];
  float arg = atof(line + 1);
  switch (c) {
    case '0': enterStage(IDLE);       break;
    case '1': enterStage(ENCODER);    break;
    case '2': enterStage(OPENLOOP);   break;
    case '3': enterStage(CURRENT);    break;
    case '4': enterStage(CLOSEDLOOP); break;
    case 'g': case 'G': arm();        break;   // (g)o / arm
    case 'x': case 'X': disarm("operator stop"); break;
    case 'v': case 'V':
      cmd_vel = arg * REV;
      Serial.print("OK target "); Serial.print(arg, 3); Serial.println(" rev/s");
      break;
    case 'a': case 'A':
      if (arg > 0) ramp_accel = arg * REV;
      Serial.print("OK accel "); Serial.print(arg, 3); Serial.println(" rev/s^2");
      break;
    case 'k': case 'K':                       // set FOC alignment voltage (takes effect next arm)
      if (arg > 0) motor.voltage_sensor_align = arg;
      Serial.print("OK align voltage "); Serial.print(motor.voltage_sensor_align, 3);
      Serial.println(" V (re-arm in stage 4 to apply)");
      break;
    case 'l': case 'L':                       // set live voltage limit (current ceiling)
      if (arg > 0) motor.voltage_limit = _constrain(arg, 0.0f, VBUS_HALF);  // never past VBUS/2
      Serial.print("OK voltage_limit "); Serial.print(motor.voltage_limit, 3); Serial.println(" V");
      break;
    case 'p': case 'P':                       // live velocity PID P
      if (arg >= 0) motor.PID_velocity.P = arg;
      Serial.print("OK PID.P "); Serial.println(motor.PID_velocity.P, 4);
      break;
    case 'i': case 'I':                       // live velocity PID I
      if (arg >= 0) motor.PID_velocity.I = arg;
      Serial.print("OK PID.I "); Serial.println(motor.PID_velocity.I, 4);
      break;
    case 'f': case 'F':                       // live velocity LPF time constant
      if (arg >= 0) motor.LPF_velocity.Tf = arg;
      Serial.print("OK LPF.Tf "); Serial.println(motor.LPF_velocity.Tf, 4);
      break;
    case 'c': case 'C':                       // live current limit (foc_current)
      if (arg > 0) motor.current_limit = arg;
      Serial.print("OK current_limit "); Serial.print(motor.current_limit, 3); Serial.println(" A");
      break;
    case 'q': case 'Q':                       // live current-loop P (both q and d)
      if (arg >= 0) { motor.PID_current_q.P = arg; motor.PID_current_d.P = arg; }
      Serial.print("OK current PID.P "); Serial.println(motor.PID_current_q.P, 4);
      break;
    case 'j': case 'J':                       // live current-loop I (both q and d)
      if (arg >= 0) { motor.PID_current_q.I = arg; motor.PID_current_d.I = arg; }
      Serial.print("OK current PID.I "); Serial.println(motor.PID_current_q.I, 4);
      break;
    case 's': case 'S':
      cmd_vel = 0.0f;
      Serial.println("OK stop (ramping to 0)");
      break;
    case 'z': case 'Z':
      // zero the cumulative revolution counter (encoder stage convenience)
      encoder.update();
      Serial.println("OK angle counter noted (turn shaft and compare rev delta)");
      break;
    case '?': printStat(); break;
    case 'h': case 'H':
      Serial.println("cmds: 0/1/2/3/4=stage  g=arm  x=stop  v<n>=vel  a<n>=accel  s=ramp0  ?=stat");
      Serial.println("link: SPIN <rpm>  INDEX <0-3>  STOP  ESTOP  PING  STATUS?  (Due-facing verbs)");
      break;
    default:
      Serial.print("ERR unknown '"); Serial.print(c); Serial.println("' (h=help)");
      break;
  }
}

static void pollSerial() {
  static char buf[32];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (n > 0) { buf[n] = '\0'; handleLine(buf); n = 0; }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  SimpleFOCDebug::enable(&Serial);        // print SimpleFOC's own align/init diagnostics

  // --- encoder (ABI quadrature, hardware TIM4 programmed directly) ---
  encoder.init();                        // configures + starts TIM4 in encoder mode
  motor.linkSensor(&encoder);
  // Verify in stage 1: spin by hand -> `cnt` (raw TIM4) and `rev` must move.

  // --- driver ---
  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.voltage_limit = VBUS_HALF;      // never modulate past VBUS/2 (lowside CS)
  bool ok_drv = driver.init();          // needs lib_archive=no, or STM32 6PWM falls
  motor.linkDriver(&driver);            // back to the weak generic stub and fails

  // --- current sense ---
  currentSense.linkDriver(&driver);
  bool ok_cs = currentSense.init();
  // Skip the powered current-sense driver-align inside initFOC(): it would energize
  // the phases and could flip our known-good -64/7 sign/order. We bring up in VOLTAGE
  // torque mode (current sense is read for telemetry only, not the control loop), and
  // stage 3 validates the currents directly. Matches the official B-G431B example.
  currentSense.skip_align = true;
  motor.linkCurrentSense(&currentSense);

  // --- motor config ---
  motor.voltage_sensor_align = ALIGN_VOLTAGE;   // gentle alignment for low-R motor
  motor.voltage_limit  = MOTOR_VOLT_LIMIT;
  motor.current_limit  = BRINGUP_CURRENT;
  motor.velocity_limit = 200.0f * REV;          // hard ceiling, rad/s
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // Torque mode is set per stage in enterStage(): voltage for open-loop (2/3),
  // foc_current for closed-loop (4) so current is bounded at any speed.
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::velocity;

  // Outer velocity loop. RETUNED SOFTER (2026-07-02): the original 0.1/0.3 was
  // marginal and, after the loaded assembly shifted slightly, oscillated/reversed
  // at speed (smooth open-loop, unstable closed-loop). Softer P/I + more velocity
  // filtering restored the stability margin -- rock-solid 0..4000 RPM. Live: `p/i/f`.
  applyPidProfile(SPIN_PROFILE);

  // Position loop (used only by the INDEX angle-control move to a tube). Gentle P so the
  // gantry indexes smoothly into the detent; the inner velocity/current loops still bound it.
  motor.P_angle.P = INDEX_P_ANGLE;

  // Inner current loop (foc_current). I dropped 100 -> 20 as part of the same retune
  // (the aggressive current integral was feeding the oscillation). Tune P `q`, I `j`.
  motor.LPF_current_q.Tf = 0.005f;
  motor.LPF_current_d.Tf = 0.005f;

  SensorlessObserverConfig obsCfg;
  obsCfg.polePairs = POLE_PAIRS;
  obsCfg.phaseResistanceOhm = MOTOR_PHASE_RESISTANCE_OHM;
  obsCfg.phaseInductanceH = MOTOR_PHASE_INDUCTANCE_H;
  obsCfg.kvRpmPerVolt = MOTOR_KV;
  obsCfg.pllKp = 250.0f;
  obsCfg.pllKi = 4000.0f;
  obsCfg.minBemfVolts = SENSORLESS_MIN_BEMF_V;
  obsCfg.lockPhaseErrorRad = SENSORLESS_LOCK_ERR_RAD;
  obsCfg.lockRpmError = SENSORLESS_LOCK_RPM_ERR;
  obsCfg.lockDwellMs = SENSORLESS_LOCK_DWELL_MS;
  obsCfg.maxRpm = MAX_SPIN_RPM * 1.15f;   // headroom so the PLL can MEASURE an overshoot
                                          // past the commandable max instead of saturating
  observer.begin(obsCfg);

  bool ok_motor = motor.init();
  Serial.print("INIT driver="); Serial.print(ok_drv);
  Serial.print(" cs=");          Serial.print(ok_cs);
  Serial.print(" motor=");       Serial.println(ok_motor);
  if (!ok_drv || !ok_cs || !ok_motor)
    Serial.println("WARN one or more inits FAILED -- do NOT energize; check build/wiring");
  motor.disable();                       // no torque until `arm`
  // NOTE: initFOC() is deferred to arm() in stage 4 -- needs bus power present.

  last_ramp_us = micros();
  sensorless_last_us = last_ramp_us;
  Serial.println("READY. Encoder live, motor OFF.");
  Serial.println("Start at stage 1: send `1`, turn the shaft, confirm rev tracks. (h=help)");
}

void loop() {
  static uint32_t loop_count = 0;
  loop_count++;

  if ((calibrationRunning() || (cal_state == CAL_DONE && armed)) && stage == CLOSEDLOOP) {
    serviceCalibration();
  } else if (armed) {
    // SimpleFOC 2.4.0: loopFOC() drives the phases in closed loop; move() computes
    // the setpoint and (in open loop) advances the commutation angle. Keep this path
    // TIGHT: at high speed commutation must refresh far faster than the electrical Hz.
    if (index_mode) {
      // Closed-loop angle move/hold to the tube position (gantry unlocked by the Due).
      motor.loopFOC();
      motor.move(index_target);
      if (!index_arrived && fabsf(encoder.getAngle() - index_target) < INDEX_TOL_RAD) {
        index_arrived = true;
        Serial.print("OK INDEX DONE "); Serial.println(index_tube);
      }
    } else if (stage == CLOSEDLOOP) {
      // High-speed builds keep encoder FOC below the crossover and run the
      // observer in shadow before handing it authority over the electrical angle.
#if HIGH_SPEED_SENSORLESS
      if (!sensorlessModeActive()) {
        motor.loopFOC();                            // closed-loop current control (updates sensor)
        updateRamp();
        motor.move(ramp_vel);

        PhaseCurrent_s c = currentSense.getPhaseCurrents();
        g_last_current_a = fabsf(currentSense.getDCCurrent());
        float ramp_rpm = (ramp_vel / REV) * 60.0f;
        float encoder_rpm = (encoder.getVelocity() / REV) * 60.0f;
        float encoder_elec = motor.electricalAngle();
        uint32_t nowus = micros();
        float dt = (nowus - sensorless_last_us) * 1e-6f;
        sensorless_last_us = nowus;

        if (ramp_rpm >= SENSORLESS_SHADOW_RPM) {
          spin_mode = OBSERVER_SHADOW;
          // Feed the observer the voltages FOC actually applied this cycle (motor.voltage
          // is set by loopFOC()), not a hypothetical schedule -- shadow lock must prove
          // the observer tracks under the real drive, or the handoff gate means nothing.
          observer.update(c.a, c.b, c.c, motor.voltage.q, motor.voltage.d, encoder_elec,
                          encoder_elec, encoder_rpm, dt);
        } else {
          spin_mode = ENCODER_FOC;
        }

        g_observer_rpm = observer.rpmMechanical();
        g_observer_phase_err = observer.phaseErrorToEncoder();
        g_observer_bemf = observer.bemfVolts();

        if (!calibrationRunning() &&
            ramp_rpm > CROSSOVER_RPM + CROSSOVER_HYST_RPM && observer.locked()) {
          enterSensorlessMode(encoder_elec, encoder_rpm);
        }
      } else {
        encoder.update();                           // keep encoder fresh for telemetry/fallback
        updateRamp();

        uint32_t nowus = micros();
        float dt = (nowus - sensorless_last_us) * 1e-6f;
        sensorless_last_us = nowus;
        float ramp_rpm = (ramp_vel / REV) * 60.0f;
        float encoder_rpm = (encoder.getVelocity() / REV) * 60.0f;
        float encoder_elec = motor.electricalAngle();
        float uq = (spin_mode == FAULT_RAMPDOWN) ? 0.0f : olVoltage(fabsf(ramp_rpm));

        PhaseCurrent_s c = currentSense.getPhaseCurrents();
        g_last_current_a = fabsf(currentSense.getDCCurrent());
        observer.update(c.a, c.b, c.c, uq, 0.0f, observer.angleElectrical(),
                        encoder_elec, encoder_rpm, dt);

        g_observer_rpm = observer.rpmMechanical();
        g_observer_phase_err = observer.phaseErrorToEncoder();
        g_observer_bemf = observer.bemfVolts();

        // NORMAL descent: once the commanded ramp is back below the crossover, hand
        // cleanly back to encoder FOC (the encoder was kept fresh throughout) -- a
        // routine STOP must not ride the observer to a stall and report as a fault.
        if (spin_mode == SENSORLESS_PLL && ramp_rpm < (CROSSOVER_RPM - CROSSOVER_HYST_RPM)) {
          enterClosedLoopMode();
        }

        bool grace_done = (uint32_t)(millis() - sensorless_enter_ms) > 150U;
        if (spin_mode == SENSORLESS_PLL && grace_done && !observer.locked()) {
          enterFaultRampdown("observer unlock");
          uq = 0.0f;
        }
        if (spin_mode == SENSORLESS_PLL && g_last_current_a > SENSORLESS_TRIAL_CURRENT_A) {
          enterFaultRampdown("trial current limit");
          uq = 0.0f;
        }
        if (spin_mode == FAULT_RAMPDOWN && fabsf(encoder_rpm) < (CROSSOVER_RPM - CROSSOVER_HYST_RPM)) {
          enterClosedLoopMode();
        }

        if (sensorlessModeActive()) {
          motor.setPhaseVoltage(uq, 0.0f, observer.angleElectrical());
        }
      }
#else
      spin_mode = ENCODER_FOC;
      motor.loopFOC();                              // production default: encoder FOC only
      updateRamp();
      motor.move(ramp_vel);
      g_last_current_a = fabsf(currentSense.getDCCurrent());
#endif
    } else {
      motor.loopFOC();                              // bench stages 2/3 (open-loop / current)
      updateRamp();
      motor.move(ramp_vel);
    }
  } else {
    encoder.update();                    // unarmed: keep angle fresh for stage 0/1
  }

  pollSerial();

  uint32_t now = millis();

  // ----- high-level link maintenance: comms watchdog + STOP completion -------
  if (link_mode) {
    // Comms watchdog: if the Due goes quiet, fail-safe ramp to 0 and disarm so a
    // dead/disconnected master can't leave the spindle spinning. Fires once per
    // outage (re-armed by the next verb/PING).
    if (!wd_tripped && (now - last_link_ms) > LINK_WD_MS) {
      wd_tripped = true;
      Serial.println("WD link timeout -> fail-safe ramp down");
      cmd_vel = 0.0f;
      stop_then_disarm = true;
    }
    // Complete a STOP / watchdog ramp-down: cut torque once the ramp hits 0.
    if (stop_then_disarm) {
      if (!armed)                 stop_then_disarm = false;     // already de-energized
      else if (index_mode)      { disarm("wd"); stop_then_disarm = false; }  // fail-safe: drop the hold
      else if (ramp_vel == 0.0f) { disarm("ramp complete"); stop_then_disarm = false; }
    }
  }

  // ----- telemetry -----------------------------------------------------------
  // In link mode the Due wants the compact ST stream at ~10 Hz; on the bench keep
  // the verbose STAT line (50 Hz running / 10 Hz hand-turn in the encoder stage).
  if (link_mode) {
    if (now - last_st_ms >= 100) {                  // ~10 Hz
      g_loop_hz = loop_count * 1000U / (now - last_st_ms);
      loop_count = 0;
      last_st_ms = now;
      printST();
    }
  } else {
    uint16_t period = (stage == ENCODER) ? 100 : 20;   // 10 Hz hand-turn / 50 Hz run
    if (now - last_tele_ms >= period) {
      g_loop_hz = loop_count * 1000U / (now - last_tele_ms);   // actual loop/FOC rate
      loop_count = 0;
      last_tele_ms = now;
      printStat();
    }
  }
}

#endif  // !ENCODER_DIAG
