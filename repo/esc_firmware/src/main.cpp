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
// Quadrature is decoded by TIM4 in silicon -> zero CPU load at any RPM (an
// interrupt-based Encoder would saturate the core past ~4-5k RPM at 4000 CPR).
// We program TIM4 directly (below) instead of the SimpleFOCDrivers STM32HWEncoder,
// whose init() silently failed on this board/core combo.

// ----------------------------- tunables ------------------------------------
#define POLE_PAIRS         7        // GT2215/10: 14 magnets / 2
#define SUPPLY_VOLTAGE     12.0f    // bench PSU (3S equivalent)

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
#define BRINGUP_CURRENT    4.5f     // closed-loop CURRENT limit (foc_current) -- enough to
                                    // break the loaded shaft loose; the real safety cap. Live: `c`.
#define MOTOR_VOLT_LIMIT   5.0f     // closed-loop voltage ceiling: high enough for 4000 RPM
                                    // back-EMF (~3.6 V). Safe because foc_current bounds the
                                    // current; voltage just follows what BEMF needs. (<= VBUS/2)

// ===== Dual-mode speed profile: closed-loop <-> open-loop ===================
// Closed-loop FOC (encoder + current sense) runs 0..CROSSOVER_RPM. Above that,
// the encoder-in-loop / low-side current sense run out of headroom, so we hand
// off to OPEN-LOOP: commutate at the commanded speed with a scheduled voltage,
// NO feedback, up to MAX_SPIN_RPM. On the way down we hand BACK to closed-loop at
// the crossover for a controlled decel. The Nano just commands a target RPM
// (SPIN <rpm>); the ESC does all the mode switching internally.
//
// !!! REACHABLE TOP SPEED IS BOUNDED BY THE SUPPLY !!! back-EMF ~ rpm/KV, and the
// phases can only be driven to ~VBUS/2. At 12 V that caps ~6k RPM; 10k needs a 6S
// (~24 V) pack -> set SUPPLY_VOLTAGE=24 and reflash. Everything below is relative
// to SUPPLY_VOLTAGE so it scales automatically.
//
// !!! UNTUNED ABOVE THE CROSSOVER !!! the open-loop regime + the closed<->open
// handoff have NOT been validated on hardware (the 12 V bench can't reach it).
// Validate incrementally on the final supply, with containment, before real use.
#define MOTOR_KV           1100.0f  // GT2215/10: 1100 RPM per volt of back-EMF
#define CROSSOVER_RPM      4000.0f  // closed-loop below, open-loop above
#define CROSSOVER_HYST_RPM  150.0f  // hysteresis so it doesn't chatter at the boundary
#define MAX_SPIN_RPM      10000.0f  // top commandable target (supply-bounded in reality)
#define OL_BEMF_MARGIN      0.8f    // open-loop drives back-EMF + this margin (keeps current low)
#define OL_VOLT_CAP        VBUS_HALF // open-loop voltage ceiling (== driver limit; raise supply for more)

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
static Stage stage = IDLE;
static bool  armed = false;          // motor energized? (stages 2-4 require arm)
static bool  foc_ready = false;      // initFOC() (alignment) done once
static bool  ol_mode = false;        // dual-mode: true = open-loop (ramp_vel > crossover)
static float ol_elec_angle = 0.0f;   // manually-integrated electrical angle (open-loop mode)
static uint32_t ol_last_us = 0;      // last open-loop commutation timestamp

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
static void disarm(const char *why) {
  motor.disable();
  armed = false;
  cmd_vel = ramp_vel = 0.0f;
  // Always leave the motor in closed-loop VELOCITY config so the next arm starts there
  // (a disarm from the open-loop or indexing regime must not strand those settings).
  ol_mode = false;
  index_mode = false;
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

// Hand off closed-loop -> open-loop. We drive the commutation MANUALLY (via
// setPhaseVoltage) rather than SimpleFOC's velocity_openloop, so we can seed the
// electrical angle from the rotor's actual calibrated position -- commutation then
// continues from where the rotor IS, staying synchronized (both are at the crossover
// speed). UNTUNED: validate the handoff at the real crossover speed + supply.
static void enterOpenLoopMode() {
  ol_mode = true;
  ol_elec_angle = motor.electricalAngle();   // rotor's real (calibrated) electrical position
  ol_last_us = micros();
  Serial.println("MODE open-loop (> crossover)");
}

// Hand off open-loop -> closed-loop. The encoder was kept fresh during open-loop,
// so FOC re-locks using the calibrated zero_electric_angle. UNTUNED.
static void enterClosedLoopMode() {
  ol_mode = false;
  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.voltage_limit = MOTOR_VOLT_LIMIT;
  Serial.println("MODE closed-loop (< crossover)");
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
  // Current sense is only valid in closed loop (open-loop runs past the lowside-CS
  // duty limit), so report it only there.
  float cur = (armed && stage == CLOSEDLOOP && !ol_mode && !index_mode) ? currentSense.getDCCurrent() : 0.0f;
  const char *st = !armed ? "idle"
                          : index_mode ? (index_arrived ? "indexed" : "indexing")
                          : (stop_then_disarm ? "stopping" : (ol_mode ? "spin-ol" : "spin"));

  Serial.print("ST rpm=");   Serial.print(rpm, 0);
  Serial.print(" tgt=");     Serial.print(tgt_rpm, 0);
  Serial.print(" state=");   Serial.print(st);
  Serial.print(" cur=");     Serial.print(cur, 2);
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
  if (index_mode) {
    disarm("index stop");              // leaves angle-hold; lock now holds the gantry
    Serial.println("OK STOP");
    return;
  }
  cmd_vel = 0.0f;
  stop_then_disarm = true;             // disarm once the ramp reaches 0
  Serial.println("OK STOP");
}

// Closed-loop angle move to a tube position (0..TUBE_COUNT-1). The gantry MUST be
// unlocked before this (the Due sequences that). Takes the shortest path; on arrival it
// holds the angle until the Due locks the gantry and sends STOP to disarm.
static void linkIndex(int tube) {
  link_mode = true; last_link_ms = millis(); wd_tripped = false;
  tube = ((tube % TUBE_COUNT) + TUBE_COUNT) % TUBE_COUNT;
  index_tube = tube;
  if (stage != CLOSEDLOOP) enterStage(CLOSEDLOOP);
  if (!armed) arm();                   // aligns on the first arm (needs the 12 V bus)
  if (!armed) { Serial.println("ERR INDEX not armed (bus power? align failed)"); return; }

  // Shortest move to the tube's ABSOLUTE mechanical angle (tube 0 = power-on position).
  float target_mech = (float)tube * TUBE_STEP_RAD;
  float cur = encoder.getAngle();                  // cumulative rad
  float delta = target_mech - _normalizeAngle(cur);
  while (delta >  _PI) delta -= _2PI;              // shortest path, [-pi, pi]
  while (delta < -_PI) delta += _2PI;
  index_target  = cur + delta;
  index_arrived = false;
  index_mode    = true;
  stop_then_disarm = false;
  motor.controller = MotionControlType::angle;      // position servo
  motor.velocity_limit = INDEX_VEL_LIMIT;           // slow, controlled index
  Serial.print("OK INDEX "); Serial.println(tube);
}

static void linkEstop() {
  link_mode = true; last_link_ms = millis(); wd_tripped = false;
  stop_then_disarm = false;
  disarm("ESTOP");                     // immediate torque cut
  Serial.println("OK ESTOP");
}

static void linkPing() {
  link_mode = true; last_link_ms = millis(); wd_tripped = false;
  Serial.println("OK PONG");
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
  if (strcmp(tok, "STOP") == 0)    { linkStop();          return true; }
  if (strcmp(tok, "ESTOP") == 0)   { linkEstop();         return true; }
  if (strcmp(tok, "PING") == 0)    { linkPing();          return true; }
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
  motor.PID_velocity.P = 0.05f;
  motor.PID_velocity.I = 0.10f;
  motor.PID_velocity.output_ramp = 200.0f;
  motor.LPF_velocity.Tf = 0.15f;

  // Position loop (used only by the INDEX angle-control move to a tube). Gentle P so the
  // gantry indexes smoothly into the detent; the inner velocity/current loops still bound it.
  motor.P_angle.P = INDEX_P_ANGLE;

  // Inner current loop (foc_current). I dropped 100 -> 20 as part of the same retune
  // (the aggressive current integral was feeding the oscillation). Tune P `q`, I `j`.
  motor.PID_current_q.P = 0.3f;  motor.PID_current_q.I = 20.0f;  motor.PID_current_q.limit = MOTOR_VOLT_LIMIT;
  motor.PID_current_d.P = 0.3f;  motor.PID_current_d.I = 20.0f;  motor.PID_current_d.limit = MOTOR_VOLT_LIMIT;
  motor.LPF_current_q.Tf = 0.005f;
  motor.LPF_current_d.Tf = 0.005f;

  bool ok_motor = motor.init();
  Serial.print("INIT driver="); Serial.print(ok_drv);
  Serial.print(" cs=");          Serial.print(ok_cs);
  Serial.print(" motor=");       Serial.println(ok_motor);
  if (!ok_drv || !ok_cs || !ok_motor)
    Serial.println("WARN one or more inits FAILED -- do NOT energize; check build/wiring");
  motor.disable();                       // no torque until `arm`
  // NOTE: initFOC() is deferred to arm() in stage 4 -- needs bus power present.

  last_ramp_us = micros();
  Serial.println("READY. Encoder live, motor OFF.");
  Serial.println("Start at stage 1: send `1`, turn the shaft, confirm rev tracks. (h=help)");
}

void loop() {
  static uint32_t loop_count = 0;
  loop_count++;

  if (armed) {
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
      // Dual-mode: closed-loop FOC below the crossover, open-loop above it. The
      // switch is driven by the ramped setpoint, with hysteresis so it can't chatter.
      float ramp_rpm = (ramp_vel / REV) * 60.0f;
      if      (!ol_mode && ramp_rpm > CROSSOVER_RPM + CROSSOVER_HYST_RPM) enterOpenLoopMode();
      else if ( ol_mode && ramp_rpm < CROSSOVER_RPM - CROSSOVER_HYST_RPM) enterClosedLoopMode();

      if (ol_mode) {
        // Open-loop: advance the electrical angle at the commanded speed and apply a
        // scheduled voltage. No current/position feedback -- the rotor stays in sync
        // because it entered at the crossover speed. Keep the sensor fresh for the
        // handoff back and for telemetry.
        encoder.update();
        updateRamp();
        uint32_t nowus = micros();
        float dt = (nowus - ol_last_us) * 1e-6f;
        ol_last_us = nowus;
        if (dt > 0.0f && dt < 0.1f)
          ol_elec_angle = _normalizeAngle(ol_elec_angle + ramp_vel * (float)POLE_PAIRS * dt);
        motor.setPhaseVoltage(olVoltage((ramp_vel / REV) * 60.0f), 0.0f, ol_elec_angle);
      } else {
        motor.loopFOC();                            // closed-loop current control (updates sensor)
        updateRamp();
        motor.move(ramp_vel);
      }
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
