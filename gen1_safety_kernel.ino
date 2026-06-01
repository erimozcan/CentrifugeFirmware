#include "src/config/pins.h"
#include "src/config/constants.h"
#include "src/core/system_context.h"
#include "src/core/faults.h"
#include "src/core/hardware_guard.h"
#include "src/core/safety_supervisor.h"
#include "src/drivers/odrive_uart.h"
#include "src/sensors/sensor_manager.h"
#include "src/utils/timing.h"
#include "src/utils/cli.h"
#include "src/utils/mcu_watchdog_due.h"

static SystemContext g_ctx;
static HardwareGuard g_guard;
static ODriveUART g_odrive;
static SensorManager g_sensors;
static CLI g_cli;

static volatile bool g_tick_1khz = false;

static void setup_timer_1khz() {
  // TC0 channel 0 -> 1 kHz interrupt on Due
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC0);
  Tc* tc = TC0;
  uint32_t channel = 0;
  tc_init(tc, channel, TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC);
  // TIMER_CLOCK4 = MCK/128 = 84MHz/128 = 656250 Hz
  // RC = 656250 / 1000 = 656
  tc_write_rc(tc, channel, 656);
  tc_enable_interrupt(tc, channel, TC_IER_CPCS);
  NVIC_EnableIRQ(TC0_IRQn);
  tc_start(tc, channel);
}

void TC0_Handler() {
  TC0->TC_CHANNEL[0].TC_SR; // clear
  g_tick_1khz = true;
}

static void safety_cycle_1khz() {
  // Snapshot timestamp
  g_ctx.time_ms = millis();

  // Update sensors (non-blocking where possible)
  g_sensors.update_isr_side();
  g_sensors.publish_to_context(g_ctx);

  // Update ODrive comm health + DC bus (UART is non-blocking)
  g_odrive.poll();
  g_odrive.publish_to_context(g_ctx);

  // Evaluate supervisor (pure function)
  FaultLevel lvl = SafetySupervisor::evaluate(g_ctx);

  if (lvl == FaultLevel::HARD_STOP || lvl == FaultLevel::FAULT_LATCHED) {
    g_guard.hard_stop(g_ctx);
    g_ctx.fault_latched = true;
    g_ctx.fault_level = FaultLevel::FAULT_LATCHED;
  } else if (lvl == FaultLevel::SOFT_STOP) {
    // In this safety kernel v0, SOFT_STOP is treated as HARD_STOP
    g_guard.hard_stop(g_ctx);
    g_ctx.fault_latched = true;
    g_ctx.fault_level = FaultLevel::FAULT_LATCHED;
  } else {
    g_ctx.fault_level = lvl;

    // Only after a successful safety pass:
    // 1) keep MotorEnable in its commanded safe state (default disabled)
    g_guard.update_outputs(g_ctx);

    // 2) refresh ODrive watchdog (the real "external" fallback you accepted)
    g_odrive.refresh_watchdog();

    // 3) kick MCU WDT
    MCUWatchdogDue::kick();
  }

  g_ctx.safety_cycles++;
  g_ctx.last_safety_ok_ms = g_ctx.time_ms;
}

void setup() {
  SerialUSB.begin(115200);
  delay(250);

  g_ctx = SystemContext{};
  g_guard.begin();
  g_odrive.begin(ODRIVE_SERIAL_PORT, ODRIVE_BAUDRATE);
  g_sensors.begin();
  g_cli.begin();

  // Motor disabled by default
  g_guard.force_disable();

  // MCU watchdog: only helpful against firmware stalls (not a real external safety layer)
  MCUWatchdogDue::begin(MCU_WDT_TIMEOUT_MS);

  setup_timer_1khz();

  SerialUSB.println("GEN1 Safety Kernel v0 ready");
}

void loop() {
  // CLI runs in main loop; safety runs in timer IRQ-driven cadence
  g_cli.poll(g_ctx, g_guard, g_odrive);

  if (g_tick_1khz) {
    g_tick_1khz = false;
    safety_cycle_1khz();
  }
}
