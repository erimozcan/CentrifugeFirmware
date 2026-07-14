from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
CONFIG = REPO / "firmware" / "src" / "config.h"
STATE_H = REPO / "firmware" / "src" / "state_machine.h"
STATE_CPP = REPO / "firmware" / "src" / "state_machine.cpp"
HARDWARE = REPO / "firmware" / "src" / "hardware_guard.cpp"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_fan_control_constants_are_declared():
    src = read(CONFIG)
    for symbol in [
        "FAN_COOLDOWN_MS",
        "FAN_RPM_ACTIVE_THRESHOLD",
        "FAN_RUNS_DURING_ROTATE",
        "FAN_RUNS_DURING_ESC_CAL",
    ]:
        assert symbol in src


def test_state_machine_has_cooldown_state_and_timed_fan_update():
    header = read(STATE_H)
    source = read(STATE_CPP)

    assert "updateFanControl(SystemContext &ctx, uint32_t nowMs)" in header
    assert "fanDemandPreviously_" in header
    assert "fanCooldownUntilMs_" in header

    assert "isFanDemandActive" in source
    assert "fanCooldownUntilMs_ = nowMs + FAN_COOLDOWN_MS" in source
    assert "fanDemand || cooldownActive" in source
    assert "ctx.fanEnabled = ctx.devicePowered" not in source


def test_fan_demand_covers_run_rotate_calibration_and_rpm_fallback():
    src = read(STATE_CPP)

    for state in [
        "STATE_PRE_RUN_VERIFY",
        "STATE_LIFT_RAMP",
        "STATE_SEAT_HOLD",
        "STATE_MAIN_RAMP",
        "STATE_SPIN_HOLD",
        "STATE_STOPPING",
        "STATE_RUN_INDEX",
        "STATE_ROTATE_MOVING",
    ]:
        assert state in src

    assert "FAN_RUNS_DURING_ROTATE" in src
    assert "FAN_RUNS_DURING_ESC_CAL" in src
    assert "ctx.escCalState" in src
    assert "FAN_RPM_ACTIVE_THRESHOLD" in src
    assert "abs32(ctx.rpm1)" in src


def test_power_off_forces_fan_off_and_hardware_remains_digital():
    source = read(STATE_CPP)
    hardware = read(HARDWARE)

    assert "fanCooldownUntilMs_ = nowMs" in source
    assert "fanDemandPreviously_ = false" in source
    assert "ctx.fanEnabled = false" in source

    assert "void HardwareGuard::updateFan(bool enabled)" in hardware
    assert "digitalWrite(PIN_FAN_DRV_IN1, enabled ? HIGH : LOW)" in hardware
    assert "analogWrite(PIN_FAN_DRV_IN1" not in hardware
