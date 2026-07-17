from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
ESC_MAIN = REPO / "esc_firmware" / "src" / "main.cpp"
MASTER_COMMANDS = REPO / "firmware" / "src" / "command_interface.cpp"
MASTER_STATE = REPO / "firmware" / "src" / "state_machine.cpp"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_esc_spin_ceiling_is_supply_aware():
    src = read(ESC_MAIN)
    # The HS ceiling must derive from the supply (BEMF + margin <= VBUS/2), clamped to
    # the 10k design target -- NOT a bare 10000 the 12 V supply can never reach.
    assert "#define MAX_SPIN_RPM_DESIGN 10000.0f" in src
    assert "#define MAX_SPIN_RPM_SUPPLY (MOTOR_KV * (OL_VOLT_CAP - OL_BEMF_MARGIN))" in src
    assert "MAX_SPIN_RPM_SUPPLY < MAX_SPIN_RPM_DESIGN" in src
    # Supply voltage is overridable per-build for the 24 V bus without editing source.
    assert "#ifndef SUPPLY_VOLTAGE" in src


def test_esc_observer_shadow_uses_actual_foc_voltages():
    src = read(ESC_MAIN)
    # Shadow validation must feed the observer what FOC actually applied, or the lock
    # gate proves nothing. Both the run-time shadow and the calibration shadow.
    assert src.count("observer.update(c.a, c.b, c.c, motor.voltage.q, motor.voltage.d") == 2


def test_esc_sensorless_normal_descent_returns_to_encoder_foc():
    src = read(ESC_MAIN)
    # A commanded ramp below the crossover must hand back to encoder FOC (a routine
    # STOP is not a fault path).
    descent = src[src.index("NORMAL descent"):src.index("grace_done")]
    assert "spin_mode == SENSORLESS_PLL" in descent
    assert "enterClosedLoopMode()" in descent


def test_esc_cal_align_waits_for_gantry_lock_release():
    src = read(ESC_MAIN)
    assert "#define CAL_LOCK_RELEASE_MS" in src
    align = src[src.index("if (cal_state == CAL_ALIGN)"):src.index('calFail("align-timeout")')]
    assert "CAL_LOCK_RELEASE_MS" in align


def test_esc_cal_encoder_check_handles_counter_wrap():
    src = read(ESC_MAIN)
    check = src[src.index("cal_encoder_start;"):src.index('calFail("encoder-moving")')]
    assert "ENCODER_CPR / 2u" in check
    assert "ENCODER_CPR - diff" in check


def test_master_cal_start_requires_door_closed():
    src = read(MASTER_COMMANDS)
    start_gate = src[src.index("bool isEscServiceStartState"):src.index("bool isEscServiceStopState")]
    assert "DOOR_STATE_CLOSED" in start_gate


def test_master_releases_gantry_lock_during_esc_calibration():
    src = read(MASTER_STATE)
    lock_policy = src[src.index("Gantry lock is state-derived"):src.index("guard.updateLockActuator")]
    assert "isEscCalibrationActive(ctx)" in lock_policy
    assert "lockActuatorCommanded = false" in lock_policy
