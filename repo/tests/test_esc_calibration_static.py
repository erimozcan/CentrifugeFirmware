from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
ESC_MAIN = REPO / "esc_firmware" / "src" / "main.cpp"
MASTER_COMMANDS = REPO / "firmware" / "src" / "command_interface.cpp"
MASTER_MOTOR = REPO / "firmware" / "src" / "motor_interface.cpp"
UI_INDEX = REPO / "ui" / "index.html"
UI_USB = REPO / "ui" / "centrifuge-usb.html"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_esc_has_manual_calibration_state_machine_and_verbs():
    src = read(ESC_MAIN)
    assert "enum CalibrationState" in src
    for state in [
        "CAL_IDLE",
        "CAL_ALIGN",
        "CAL_ENCODER_CHECK",
        "CAL_CURRENT_CHECK",
        "CAL_LOW_SPEED_PID",
        "CAL_OBSERVER_SHADOW",
        "CAL_DONE",
        "CAL_FAILED",
    ]:
        assert state in src

    for symbol in [
        "handleCalCommand",
        "printCalStatus",
        "serviceCalibration",
        'strcmp(tok, "CAL")',
        'strcmp(tok, "PROFILE")',
        'strcmp(tok, "TUNE?")',
    ]:
        assert symbol in src


def test_esc_has_owned_pid_profiles_and_calibration_blocks_sensorless_handoff():
    src = read(ESC_MAIN)
    for symbol in [
        "SPIN_PROFILE",
        "CRAWL_PROFILE",
        "applyPidProfile",
        "printTune",
        "calibrationRunning()",
    ]:
        assert symbol in src

    assert "PROFILE SPIN" in src
    assert "PROFILE CRAWL" in src
    assert "!calibrationRunning()" in src


def test_master_forwards_service_commands_only_as_explicit_safe_verbs():
    src = read(MASTER_COMMANDS)
    for cmd in [
        "CAL_START",
        "CAL_STOP",
        "CAL_STATUS",
        "CAL_APPLY",
        "PROFILE_SPIN",
        "PROFILE_CRAWL",
        "TUNE_STATUS",
    ]:
        assert f'"{cmd}"' in src

    for esc_line in [
        "CAL START",
        "CAL STOP",
        "CAL STATUS?",
        "CAL APPLY",
        "PROFILE SPIN",
        "PROFILE CRAWL",
        "TUNE?",
    ]:
        assert esc_line in src

    assert "isEscServiceStartState" in src
    assert "isEscServiceStopState" in src


def test_master_uses_esc_owned_profiles_instead_of_raw_pid_push():
    src = read(MASTER_MOTOR)
    assert 'txLiteral("PROFILE SPIN")' in src
    assert 'txLiteral("PROFILE CRAWL")' in src
    for legacy in ['"p 0.05"', '"i 0.1"', '"f 0.15"', '"q 0.3"', '"j 20"']:
        assert legacy not in src


def test_ui_exposes_esc_calibration_service_controls():
    for ui in [UI_INDEX, UI_USB]:
        src = read(ui)
        for text in [
            "Start ESC calibration",
            "Stop calibration",
            "CAL_START",
            "CAL_STOP",
            "CAL_STATUS",
            "PROFILE_SPIN",
            "PROFILE_CRAWL",
            "TUNE_STATUS",
            "escCalState",
            "escPhaseErr",
        ]:
            assert text in src


def test_master_allows_cal_stop_with_active_calibration_gate():
    src = read(MASTER_COMMANDS)
    assert "isEscCalibrationActive" in src
    assert "isEscServiceStartState" in src
    assert "isEscServiceStopState" in src

    cal_start = src[src.index('strcmp(cmdToken, "CAL_START")'):src.index('strcmp(cmdToken, "CAL_STOP")')]
    cal_stop = src[src.index('strcmp(cmdToken, "CAL_STOP")'):src.index('strcmp(cmdToken, "CAL_STATUS")')]

    assert "isEscServiceStartState(ctx)" in cal_start
    assert "isEscServiceStopState(ctx)" in cal_stop
    assert "isEscServiceState(ctx)" not in cal_stop


def test_master_blocks_normal_motion_during_esc_calibration():
    src = read(MASTER_COMMANDS)
    assert "isMachineMotionCommandBlocked(ctx)" in src

    for cmd in [
        '"RUN"',
        '"LOCK"',
        '"UNLOCK"',
        '"DOOR_OPEN"',
        '"DOOR_CLOSE"',
        '"ROTATE"',
    ]:
        block_start = src.index(f"strcmp(cmdToken, {cmd})")
        block_end = src.find("return;", block_start)
        block = src[block_start:block_end]
        assert "isMachineMotionCommandBlocked(ctx)" in block


def test_esc_stop_and_estop_abort_active_calibration_state():
    src = read(ESC_MAIN)
    assert "abortCalibration" in src

    link_stop = src[src.index("static void linkStop()"):src.index("// Closed-loop angle move")]
    link_estop = src[src.index("static void linkEstop()"):src.index("static void linkPing()")]

    assert "abortCalibration" in link_stop
    assert "abortCalibration" in link_estop
    assert "calibrationRunning()" in link_stop
    assert "calibrationRunning()" in link_estop
