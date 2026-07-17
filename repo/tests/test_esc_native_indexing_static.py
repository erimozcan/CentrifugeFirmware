from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
ESC_MAIN = REPO / "esc_firmware" / "src" / "main.cpp"
MASTER_CONFIG = REPO / "firmware" / "src" / "config.h"
MASTER_MOTOR = REPO / "firmware" / "src" / "motor_interface.cpp"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_master_defaults_to_esc_native_index_path():
    src = read(MASTER_CONFIG)
    # The flag must be ACTIVE (line-start define), not the old commented-out default.
    assert "\n#define ROTATE_VIA_INDEX" in src
    assert "// #define ROTATE_VIA_INDEX" not in src


def test_esc_has_home_verb_and_homed_index_reference():
    src = read(ESC_MAIN)
    assert 'strcmp(tok, "HOME")' in src
    assert "linkHome" in src
    assert "home_ref_rad" in src
    # INDEX targets are relative to the homed detent reference, not raw power-on zero.
    index_body = src[src.index("static void linkIndex"):src.index("static void linkEstop")]
    assert "home_ref_rad + (float)tube * TUBE_STEP_RAD" in index_body


def test_esc_index_uses_crawl_torque_and_restores_spin_profile():
    src = read(ESC_MAIN)
    index_body = src[src.index("static void linkIndex"):src.index("static void linkEstop")]
    assert "applyPidProfile(CRAWL_PROFILE)" in index_body
    disarm_body = src[src.index("static void disarm"):src.index("static void printStat")]
    assert "applyPidProfile(SPIN_PROFILE)" in disarm_body


def test_esc_index_refuses_while_rotor_moving():
    src = read(ESC_MAIN)
    index_body = src[src.index("static void linkIndex"):src.index("static void linkEstop")]
    assert "ERR INDEX rotor moving" in index_body


def test_esc_index_is_settled_velocity_crawl_not_position_servo():
    src = read(ESC_MAIN)
    # A position servo closed through this assembly's stiction limit-cycles (bench
    # 2026-07-17): INDEX must crawl in velocity mode and only report DONE after a
    # settled dwell, with bounded bidirectional correction passes.
    index_body = src[src.index("static void linkIndex"):src.index("static void linkEstop")]
    assert "MotionControlType::velocity" in index_body
    assert "MotionControlType::angle" not in index_body
    assert "INDEX_SETTLE_MS" in src and "INDEX_STOP_LEAD_RAD" in src
    assert "idx_retries" in src and "INDEX_MAX_RETRIES" in src
    assert "err_deg=" in src


def test_esc_motion_verbs_supersede_lingering_cal_state():
    src = read(ESC_MAIN)
    # INDEX/SPIN right after CAL DONE must clear cal state, or the CAL service branch
    # in loop() hijacks the armed motor and instantly disarms it (bench 2026-07-17).
    spin_body = src[src.index("static void linkSpin"):src.index("static void linkStop")]
    index_body = src[src.index("static void linkIndex"):src.index("static void linkEstop")]
    for body in (spin_body, index_body):
        assert 'abortCalibration("superseded", CAL_IDLE)' in body


def test_master_syncs_esc_home_reference_and_reads_st_angle():
    src = read(MASTER_MOTOR)
    assert 'txLiteral("HOME")' in src
    # ST telemetry pos= keeps measuredFrac_ fresh (the INDEX path never polls STAT).
    assert '"pos="' in src


def test_master_resends_index_until_arrival():
    src = read(MASTER_MOTOR)
    body = src[src.index("void MotorInterface::commandIndex"):src.index("void MotorInterface::finishIndexIfNeeded")]
    assert "indexArrived_" in body
    assert body.count("txIndex(escTube)") == 2
