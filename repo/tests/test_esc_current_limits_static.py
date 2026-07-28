from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
ESC_MAIN = REPO / "esc_firmware" / "src" / "main.cpp"


def test_esc_current_limits():
    src = ESC_MAIN.read_text(encoding="utf-8")

    # 8 A spin limit (2026-07-28): loaded-tube drag needs >5 A to reach the top of
    # the 24 V range. Board peak is 40 A, motor 18 A / 200 W for 30 s.
    assert "#define BRINGUP_CURRENT    8.0f" in src
    # The (disabled) experimental sensorless drive keeps its conservative 5 A trip.
    assert "#define SENSORLESS_TRIAL_CURRENT_A  5.0f" in src
    assert "motor.current_limit  = BRINGUP_CURRENT" in src
