from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
ESC_MAIN = REPO / "esc_firmware" / "src" / "main.cpp"


def test_esc_default_and_sensorless_current_limits_are_5a():
    src = ESC_MAIN.read_text(encoding="utf-8")

    assert "#define BRINGUP_CURRENT    5.0f" in src
    assert "#define SENSORLESS_TRIAL_CURRENT_A  5.0f" in src
    assert "motor.current_limit  = BRINGUP_CURRENT" in src
