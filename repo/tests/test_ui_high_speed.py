from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
UI_FILES = [
    REPO / "ui" / "index.html",
    REPO / "ui" / "centrifuge-usb.html",
]


def test_ui_speed_is_rcf_at_rotor_radius():
    # Operator speed is RCF (x g) at the tube radius; the protocol stays RPM.
    for ui_file in UI_FILES:
        html = ui_file.read_text(encoding="utf-8")
        assert "const ROTOR_RADIUS_MM = 52.5" in html, f"{ui_file} missing rotor radius"
        assert "rpmToRcf" in html and "rcfToRpm" in html, f"{ui_file} missing RCF conversion"
        assert 'id="inRcf"' in html, f"{ui_file} speed input not RCF"


def test_ui_full_range_is_default_after_validation():
    # 10,000 RPM (~5869 x g) was validated on the 24 V bus 2026-07-28: the old
    # high-speed opt-in toggle is gone and the full range is simply the range.
    for ui_file in UI_FILES:
        html = ui_file.read_text(encoding="utf-8")
        assert "const MAX_RUN_RPM = 12000" in html, f"{ui_file} missing 12k ceiling"
        assert 'id="hsMode"' not in html, f"{ui_file} still has the high-speed toggle"
        assert 'id="hsWarning"' not in html, f"{ui_file} still has the high-speed warning"
        assert "RUN_RPM_LIMITS" not in html, f"{ui_file} still has dual-mode RPM limits"


def test_ui_rcf_max_matches_validated_rpm_ceiling():
    # MAX_RCF must DERIVE from the validated RPM ceiling (floor keeps rpm <= cap),
    # never a hand-typed number that could drift from the firmware clamp.
    for ui_file in UI_FILES:
        html = ui_file.read_text(encoding="utf-8")
        assert "Math.floor(rpmToRcf(MAX_RUN_RPM))" in html, f"{ui_file} MAX_RCF not derived"
