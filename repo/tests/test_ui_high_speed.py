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


def test_ui_rcf_operator_cap_is_under_the_rpm_ceiling():
    # MAX_RCF is an OPERATOR cap (6000 x g for now, 2026-08-03), deliberately below
    # the validated hardware ceiling (~8452 at 12000 RPM). It must stay under that
    # ceiling so the RPM it maps to (~10.1k) never exceeds the firmware clamp, and
    # the input field's max attribute must match the JS cap.
    import re
    rcf_per_rpm2 = 1.118e-5 * 5.25          # mirrors the UI's RCF_PER_RPM2 (r=52.5 mm)
    for ui_file in UI_FILES:
        html = ui_file.read_text(encoding="utf-8")
        m = re.search(r"const MAX_RCF = (\d+);", html)
        assert m, f"{ui_file} MAX_RCF not a literal cap"
        cap = int(m.group(1))
        assert cap == 6000, f"{ui_file} unexpected operator cap {cap}"
        assert (cap / rcf_per_rpm2) ** 0.5 <= 12000, f"{ui_file} cap exceeds the RPM ceiling"
        assert f'id="inRcf" value="1000" min="0" max="{cap}"' in html, f"{ui_file} field max drifted"
