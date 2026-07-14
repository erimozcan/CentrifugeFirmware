from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
UI_FILES = [
    REPO / "ui" / "index.html",
    REPO / "ui" / "centrifuge-usb.html",
]


def test_ui_exposes_high_speed_validation_toggle():
    for ui_file in UI_FILES:
        html = ui_file.read_text(encoding="utf-8")
        assert 'id="hsMode"' in html, f"{ui_file} missing high-speed toggle"
        assert 'id="hsWarning"' in html, f"{ui_file} missing high-speed warning"
        assert "RUN_RPM_LIMITS" in html, f"{ui_file} missing named RPM limits"
        assert "function setHighSpeedMode" in html, f"{ui_file} missing mode switcher"
        assert "const HIGH_SPEED_SENSORLESS = false" not in html
