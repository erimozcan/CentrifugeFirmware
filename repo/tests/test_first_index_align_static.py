"""The first index after power-up is the only one that aligns FOC -- and it used to
overshoot the tube and walk back while later indexes crept in cleanly."""

import re
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
ESC_MAIN = REPO / "esc_firmware" / "src" / "main.cpp"


def index_body():
    src = ESC_MAIN.read_text(encoding="utf-8")
    return src[src.index("static void linkIndex"):src.index("static void linkEstop")]


def test_move_is_planned_after_the_alignment_twitch_settles():
    # initFOC() physically swings the shaft, and encoder.getAngle() is a CACHED value that
    # only advances when the sensor is updated. Reading it on the line after arm() planned
    # the move from a stale mid-twitch angle, so the gantry shot past the tube.
    body = index_body()
    assert "bool fresh_align = !foc_ready" in body
    assert "INDEX_ALIGN_SETTLE_MS" in body
    assert "motor.loopFOC()" in body, "the settle must refresh the encoder, not just wait"

    settle = body.index("if (fresh_align)")
    target = body.index("float target_mech")
    assert settle < target, "the settle has to happen BEFORE the target is computed"


def test_blocking_alignment_refeeds_the_comms_watchdog():
    # arm() blocks for seconds aligning; LINK_WD_MS is 750 ms. The watchdog fired the
    # instant linkIndex returned and disarmed the move it had just set up -- the master
    # was never dead, the ESC was busy.
    src = ESC_MAIN.read_text(encoding="utf-8")
    wd_ms = int(re.search(r"LINK_WD_MS = (\d+)", src).group(1))
    settle_ms = int(re.search(r"INDEX_ALIGN_SETTLE_MS (\d+)U", src).group(1))
    body = index_body()
    after_arm = body[body.index("if (!armed) arm();"):]
    assert "last_link_ms = millis();" in after_arm, "watchdog not refed after arm()"
    if settle_ms >= wd_ms:
        loop = body[body.index("if (fresh_align)"):body.index("float target_mech")]
        assert "last_link_ms = millis();" in loop, (
            "the settle outlasts the watchdog and must refeed it from inside the loop")


def test_sweep_hold_is_a_partial_extend_between_locked_and_released():
    # The exact percentage is a bench tuning value (50 -> 40 -> 30 so far, via LOCK_PCT),
    # so pin the INVARIANT rather than the number: a partial extend, strictly between the
    # two calibrated endpoints, and never so shallow it cannot reach a bucket.
    config = (REPO / "firmware" / "src" / "config.h").read_text(encoding="utf-8")
    esp_block = config[:config.index("// ---- Arduino Due (prototype)")]
    locked = int(re.search(r"#define LOCK_PULSE_LOCKED_US\s+(\d+)", esp_block).group(1))
    released = int(re.search(r"#define LOCK_PULSE_UNLOCKED_US\s+(\d+)", esp_block).group(1))
    sweep = int(re.search(r"#define LOCK_PULSE_SWEEP_US\s+(\d+)", esp_block).group(1))
    assert locked < sweep < released, "the sweep hold must sit between the two endpoints"
    pct = (released - sweep) * 100.0 / (released - locked)
    assert 10.0 <= pct <= 60.0, "sweep travel %.0f%% is outside anything sensible" % pct


def test_lock_travel_can_be_driven_live_for_tuning():
    # Every sweep-depth change used to cost a full reflash. LOCK_PCT <0-100> drives the
    # lock to an explicit travel from the console, and STATUS reads it back.
    cmd = (REPO / "firmware" / "src" / "command_interface.cpp").read_text(encoding="utf-8")
    assert 'strcmp(cmdToken, "LOCK_PCT")' in cmd
    # Same stopped-only gate as LOCK/UNLOCK -- it must not be usable while spinning.
    body = cmd[cmd.index('strcmp(cmdToken, "LOCK_PCT")'):]
    body = body[:body.index("DOOR_OPEN")]
    assert "isLockControlState(ctx.state)" in body and "isMachineMotionCommandBlocked" in body

    protocol = (REPO / "firmware" / "src" / "protocol.cpp").read_text(encoding="utf-8")
    assert "LOCKPCT=" in protocol

    # The percentage <-> pulse maths lives in ONE place, keyed off the two endpoints.
    context = (REPO / "firmware" / "src" / "context.h").read_text(encoding="utf-8")
    assert "lockPulseForPercent" in context and "lockPercentForPulse" in context

    client = (REPO / "opentrons" / "centrifuge.py").read_text(encoding="utf-8")
    assert "def lock_percent" in client

    # Both consoles carry the slider -- they drift apart easily, so check the pair.
    for ui in ("index.html", "centrifuge-usb.html"):
        html = (REPO / "ui" / ui).read_text(encoding="utf-8")
        assert 'id="lockPctSlider"' in html, "%s has no lock travel slider" % ui
        assert "'LOCK_PCT '+e.target.value" in html, "%s slider sends nothing" % ui
        # Commanding on release, not on every input event, keeps a drag off the link.
        assert "$('lockPctSlider').onchange" in html, "%s commands on drag" % ui
        assert "f.LOCKPCT" in html, "%s never reads the real travel back" % ui
