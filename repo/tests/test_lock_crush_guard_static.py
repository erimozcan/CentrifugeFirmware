import re
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
MASTER_STATE = REPO / "firmware" / "src" / "state_machine.cpp"
MASTER_GUARD = REPO / "firmware" / "src" / "hardware_guard.cpp"
MASTER_MOTOR = REPO / "firmware" / "src" / "motor_interface.cpp"
ESC_MAIN = REPO / "esc_firmware" / "src" / "main.cpp"


def test_full_lock_throw_requires_a_verified_detent():
    # The crush guard (2026-08-04): a mid-cycle brownout reboot once re-extended the
    # lock at a half-detent and crushed a tube. The full throw must be gated on the
    # ESC-reported angle actually sitting on a learned detent, and index arrivals must
    # be verified before engaging (fault, not crush, on a mismatch).
    src = MASTER_STATE.read_text(encoding="utf-8")
    assert "motor.detentVerified()" in src
    assert "FAULT_DETENT_MISMATCH" in src
    # The gate must sit in the lock-policy block, before the actuator write.
    policy = src[src.index("Gantry lock is state-derived"):src.index("guard.updateLockActuator")]
    assert "detentVerified" in policy


def test_lock_boots_retracted_not_extended():
    # Boot must NOT blind-extend the lock: after a mid-cycle reboot the gantry can be
    # anywhere (e.g. a bucket under the pin). The state machine re-engages once the
    # detent is verified.
    src = MASTER_GUARD.read_text(encoding="utf-8")
    boot = src[src.index("void HardwareGuard::begin"):src.index("void HardwareGuard::updateMotorEnable")]
    assert "lockServo_.writeMicroseconds(LOCK_PULSE_UNLOCKED_US)" in boot
    assert "writeMicroseconds(LOCK_PULSE_LOCKED_US)" not in boot


def test_detent_capture_only_happens_where_a_detent_is_known():
    # Learning "detent 0" mid-cycle (post-spin the rotor stops anywhere) would shift the
    # whole grid and put the lock into a bucket, so a capture is gated on being at rest.
    motor = MASTER_MOTOR.read_text(encoding="utf-8")
    body = motor[motor.index("void MotorInterface::maintainHome"):]
    body = body[:body.index("\nbool MotorInterface::homeSet")]
    assert "bool atDetent" in motor or "maintainHome(bool atDetent)" in motor
    assert "if (!atDetent)" in body, "capture is not gated on being parked at a detent"

    state = MASTER_STATE.read_text(encoding="utf-8")
    assert "bool atDetent = (ctx.state == STATE_BOOT || ctx.state == STATE_SAFE_IDLE)" in state
    assert "motor.maintainHome(atDetent)" in state


def test_master_reconciles_when_the_esc_loses_its_home_reference():
    # An ESC reboot/reflash silently resets its reference to raw encoder zero, which is
    # NOT a detent -- INDEX then drives the pin into a bucket. Flashing Nano-then-ESC
    # produced exactly this (2026-08-04): every run ended in FAULT_DETENT_MISMATCH.
    motor = MASTER_MOTOR.read_text(encoding="utf-8")
    body = motor[motor.index("void MotorInterface::maintainHome"):]
    body = body[:body.index("\nbool MotorInterface::homeSet")]
    # ESC says "not homed" while we think we are -> re-teach it if we can still prove a
    # detent, otherwise drop our own reference rather than trusting a stale grid.
    assert "!escHomeSet_" in body
    assert 'txLiteral("HOME")' in body and "detentVerified()" in body
    assert "homeSet_ = false" in body
    # ... and a drifted-but-present ESC reference wins, since INDEX moves to ITS grid.
    assert "detentRef_ = escHomeRefFrac_" in body


def test_status_exposes_the_crush_guard_diagnostic():
    # A DETENT_MISMATCH fault must be diagnosable from the console, not by removing panels.
    protocol = (REPO / "firmware" / "src" / "protocol.cpp").read_text(encoding="utf-8")
    assert "DETENT_OK=" in protocol and "DETENT_D10=" in protocol
    state = MASTER_STATE.read_text(encoding="utf-8")
    assert "ctx.detentErrDeg10 = motor.detentErrorDeg10()" in state


def test_sweep_progress_does_not_assume_a_rotation_direction():
    # Nothing else in the firmware depends on the sign of the ESC's cumulative rev=;
    # if SPIN drives the shaft the other way, a signed test would never finish the turn
    # and the run would die on the rotate timeout.
    motor = MASTER_MOTOR.read_text(encoding="utf-8")
    body = motor[motor.index("void MotorInterface::updateSweepTurn"):]
    body = body[:body.index("bool MotorInterface::sweepTurnDone")]
    assert "fabsf(measuredRev_ - sweepStartRev_) >= SWEEP_REV_TARGET" in body


def test_sweep_hold_is_not_dropped_by_crawl_speed_jitter():
    # The sweeper is meant to touch buckets at SWEEP_TURN_RPM, so the ordinary
    # SAFE_UNLOCK_RPM net (50) would retract it on measurement noise and silently skip
    # buckets. It gets its own, higher backstop.
    config = (REPO / "firmware" / "src" / "config.h").read_text(encoding="utf-8")
    turn = int(re.search(r"#define SWEEP_TURN_RPM\s+(\d+)", config).group(1))
    abort = int(re.search(r"#define SWEEP_ABORT_RPM\s+(\d+)", config).group(1))
    assert abort > turn * 2, "the sweep backstop sits too close to the crawl speed"
    state = MASTER_STATE.read_text(encoding="utf-8")
    hold = state[state.index("ctx.lockSweepHold ="):]
    assert "SWEEP_ABORT_RPM" in hold[:200]


def test_master_adopts_surviving_esc_home_reference():
    # The ESC rides the 24 V bus and survives a master brownout reboot; the master must
    # adopt its explicitly-homed reference instead of blindly re-capturing home at
    # whatever mid-cycle angle the gantry was left (that shift is what put the lock on
    # a bucket). Only an explicit HOME request forces a fresh capture.
    motor = MASTER_MOTOR.read_text(encoding="utf-8")
    assert "escHomeSet_" in motor and "escHomeRefFrac_" in motor
    assert 'strstr(line, "home=")' in motor and 'strstr(line, "href=")' in motor
    esc = ESC_MAIN.read_text(encoding="utf-8")
    assert "home_explicit" in esc
    assert esc.count('" home="') + esc.count('"home="') + esc.count("\" home=\"") >= 1
