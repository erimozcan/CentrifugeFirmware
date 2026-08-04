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
