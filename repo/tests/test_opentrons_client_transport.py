"""Transport-level tests: drive the client against a fake firmware, no hardware.

The fake answers exactly like repo/firmware/src/protocol.cpp -- sequence-numbered
"<seq> OK ..." / "<seq> ERR CODE=..." lines, interleaved with the unsolicited ESC
telemetry the real device emits between replies.
"""

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "opentrons"))
import centrifuge  # noqa: E402


STATUS_PAYLOAD = (
    "STATE=1 RPM_CMD=0 RPM1=0 ESC_CAL=idle ESC_CAL_STEP=0 FAULT=0 LOCK=1 LOCKCMD=1 "
    "ENABLE=0 DOOR=2 DOORMOVE=0 DOORCMD=0 DOORPWM=220 TUBE=1 HOMED=1 ROTTGT=0 "
    "TEMP_ADC=2500 FAN=0 POWER=1 AUDIO=1"
)


class FakeSerial:
    """Minimal pyserial stand-in that speaks the firmware's line protocol."""

    def __init__(self, status_payload=STATUS_PAYLOAD, error_code=None, noise=True):
        self.status_payload = status_payload
        self.error_code = error_code
        self.noise = noise
        self.sent = []
        self._out = []
        self.is_open = True

    # -- pyserial surface used by the client --
    def reset_input_buffer(self):
        pass

    def flush(self):
        pass

    def close(self):
        self.is_open = False

    def write(self, data):
        line = data.decode("ascii").strip()
        self.sent.append(line)
        seq, _, rest = line.partition(" ")
        verb = rest.split(" ")[0]
        if self.noise:
            # The real device interleaves ESC telemetry between replies.
            self._out.append(b"ESC ST rpm=0 state=idle\n")
        if self.error_code:
            self._out.append(("%s ERR CODE=%s\n" % (seq, self.error_code)).encode())
        elif verb == "VERSION":
            self._out.append(("%s OK FW=GEN1_DUE_V1\n" % seq).encode())
        elif verb == "STATUS":
            self._out.append(("%s OK %s\n" % (seq, self.status_payload)).encode())
        else:
            self._out.append(("%s OK QUEUED=1\n" % seq).encode())
        return len(data)

    def readline(self):
        return self._out.pop(0) if self._out else b""


def client_with(fake):
    cf = centrifuge.Centrifuge(connect=False)
    cf._ser = fake
    return cf


def test_reply_is_matched_by_sequence_number_through_telemetry_noise():
    fake = FakeSerial()
    cf = client_with(fake)
    assert cf.command("VERSION") == "FW=GEN1_DUE_V1"
    assert cf.command("VERSION") == "FW=GEN1_DUE_V1"
    # Sequence numbers must advance, or the second reply would match the first's pattern.
    assert [line.split(" ")[0] for line in fake.sent] == ["1", "2"]


def test_status_parses_into_typed_fields():
    cf = client_with(FakeSerial())
    status = cf.status()
    assert status["STATE"] == 1 and isinstance(status["STATE"], int)
    assert status["DOOR"] == centrifuge.DOOR_CLOSED
    assert status["TUBE"] == 1
    assert status["ESC_CAL"] == "idle"          # non-numeric stays a string


def test_error_replies_raise_with_the_firmware_code():
    cf = client_with(FakeSerial(error_code="ILLEGAL_STATE"))
    try:
        cf.door_open(wait=False)
    except centrifuge.CommandError as exc:
        assert exc.code == "ILLEGAL_STATE"
        assert "DOOR_OPEN" in str(exc)
    else:
        raise AssertionError("an ERR reply did not raise")


def test_timeout_when_the_device_says_nothing():
    class Mute(FakeSerial):
        def write(self, data):
            self.sent.append(data.decode("ascii").strip())
            return len(data)

    cf = client_with(Mute())
    cf.timeout = 0.05
    try:
        cf.command("STATUS")
    except centrifuge.CentrifugeTimeout:
        pass
    else:
        raise AssertionError("a mute device did not time out")


def test_waiting_aborts_the_moment_a_fault_latches():
    faulted = STATUS_PAYLOAD.replace("STATE=1", "STATE=9").replace("FAULT=0", "FAULT=11")
    cf = client_with(FakeSerial(status_payload=faulted))
    try:
        cf.wait_until(lambda s: False, timeout=5.0, desc="never")
    except centrifuge.MachineFault as exc:
        assert exc.fault == 11
        assert "detent mismatch" in str(exc)      # the crush guard's fault, named
    else:
        raise AssertionError("a latched fault did not interrupt the wait")


def test_spin_sends_the_run_line_the_firmware_parser_expects():
    cf = client_with(FakeSerial())
    cf.spin(rcf=4000, minutes=2, ramp_seconds=120, wait=False)
    run = [line for line in cf._ser.sent if " RUN " in line][0]
    rpm = centrifuge.rcf_to_rpm(4000)
    assert "LIFT=%d" % rpm in run and "FINAL=%d" % rpm in run
    assert "HOLD=120000" in run                   # 2 min
    assert "RAMPUP=120000" in run and "RAMPDOWN=120000" in run
    assert "SEAT=500" in run


def test_rotate_rejects_tubes_outside_the_carousel():
    cf = client_with(FakeSerial())
    for bad in (0, 5, -1):
        try:
            cf.rotate(bad)
        except ValueError:
            pass
        else:
            raise AssertionError("rotate accepted tube %s" % bad)


def test_handshake_rejects_a_port_that_is_not_the_centrifuge():
    class Smoothie(FakeSerial):
        def write(self, data):
            self.sent.append(data.decode("ascii").strip())
            self._out.append(b"ok\n")             # what a motion controller answers
            return len(data)

    cf = client_with(Smoothie())
    cf.timeout = 0.05
    assert cf._handshake(settle=0.2) is False
