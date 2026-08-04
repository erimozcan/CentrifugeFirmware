"""Python client for the centrifuge master controller (Arduino Nano ESP32).

Speaks the same line protocol the browser console uses over Web Serial:

    host -> device :  "<seq> VERB args\\n"
    device -> host :  "<seq> OK <payload>"   or   "<seq> ERR CODE=<code>"

One command is in flight at a time and replies are matched by sequence number,
so unsolicited device output (ESC telemetry, boot banners) is simply ignored.

Used from three places:
  * repo/opentrons/centrifuge_control.py -- the OT-2 protocol (inlined verbatim;
    the Opentrons app uploads a SINGLE file, so it cannot import this one)
  * repo/opentrons/console.py            -- interactive debug console
  * a Jupyter notebook on the OT-2       -- import and drive it cell by cell

Machine constants below MIRROR the firmware and UI. repo/tests/
test_opentrons_client_static.py fails if they drift.
"""

import glob
import math
import os
import re
import time

BAUD = 115200
TUBE_COUNT = 4

# Rotor geometry -> RCF. Must match ROTOR_RADIUS_MM / RCF_PER_RPM2 in repo/ui/index.html.
ROTOR_RADIUS_MM = 52.5
RCF_PER_RPM2 = 1.118e-5 * (ROTOR_RADIUS_MM / 10.0)   # r in cm
MAX_RCF = 6000        # operator cap (repo/ui/index.html)
MAX_RUN_RPM = 12000   # firmware ceiling (MAX_RPM_BAREBONES, high-speed build)

# SystemState (repo/firmware/src/context.h)
STATE_BOOT = 0
STATE_SAFE_IDLE = 1
STATE_HARD_STOP = 8
STATE_FAULT_LATCHED = 9
IDLE_STATES = frozenset((STATE_BOOT, STATE_SAFE_IDLE))
FAULT_STATES = frozenset((STATE_HARD_STOP, STATE_FAULT_LATCHED))

STATE_NAMES = {
    0: "BOOT", 1: "SAFE IDLE", 2: "PRE-RUN", 3: "LIFT RAMP", 4: "SEAT HOLD",
    5: "MAIN RAMP", 6: "SPIN HOLD", 7: "STOPPING", 8: "HARD STOP", 9: "FAULT",
    10: "CLOSING", 11: "UNLOCKING", 12: "SETTLING", 13: "LOCKING", 14: "OPENING",
    15: "ROTATE UNLOCK", 16: "ROTATING", 17: "ROTATE LOCK", 18: "INDEXING",
    19: "FINISHING - SWEEP", 20: "FINISHING - SWEEP",
}

# FaultCode (repo/firmware/src/context.h)
FAULT_NAMES = {
    0: "none", 1: "illegal state", 2: "bad command", 3: "hard stop",
    4: "lock timeout", 5: "door invalid", 6: "door open in spin",
    7: "door timeout", 8: "over temp", 9: "interlock unsafe",
    10: "rotate timeout", 11: "detent mismatch",
}

# DoorState (repo/firmware/src/context.h)
DOOR_UNKNOWN, DOOR_OPEN, DOOR_CLOSED, DOOR_INVALID = 0, 1, 2, 3
DOOR_NAMES = {0: "unknown", 1: "open", 2: "closed", 3: "invalid"}


class CentrifugeError(Exception):
    """Base class for every failure this client raises."""


class NotConnected(CentrifugeError):
    pass


class CommandError(CentrifugeError):
    """The device answered ERR. `code` is the firmware's error code."""

    def __init__(self, verb, code):
        super().__init__("%s rejected: %s" % (verb, code))
        self.verb = verb
        self.code = code


class CentrifugeTimeout(CentrifugeError):
    pass


class MachineFault(CentrifugeError):
    """The machine latched a fault while we were waiting on it."""

    def __init__(self, status):
        fault = status.get("FAULT", 0)
        super().__init__(
            "machine faulted: %s (state %s)"
            % (FAULT_NAMES.get(fault, fault), STATE_NAMES.get(status.get("STATE"), status.get("STATE")))
        )
        self.status = status
        self.fault = fault


def rcf_to_rpm(rcf):
    """Relative centrifugal force (x g) -> RPM, clamped to the machine's range."""
    if rcf < 0:
        rcf = 0
    rpm = int(round(math.sqrt(rcf / RCF_PER_RPM2)))
    return max(0, min(rpm, MAX_RUN_RPM))


def rpm_to_rcf(rpm):
    """RPM -> relative centrifugal force (x g) at the tube radius."""
    return RCF_PER_RPM2 * rpm * rpm


def describe_detent(status):
    """Human form of the crush guard's view: is the pin over a real detent?

    DETENT_D10 is tenths of a degree to the nearest learned detent, -1 when there is no
    home reference or the ESC's angle is stale. This is what a "detent mismatch" fault
    is complaining about, so it is worth reading before opening the panels.
    """
    err = status.get("DETENT_D10")
    if err is None:
        return "detent ?"                     # firmware predates the diagnostic
    if err < 0:
        return "detent UNKNOWN (no home reference or stale ESC angle)"
    return "detent %s (%.1f deg off)" % ("ok" if status.get("DETENT_OK") else "OFF", err / 10.0)


def _candidate_ports():
    """Serial devices that could plausibly be the centrifuge, best guess first.

    On the OT-2 the robot's OWN motor controller is also a USB CDC device, so
    stable by-id names are preferred and anything that looks like the
    smoothieboard is dropped outright. Every candidate still has to pass the
    VERSION handshake before this client will talk to it.
    """
    by_id = sorted(glob.glob("/dev/serial/by-id/*"))
    wanted, other = [], []
    for path in by_id:
        low = os.path.basename(path).lower()
        if "smoothie" in low or "marlin" in low:
            continue                                    # the robot's own motion controller
        (wanted if ("arduino" in low or "esp32" in low or "espressif" in low) else other).append(path)
    # macOS (bench/dev) has no by-id tree.
    mac = sorted(glob.glob("/dev/cu.usbmodem*"))
    return wanted + other + mac + sorted(glob.glob("/dev/ttyACM*"))


class Centrifuge(object):
    """Synchronous client. Use as a context manager, or call close() yourself."""

    def __init__(self, port=None, timeout=3.0, connect=True):
        self.port = port or os.environ.get("CENTRIFUGE_PORT") or None
        self.timeout = timeout
        self._ser = None
        self._seq = 0
        if connect:
            self.connect()

    # ---- connection ------------------------------------------------------
    def connect(self):
        try:
            import serial                                   # pyserial
        except ImportError:
            raise CentrifugeError(
                "pyserial is not installed. On the OT-2, SSH in and run:\n"
                "    pip install pyserial"
            )

        candidates = [self.port] if self.port else _candidate_ports()
        if not candidates:
            raise NotConnected(
                "no serial devices found. Is the Nano's USB plugged in? "
                "Set CENTRIFUGE_PORT to name one explicitly."
            )

        errors = []
        for path in candidates:
            ser = serial.Serial()
            ser.port = path
            ser.baudrate = BAUD
            ser.timeout = 0.2
            # Opening a USB-CDC port can reset the ESP32; don't assert the lines.
            ser.dtr = False
            ser.rts = False
            try:
                ser.open()
            except Exception as exc:                        # busy, permissions, vanished
                errors.append("%s: %s" % (path, exc))
                continue

            self._ser = ser
            self._seq = 0
            if self._handshake():
                self.port = path
                return self
            self._ser = None
            ser.close()
            errors.append("%s: no centrifuge firmware on this port" % path)

        raise NotConnected(
            "could not find the centrifuge. Tried:\n  " + "\n  ".join(errors)
            + "\nIf the browser console is connected to it, disconnect there first "
              "-- only one program can hold the port."
        )

    def _handshake(self, settle=6.0):
        """VERSION must answer. Retries: the board may be rebooting from the open()."""
        deadline = time.monotonic() + settle
        while time.monotonic() < deadline:
            try:
                if self._txn("VERSION", timeout=1.0).startswith("FW="):
                    return True
            except CentrifugeError:
                time.sleep(0.25)
        return False

    def close(self):
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # ---- transport -------------------------------------------------------
    def _txn(self, verb, timeout=None):
        """Send one command, return its OK payload. Raises on ERR/timeout."""
        if self._ser is None:
            raise NotConnected("not connected -- call connect() first")
        limit = self.timeout if timeout is None else timeout
        self._seq += 1
        seq = self._seq
        try:
            self._ser.reset_input_buffer()      # drop unsolicited telemetry
            self._ser.write(("%d %s\n" % (seq, verb)).encode("ascii"))
            self._ser.flush()
        except Exception as exc:
            raise NotConnected("write failed (cable unplugged?): %s" % exc)

        pattern = re.compile(r"^%d\s+(OK|ERR)\b(.*)$" % seq)
        deadline = time.monotonic() + limit
        while time.monotonic() < deadline:
            raw = self._ser.readline()
            if not raw:
                continue
            match = pattern.match(raw.decode("ascii", "replace").strip())
            if match is None:
                continue                         # someone else's line; keep reading
            kind, rest = match.group(1), match.group(2).strip()
            if kind == "ERR":
                code = rest[5:] if rest.startswith("CODE=") else rest
                raise CommandError(verb, code)
            return rest
        raise CentrifugeTimeout("no reply to %r within %.1fs" % (verb, limit))

    def command(self, verb, timeout=None):
        """Send a raw verb (e.g. "DOOR_SPEED 200"). Returns the OK payload."""
        return self._txn(verb, timeout=timeout)

    # ---- state -----------------------------------------------------------
    def status(self):
        """Full STATUS as a dict; numeric fields are ints (STATE, DOOR, TUBE, ...)."""
        out = {}
        for part in self._txn("STATUS").split():
            if "=" not in part:
                continue
            key, value = part.split("=", 1)
            out[key] = int(value) if re.match(r"^-?\d+$", value) else value
        return out

    def describe(self):
        """One-line human summary, the same facts the console header shows."""
        s = self.status()
        return "%-18s door %-7s tube %s  lock %s  %5d rpm (%d x g)  %s%s" % (
            STATE_NAMES.get(s.get("STATE"), "state %s" % s.get("STATE")),
            DOOR_NAMES.get(s.get("DOOR"), "?"),
            s.get("TUBE") or "?",
            "in" if s.get("LOCKCMD") else "out",
            s.get("RPM1", 0),
            round(rpm_to_rcf(s.get("RPM1", 0))),
            describe_detent(s),
            "" if not s.get("FAULT") else "  FAULT: %s" % FAULT_NAMES.get(s["FAULT"], s["FAULT"]),
        )

    def wait_until(self, predicate, timeout, desc, poll=0.3, on_poll=None):
        """Poll STATUS until predicate(status) is true.

        Raises MachineFault the moment the machine latches a fault, so a stuck
        door or a detent mismatch surfaces immediately instead of at timeout.
        """
        deadline = time.monotonic() + timeout
        while True:
            status = self.status()
            if status.get("FAULT") or status.get("STATE") in FAULT_STATES:
                raise MachineFault(status)
            if predicate(status):
                return status
            if on_poll is not None:
                on_poll(status)
            if time.monotonic() >= deadline:
                raise CentrifugeTimeout(
                    "timed out after %.0fs waiting for %s (now: %s)"
                    % (timeout, desc, STATE_NAMES.get(status.get("STATE"), status.get("STATE")))
                )
            time.sleep(poll)

    # ---- controls (the browser console's buttons, one method each) -------
    def power_on(self):
        self._txn("POWER_ON")

    def power_off(self):
        self._txn("POWER_OFF")

    def init(self):
        """BOOT -> SAFE_IDLE. Harmless when already idle."""
        self._txn("INIT")
        self.wait_until(lambda s: s.get("STATE") == STATE_SAFE_IDLE, 10.0, "arm")

    def clear_fault(self):
        self._txn("CLEAR_FAULT")
        self.wait_until(lambda s: not s.get("FAULT"), 10.0, "fault to clear")

    def abort(self):
        """Ramp down a running spin."""
        self._txn("ABORT")

    def hard_stop(self):
        """E-STOP: immediate torque cut, latches a fault."""
        self._txn("HARDSTOP")

    def door_open(self, wait=True, timeout=30.0):
        self._txn("DOOR_OPEN")
        if wait:
            self.wait_until(
                lambda s: s.get("DOOR") == DOOR_OPEN and not s.get("DOORMOVE"),
                timeout, "the door to open")

    def door_close(self, wait=True, timeout=30.0):
        self._txn("DOOR_CLOSE")
        if wait:
            self.wait_until(
                lambda s: s.get("DOOR") == DOOR_CLOSED and not s.get("DOORMOVE"),
                timeout, "the door to close")

    def rotate(self, tube, wait=True, timeout=90.0):
        """Index the gantry so `tube` (1..4) faces the pipette."""
        if not 1 <= int(tube) <= TUBE_COUNT:
            raise ValueError("tube must be 1..%d" % TUBE_COUNT)
        tube = int(tube)
        self._txn("ROTATE %d" % tube)
        if wait:
            self.wait_until(
                lambda s: s.get("STATE") in IDLE_STATES and s.get("TUBE") == tube,
                timeout, "tube %d" % tube)

    def lock(self, wait=True, timeout=10.0):
        """DEBUG: force the gantry lock in. Overrides the automatic policy while idle."""
        self._txn("LOCK")
        if wait:
            self.wait_until(lambda s: s.get("LOCKCMD") == 1, timeout, "the lock to engage")

    def unlock(self, wait=True, timeout=10.0):
        """DEBUG: force the gantry lock out."""
        self._txn("UNLOCK")
        if wait:
            self.wait_until(lambda s: s.get("LOCKCMD") == 0, timeout, "the lock to release")

    def home(self):
        """Capture the CURRENT angle as the tube-1 detent. Gantry must sit in a detent."""
        self._txn("HOME")

    def spin(self, rcf, minutes, ramp_seconds=120, wait=True, on_poll=None):
        """Run the full automatic cycle: close door, spin, sweep buckets, open door.

        rcf/minutes mirror the browser console's Force and Time fields. Returns the
        commanded RPM. With wait=True this blocks for the whole run.
        """
        if rcf > MAX_RCF:
            raise ValueError("%g x g exceeds the %d x g operator cap" % (rcf, MAX_RCF))
        if minutes <= 0:
            raise ValueError("spin time must be positive")
        rpm = rcf_to_rpm(rcf)
        hold_ms = int(round(minutes * 60000))
        ramp_ms = max(1000, int(round(ramp_seconds * 1000)))

        if self.status().get("STATE") == STATE_BOOT:
            self.init()

        self._txn("RUN LIFT=%d FINAL=%d SEAT=500 HOLD=%d RAMPUP=%d RAMPDOWN=%d"
                  % (rpm, rpm, hold_ms, ramp_ms, ramp_ms))
        if not wait:
            return rpm

        # Leave idle first, so a run that never starts is caught as its own failure.
        self.wait_until(lambda s: s.get("STATE") not in IDLE_STATES, 30.0, "the run to start")
        # Then the whole cycle. Tail = door moves + settle + bucket sweep + re-index.
        budget = (hold_ms + 2 * ramp_ms) / 1000.0 + 300.0
        self.wait_until(lambda s: s.get("STATE") in IDLE_STATES, budget,
                        "the run to finish", poll=1.0, on_poll=on_poll)
        return rpm
