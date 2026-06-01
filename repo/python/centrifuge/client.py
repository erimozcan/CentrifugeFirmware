import time
from typing import Optional

import serial

from .exceptions import CommandTimeoutError, DeviceError, ProtocolError
from .models import Response, RunProfile, Status
from .protocol import build_command, parse_response, run_params, status_from_response


class CentrifugeClient:
    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout_s: float = 1.0,
        read_slice_s: float = 0.05,
    ) -> None:
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=read_slice_s,
            write_timeout=0,
        )
        self.timeout_s = timeout_s
        self._next_seq_value = 1
        self._in_flight = False

    def close(self) -> None:
        if self.ser.is_open:
            self.ser.close()

    def __enter__(self) -> "CentrifugeClient":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def ping(self) -> Response:
        return self._send_command("PING")

    def version(self) -> Response:
        return self._send_command("VERSION")

    def status(self) -> Status:
        response = self._send_command("STATUS")
        return status_from_response(response)

    def init(self) -> Response:
        return self._send_command("INIT")

    def run(self, profile: RunProfile) -> Response:
        return self._send_command("RUN", run_params(profile))

    def abort(self) -> Response:
        return self._send_command("ABORT")

    def hardstop(self) -> Response:
        return self._send_command("HARDSTOP")

    def clear_fault(self) -> Response:
        return self._send_command("CLEAR_FAULT")

    def lock(self) -> Response:
        return self._send_command("LOCK")

    def unlock(self) -> Response:
        return self._send_command("UNLOCK")

    def door_open(self) -> Response:
        return self._send_command("DOOR_OPEN")

    def door_close(self) -> Response:
        return self._send_command("DOOR_CLOSE")

    def _next_seq(self) -> int:
        seq = self._next_seq_value
        self._next_seq_value += 1
        if self._next_seq_value > 999999:
            self._next_seq_value = 1
        return seq

    def _send_command(self, cmd: str, params: Optional[dict] = None) -> Response:
        if self._in_flight:
            raise ProtocolError("only one in-flight command is allowed")

        self._in_flight = True
        try:
            expected_seq = self._next_seq()
            self.ser.reset_input_buffer()

            payload = build_command(expected_seq, cmd, params)
            self.ser.write(payload)

            deadline = time.monotonic() + self.timeout_s
            while time.monotonic() < deadline:
                raw = self.ser.readline()
                if not raw:
                    continue

                try:
                    line = raw.decode("ascii", errors="strict")
                except UnicodeDecodeError:
                    continue

                try:
                    response = parse_response(line)
                except ProtocolError:
                    continue

                if response.seq != expected_seq:
                    continue

                if not response.ok:
                    raise DeviceError(response.error_code or "UNKNOWN")

                return response

            raise CommandTimeoutError(f"timeout waiting for response to seq {expected_seq}")
        finally:
            self._in_flight = False
