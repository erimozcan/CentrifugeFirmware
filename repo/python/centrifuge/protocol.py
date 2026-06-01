from typing import Dict, Optional

from .exceptions import ProtocolError
from .models import Response, RunProfile, Status


def build_command(seq: int, cmd: str, params: Optional[Dict[str, int]] = None) -> bytes:
    parts = [str(int(seq)), cmd]
    if params:
        for key, value in params.items():
            parts.append(f"{key}={int(value)}")
    return (" ".join(parts) + "\n").encode("ascii")


def parse_response(line: str) -> Response:
    text = line.strip()
    if not text:
        raise ProtocolError("empty response")

    parts = text.split()
    if len(parts) < 2:
        raise ProtocolError(f"malformed response: {text}")

    try:
        seq = int(parts[0])
    except ValueError as exc:
        raise ProtocolError(f"invalid sequence: {parts[0]}") from exc

    status = parts[1]
    if status not in {"OK", "ERR"}:
        raise ProtocolError(f"invalid response type: {status}")

    fields: Dict[str, str] = {}
    for token in parts[2:]:
        if "=" not in token:
            raise ProtocolError(f"invalid token: {token}")
        key, value = token.split("=", 1)
        fields[key] = value

    if status == "ERR":
        code = fields.get("CODE")
        if not code:
            raise ProtocolError("ERR response without CODE")
        return Response(seq=seq, ok=False, fields=fields, error_code=code)

    return Response(seq=seq, ok=True, fields=fields)


def status_from_response(response: Response) -> Status:
    if not response.ok:
        raise ProtocolError("cannot build status from ERR response")

    required = ["STATE", "RPM_CMD", "RPM1", "FAULT", "LOCK", "ENABLE"]
    for key in required:
        if key not in response.fields:
            raise ProtocolError(f"missing status field: {key}")

    try:
        return Status(
            state=int(response.fields["STATE"]),
            rpm_cmd=int(response.fields["RPM_CMD"]),
            rpm1=int(response.fields["RPM1"]),
            fault=int(response.fields["FAULT"]),
            lock=int(response.fields["LOCK"]),
            enable=int(response.fields["ENABLE"]),
        )
    except ValueError as exc:
        raise ProtocolError("status field is not an integer") from exc


def run_params(profile: RunProfile) -> Dict[str, int]:
    return {
        "LIFT": int(profile.lift),
        "FINAL": int(profile.final),
        "SEAT": int(profile.seat),
        "HOLD": int(profile.hold),
        "RAMPUP": int(profile.rampup),
        "RAMPDOWN": int(profile.rampdown),
    }
