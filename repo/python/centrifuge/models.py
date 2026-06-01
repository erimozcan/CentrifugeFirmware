from dataclasses import dataclass
from typing import Dict, Optional


@dataclass(frozen=True)
class RunProfile:
    lift: int
    final: int
    seat: int
    hold: int
    rampup: int
    rampdown: int


@dataclass(frozen=True)
class Status:
    state: int
    rpm_cmd: int
    rpm1: int
    fault: int
    lock: int
    enable: int


@dataclass(frozen=True)
class Response:
    seq: int
    ok: bool
    fields: Dict[str, str]
    error_code: Optional[str] = None
