class CentrifugeError(Exception):
    """Base exception for centrifuge client errors."""


class ProtocolError(CentrifugeError):
    """Raised when data does not match protocol expectations."""


class DeviceError(CentrifugeError):
    """Raised when device returns ERR CODE=<...>."""

    def __init__(self, code: str):
        super().__init__(f"Device error: {code}")
        self.code = code


class CommandTimeoutError(CentrifugeError):
    """Raised when no matching response is received before timeout."""
