"""
Shared wire format for the cmd_vel UDP relay.

Mac → robot:  struct.pack('<fffI', x, y, rotate, expires_ms)
  x:          float32  linear.x (strafe)
  y:          float32  linear.y (forward/back)
  rotate:     float32  angular.z (rad/s)
  expires_ms: uint32   how long command is valid from time of receipt

Total: 16 bytes per datagram. UDP only — fire-and-forget.
"""
import socket
import struct

_FMT  = '<fffI'
_SIZE = struct.calcsize(_FMT)   # 16

RELAY_HOST = '10.42.0.1'
RELAY_PORT = 9999


def encode(x: float, y: float, rotate: float, expires_ms: int) -> bytes:
    return struct.pack(_FMT, float(x), float(y), float(rotate), int(expires_ms))


def decode(data: bytes):
    if len(data) < _SIZE:
        raise ValueError(f"short datagram: {len(data)} < {_SIZE}")
    return struct.unpack_from(_FMT, data)  # (x, y, rotate, expires_ms)


class RelayClient:
    """UDP sender — call send() at up to 60 Hz from bridge-api.py."""

    def __init__(self, host: str = RELAY_HOST, port: int = RELAY_PORT):
        self._addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, x: float, y: float, rotate: float, expires_ms: int = 200) -> None:
        expires_ms = max(0, min(int(expires_ms), 1000))
        try:
            self._sock.sendto(encode(x, y, rotate, expires_ms), self._addr)
        except OSError:
            pass  # fire-and-forget; next packet arrives in ~16 ms

    def close(self) -> None:
        self._sock.close()
