import json
import time
from contextlib import ExitStack
from typing import Optional, Sequence

from websockets.exceptions import WebSocketException

from websockets.sync.client import connect


def build_message(tag: str, seq: int, timestamp: float, x: float, y: float, speed: float,
                  lane: Optional[int], stroke_count: int, stroke_rate: Optional[float],
                  accel_g: Sequence[float], gyro_dps: Sequence[float]) -> dict:
    """One telemetry_packet in the format of the web app's mock_telemetry_generator."""
    data = {
        "tag_id": tag,
        "sequence_id": seq,
        "timestamp": round(timestamp, 4),
        "phase": "SURFACE_SWIM" if stroke_rate else "",
        "x_m": round(x, 2),
        "y_m": round(y, 2),
        "speed_mps": round(speed, 2),
        "stroke_count": stroke_count,
        "stroke_rate_spm": round(stroke_rate, 1) if stroke_rate else 0.0,
        "accel": {"x": round(accel_g[0], 3), "y": round(accel_g[1], 3), "z": round(accel_g[2], 3)},
        "gyro": {"roll": round(gyro_dps[0], 2), "pitch": round(gyro_dps[1], 2),
                 "yaw": round(gyro_dps[2], 2)},
    }
    if lane is not None:
        data["lane"] = lane
    return {"type": "telemetry_packet", "data": data}

class Uplink:
    """Best-effort sender: reconnects in the background and drops data while offline."""

    def __init__(self, url: str, retry_s: float = 2.0):
        self.url = url
        self.retry_s = retry_s
        self.ws = None
        self._stack = ExitStack()
        self.next_try = 0.0
        self.sent = 0
        self.dropped = 0


    def _ensure_connected(self) -> bool:
        if self.ws is not None:
            return True
        now = time.monotonic()
        if now < self.next_try:
            return False
        self.next_try = now + self.retry_s
        try:
            self.ws = self._stack.enter_context(connect(self.url, open_timeout=2))
            return True
        except (OSError, TimeoutError, WebSocketException):
            return False

    def _drain(self) -> None:
        # The server broadcasts every packet back to all clients, us included.
        # Read and discard them so they don't pile up and stall the connection.
        while True:
            try:
                self.ws.recv(timeout=0)
            except TimeoutError:
                return


    def send(self, message: dict) -> bool:
        if not self._ensure_connected():
            self.dropped += 1
            return False
        try:
            self._drain()
            self.ws.send(json.dumps(message))
            self.sent += 1
            return True
        except (OSError, WebSocketException):
            self.close()
            self.dropped += 1
            return False

    def close(self) -> None:
        try:
            if self.ws is not None:
                self._drain()
            self._stack.close()
        except (OSError, WebSocketException):
            pass
        self.ws = None