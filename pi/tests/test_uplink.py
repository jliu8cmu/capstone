import json
import threading
import time

import pytest
from websockets.sync.server import serve

from swimtrack.uplink import Uplink, build_message

ARGS = dict(tag="000000000001", seq=7, timestamp=1700000000.123456, x=12.345, y=3.21,
            speed=1.456, lane=2, stroke_count=14, stroke_rate=41.27,
            accel_g=(0.1234, -0.98765, 0.05), gyro_dps=(12.345, -6.789, 0.0))


def test_message_matches_web_app_format():
    msg = build_message(**ARGS)
    assert msg["type"] == "telemetry_packet"
    d = msg["data"]
    assert d["tag_id"] == "000000000001" and d["sequence_id"] == 7 and d["lane"] == 2
    assert (d["x_m"], d["y_m"], d["speed_mps"]) == (12.35, 3.21, 1.46)
    assert (d["stroke_count"], d["stroke_rate_spm"], d["phase"]) == (14, 41.3, "SURFACE_SWIM")
    assert d["accel"] == {"x": 0.123, "y": -0.988, "z": 0.05}
    assert d["gyro"] == {"roll": 12.35, "pitch": -6.79, "yaw": 0.0}
    json.dumps(msg)


def test_no_rate_and_no_lane():
    d = build_message(**{**ARGS, "stroke_rate": None, "lane": None})["data"]
    assert d["stroke_rate_spm"] == 0.0 and d["phase"] == "" and "lane" not in d


@pytest.fixture
def server():
    received = []

    def handler(ws):
        for raw in ws:
            received.append(json.loads(raw))
            ws.send(json.dumps({"type": "telemetry_packet", "data": {}}))   # echo like Channels

    with serve(handler, "127.0.0.1", 0) as srv:
        port = srv.socket.getsockname()[1]
        t = threading.Thread(target=srv.serve_forever, daemon=True)
        t.start()
        yield f"ws://127.0.0.1:{port}/ws/pool/1/", received
        srv.shutdown()


def wait_for(cond, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end and not cond():
        time.sleep(0.01)
    return cond()


def test_sends_to_server(server):
    url, received = server
    up = Uplink(url)
    for i in range(50):
        assert up.send(build_message(**{**ARGS, "seq": i}))
    assert wait_for(lambda: len(received) == 50)
    assert [m["data"]["sequence_id"] for m in received] == list(range(50))
    up.close()


def test_offline_drops_without_blocking():
    up = Uplink("ws://127.0.0.1:9/ws/pool/1/", retry_s=10)
    t = time.monotonic()
    for _ in range(20):
        assert up.send(build_message(**ARGS)) is False
    assert up.dropped == 20
    assert time.monotonic() - t < 3.0


def test_reconnects_after_server_restart(server):
    url, received = server
    up = Uplink(url, retry_s=0)
    assert up.send(build_message(**ARGS))
    up.ws.close()                       # simulate the link dropping
    up.send(build_message(**ARGS))      # fails and resets
    assert up.send(build_message(**ARGS))
    assert wait_for(lambda: len(received) >= 2)
    up.close()