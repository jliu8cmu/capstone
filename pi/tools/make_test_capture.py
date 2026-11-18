#!/usr/bin/env python3
"""Write a fake Pico capture (frames + noise + one corrupted frame + one gap) for read_frames.py.

Usage (from pi/):  python3 tools/make_test_capture.py test_capture.bin
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from swimtrack import packet as pk  # noqa: E402

rng = np.random.default_rng(0)
out = bytearray(b"\x00\x12\xAA\x99")
t = 0
for seq in range(0xFFF0, 0xFFF0 + 40):
    seq &= 0xFFFF
    n = 6 if seq % 4 == 0 else 5
    s = np.zeros((n, 6), dtype=np.int16)
    s[:, 2] = 4098 + rng.integers(-30, 30, n)
    s[:, 3:] = rng.integers(-15, 15, (n, 3))
    frame = pk.encode(pk.Packet(1, seq, t, s, 200, 0))
    t += 50
    if seq == 3:
        continue
    raw = bytearray(b"\xAA\x55" + bytes([len(frame)]) + frame)
    if seq == 0xFFFA:
        raw[10] ^= 0xFF
    out += raw
out += b"\xAA"
Path(sys.argv[1] if len(sys.argv) > 1 else "test_capture.bin").write_bytes(out)
print("wrote", len(out), "bytes")