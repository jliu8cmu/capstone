import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
from read_frames import FrameParser, StreamStats  # noqa: E402
from swimtrack import packet as pk  # noqa: E402


def frame(seq, n=5):
    p = pk.Packet(1, seq, 0, np.zeros((n, 6), np.int16), 0, 0)
    body = pk.encode(p)
    return b"\xAA\x55" + bytes([len(body)]) + body


def test_byte_by_byte():
    data = b"\x00\xAA" + frame(1) + frame(2)
    fp = FrameParser()
    got = [p for b in data for p in fp.feed(bytes([b]))]
    assert [p.seq for p in got] == [1, 2]


def test_resync_after_corruption():
    bad = bytearray(frame(1)); bad[12] ^= 0xFF
    fp = FrameParser()
    got = fp.feed(bytes(bad) + frame(2))
    assert [p.seq for p in got] == [2]
    assert fp.crc_errors == 1


def test_bad_len_byte():
    fp = FrameParser()
    got = fp.feed(b"\xAA\x55\x05" + frame(7))
    assert [p.seq for p in got] == [7]
    assert fp.format_errors == 1


def test_gap_and_wrap():
    st = StreamStats()
    for s in (65534, 65535, 0, 2):
        st.update(pk.decode(frame(s)[3:]))
    assert st.gaps == 1 and st.lost == 1