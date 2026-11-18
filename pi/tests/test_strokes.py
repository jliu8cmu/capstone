from pathlib import Path

import numpy as np
import pytest

from swimtrack import packet as pk
from swimtrack.strokes import SAMPLE_PERIOD_MS, StrokeConfig, StrokeDetector, StrokeEvent

FS = 104.0
PI_DIR = Path(__file__).resolve().parent.parent
DATA_DIRS = [PI_DIR / "data", PI_DIR.parent / "data"]      # pi/data or Capstone/data


def find_data(name):
    return next((d / name for d in DATA_DIRS if (d / name).exists()), None)


def synth(rate_hz, dur_s, amp=250.0, harmonic=0.4, noise=20.0, seed=0):
    """Fake wrist gyro: the arm swings back and forth about one (random) axis each cycle,
    with a second harmonic and noise, like real strokes."""
    rng = np.random.default_rng(seed)
    t = np.arange(int(dur_s * FS)) / FS
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)
    swing = (amp * np.sin(2 * np.pi * rate_hz * t)
             + harmonic * amp * np.sin(4 * np.pi * rate_hz * t + 1.0))
    return t * 1000.0, np.outer(swing, axis) + rng.normal(0, noise, (len(t), 3))


def run(det, t, g, chunk=6):
    for i in range(0, len(t), chunk):
        det.update(t[i:i + chunk], g[i:i + chunk])
    return det


def test_rest_has_no_strokes():
    rng = np.random.default_rng(1)
    t = np.arange(int(20 * FS)) / FS * 1000
    g = rng.normal(0, 3, (len(t), 3))
    assert run(StrokeDetector(), t, g).count == 0


@pytest.mark.parametrize("rate_hz", [0.3, 0.5, 0.7, 1.0, 1.2])
def test_counts_synthetic_strokes(rate_hz):
    dur = 20.0
    det = run(StrokeDetector(), *synth(rate_hz, dur))
    assert abs(det.count - rate_hz * dur) <= 1


def test_rate_spm():
    t, g = synth(0.7, 20.0)
    det = run(StrokeDetector(), t, g)
    assert det.rate_spm(t[-1]) == pytest.approx(42.0, rel=0.05)


def test_rate_goes_stale():
    t, g = synth(0.7, 10.0)
    det = run(StrokeDetector(), t, g)
    assert det.rate_spm(t[-1] + 10_000) is None


def test_chunk_size_does_not_matter():
    t, g = synth(0.8, 15.0)
    a = [e.t_ms for e in run(StrokeDetector(), t, g, chunk=1).events]
    b = [e.t_ms for e in run(StrokeDetector(), t, g, chunk=500).events]
    assert a == b


def test_update_packet():
    t, g = synth(0.7, 10.0)
    counts = np.round(g / pk.GYR_DPS_PER_COUNT).astype(np.int16)
    samples = np.zeros((len(t), 6), np.int16)
    samples[:, 3:] = counts
    det = StrokeDetector()
    for seq, i in enumerate(range(0, len(t) - 5, 5)):
        p = pk.Packet(1, seq, int(i * SAMPLE_PERIOD_MS), samples[i:i + 5], 200, 0)
        det.update_packet(p)
    assert abs(det.count - 7) <= 1


@pytest.mark.skipif(find_data("session7.csv") is None, reason="session7.csv not found")
def test_session7_dryland_counts():
    """Session 7: 10 each of free/back/breast/fly, separated by tap markers."""
    pd = pytest.importorskip("pandas")
    d = pd.read_csv(find_data("session7.csv"))
    t = d.t_ms.to_numpy()
    g = d[["gx_dps", "gy_dps", "gz_dps"]].to_numpy()
    det = run(StrokeDetector(), t, g, chunk=5)
    ts = (np.array([e.t_ms for e in det.events]) - t[0]) / 1000
    segments = {"free": (10.9, 30.2), "back": (32.5, 50.3),
                "breast": (52.3, 68.3), "fly": (71.2, 88.9)}
    for name, (s, e) in segments.items():
        n = int(((ts >= s) & (ts <= e)).sum())
        assert abs(n - 10) <= 2, f"{name}: {n}"


# First pool session: 4 lengths of IM, then 4 free, 4 back, 4 breast (one arm's cycles,
# counted per length by the swimmer). Block times are seconds from the start of the file.
POOL_BLOCKS = {"IM": (14, 106, 5 + 9 + 10 + 8), "free": (185, 281, 32),
               "back": (318, 417, 6 + 7 + 7 + 9), "breast": (540, 662, 11 + 12 + 12 + 12)}


@pytest.mark.skipif(find_data("real_session_1.bin") is None, reason="real_session_1.bin not found")
def test_real_pool_session_counts():
    import sys
    sys.path.insert(0, str(PI_DIR / "tools"))
    from read_frames import FrameParser
    packets = FrameParser().feed(find_data("real_session_1.bin").read_bytes())
    det = StrokeDetector()
    for p in packets:
        det.update_packet(p)
    t0 = packets[0].t0_ms
    ts = (np.array([e.t_ms for e in det.events]) - t0) / 1000
    total_found = total_true = 0
    for name, (start, end, truth) in POOL_BLOCKS.items():
        found = int(((ts >= start) & (ts <= end)).sum())
        assert abs(found - truth) <= 0.2 * truth + 1, f"{name}: {found} vs {truth}"
        total_found += found
        total_true += truth
    assert abs(total_found - total_true) <= 0.2 * total_true


def test_isolated_detections_give_no_rate():
    det = StrokeDetector()
    det.events = [StrokeEvent(t, 200.0) for t in (0.0, 10_000.0, 12_500.0 + 6_000)]
    assert det.rate_spm(18_600.0) is None


def test_rate_ignores_one_long_gap():
    det = StrokeDetector()
    det.events = [StrokeEvent(t, 200.0) for t in (0.0, 2_000.0, 4_000.0, 12_000.0, 14_000.0)]
    assert det.rate_spm(14_100.0) == pytest.approx(30.0)