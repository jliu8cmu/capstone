import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from swimtrack.config import default_anchor
from swimtrack.geometry import PoolConfig
from swimtrack.main import Tracker
from swimtrack.strokes import StrokeDetector, StrokeEvent
from swimtrack.walls import WallConfig, WallDetector

PI_DIR = Path(__file__).resolve().parent.parent
DATA_DIRS = [PI_DIR / "data", PI_DIR.parent / "data"]
POOL = PoolConfig()


def find_data(name):
    return next((d / name for d in DATA_DIRS if (d / name).exists()), None)


def detector():
    return WallDetector(POOL, default_anchor())


def swim(det, legs, speed=1.4, start_x=1.0, dt_ms=50, noise=0.0, seed=0):
    rng = np.random.default_rng(seed)
    x, t, events = start_x, 0.0, []
    for target in legs:
        direction = np.sign(target - x)
        while (target - x) * direction > 0:
            x += direction * speed * dt_ms / 1000
            t += dt_ms
            ev = det.update_position(t, x + rng.normal(0, noise), direction * speed)
            if ev:
                events.append(ev)
    return events, t


def test_detects_each_turn_and_alternates_ends():
    events, _ = swim(detector(), [24.0, 1.0, 24.0, 1.0])
    assert [e.end for e in events] == [1, 0, 1]


def test_touch_time_is_closest_approach():
    events, _ = swim(detector(), [24.0, 1.0])
    expected = (24.0 - 1.0) / 1.4 * 1000
    assert events[0].touch_ms == pytest.approx(expected, abs=100)


def test_noisy_position_still_one_event_per_turn():
    events, _ = swim(detector(), [24.0, 1.0, 24.0, 1.0, 24.0], noise=0.5, seed=3)
    assert len(events) == 4


def test_turning_back_mid_pool_is_not_a_wall():
    events, _ = swim(detector(), [12.0, 5.0, 24.0])
    assert events == []


def test_min_length_time_blocks_double_events():
    det = WallDetector(POOL, default_anchor(), WallConfig(min_length_s=30.0))
    events, _ = swim(det, [24.0, 1.0, 24.0])
    assert len(events) == 1


def test_pushoff_found_after_touch():
    det = detector()
    touch = (24.0 - 1.0) / 1.4 * 1000
    det.update_imu([touch + 800], [[2.5, 0.0, 0.0]])
    events, _ = swim(det, [24.0, 1.0])
    ev = events[0]
    assert ev.pushoff_ms == pytest.approx(touch + 800, abs=100)
    assert ev.suppress_to_ms == pytest.approx(ev.pushoff_ms + 1000)


def test_pushoff_arriving_after_the_event_is_still_used():
    det = detector()
    events, t_end = swim(det, [24.0, 1.0])
    ev = events[0]
    assert ev.pushoff_ms is None
    det.update_imu([ev.touch_ms + 2500], [[0.0, 3.0, 0.0]])
    assert ev.pushoff_ms == pytest.approx(ev.touch_ms + 2500)
    assert ev.suppress_to_ms == pytest.approx(ev.touch_ms + 3500)


def test_pushoff_outside_search_window_ignored():
    det = detector()
    events, _ = swim(det, [24.0, 1.0])
    ev = events[0]
    det.update_imu([ev.touch_ms + 3500], [[3.0, 0.0, 0.0]])
    assert ev.pushoff_ms is None


def test_no_pushoff_uses_default_window():
    events, _ = swim(detector(), [24.0, 1.0])
    ev = events[0]
    assert ev.pushoff_ms is None
    assert ev.suppress_to_ms - ev.touch_ms == pytest.approx(2500)
    assert ev.touch_ms - ev.suppress_from_ms == pytest.approx(500)


def test_far_wall_zone_is_wider():
    det = detector()
    assert det.zones == (3.0, 4.5)


def test_strokes_in_length_uses_windows_not_running_totals():
    det = detector()
    events, _ = swim(det, [24.0, 1.0, 24.0])
    first, second = events
    strokes = [first.suppress_from_ms - 3000, first.suppress_from_ms - 1000,
               first.touch_ms + 100,
               first.suppress_to_ms + 1000, first.suppress_to_ms + 3000, second.suppress_from_ms - 500,
               second.touch_ms]
    assert det.strokes_in_length(strokes) == 3
    det.events = [first]
    assert det.strokes_in_length(strokes) == 2


def test_discard_between():
    d = StrokeDetector()
    d.events = [StrokeEvent(t, 200.0) for t in (1000.0, 2000.0, 3000.0, 4000.0)]
    assert d.discard_between(1500, 3000) == 2
    assert [e.t_ms for e in d.events] == [1000.0, 4000.0]


@pytest.mark.skipif(find_data("session7.bin") is None, reason="session7.bin not found")
def test_pipeline_counts_simulated_lengths():
    lines = subprocess.run([sys.executable, str(PI_DIR / "tools" / "sim.py"), str(find_data("session7.bin")),
                            "--speed", "0"], capture_output=True, text=True, check=True).stdout
    tr = Tracker()
    for line in lines.splitlines():
        tr.handle_line(line)
    st = tr.tags["000000000001"]
    assert st.walls.lengths == 5
    assert [e.end for e in st.walls.events] == [1, 0, 1, 0, 1]
    assert len(st.length_strokes) == 5
    assert sum(st.length_strokes) <= st.detector.count