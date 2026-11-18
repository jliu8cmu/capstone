import pytest

from swimtrack.filters import AngleFilter, FilterConfig
from swimtrack.ublox import AngleReading


def reading(az, el, rssi=-60, ts=0):
    return AngleReading("T", rssi, az, el, 11, "A", ts, 0)


def feed(f, values, step_ms=50):
    return [f.update(reading(az, el, ts=i * step_ms)) for i, (az, el) in enumerate(values)]


def test_first_reading_passes_through():
    assert AngleFilter().update(reading(10, -5)) == (10.0, -5.0)


def test_median_removes_single_spike():
    out = feed(AngleFilter(), [(10, 0), (10, 0), (60, 40), (11, 1), (11, 1)])
    assert out[2] == (10.0, 0.0)
    assert out[3] == (11.0, 1.0)


def test_median_follows_a_real_change_after_two_samples():
    out = feed(AngleFilter(), [(0, 0), (0, 0), (20, 0), (20, 0)])
    assert out[2] == (0.0, 0.0)
    assert out[3] == (20.0, 0.0)


def test_weak_rssi_rejected_and_counted():
    f = AngleFilter()
    assert f.update(reading(10, 0, rssi=-90)) is None
    assert f.rejected_weak == 1
    assert len(f.window) == 0


def test_threshold_is_inclusive():
    assert AngleFilter().update(reading(10, 0, rssi=-85)) is not None


def test_long_gap_clears_window():
    f = AngleFilter()
    f.update(reading(50, 0, ts=0))
    f.update(reading(50, 0, ts=50))
    assert f.update(reading(-20, 0, ts=5000)) == (-20.0, 0.0)


def test_custom_window_length():
    f = AngleFilter(FilterConfig(median_len=5))
    out = feed(f, [(0, 0), (0, 0), (0, 0), (90, 90), (90, 90), (0, 0)])
    assert out[4] == (0.0, 0.0)
    assert out[5] == (0.0, 0.0)


def test_elevation_filtered_independently():
    out = feed(AngleFilter(), [(0, 5), (0, -40), (0, 6)])
    assert out[2] == (0.0, 5.0)