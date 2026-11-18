import numpy as np
import pytest

from swimtrack.geometry import (AnchorConfig, PoolConfig, angles_to_position, in_pool,
                                lane_of, position_to_angles)

POOL = PoolConfig()
# Anchor on the middle of the near end wall (x = 0), facing down the pool.
MID_END = AnchorConfig(x=0.0, y=POOL.width / 2, height=2.5, yaw_deg=0.0, tilt_deg=15.0)
# Anchor on a corner, angled diagonally across the pool.
CORNER = AnchorConfig(x=0.0, y=0.0, height=3.0, yaw_deg=35.0, tilt_deg=10.0)


@pytest.mark.parametrize("cfg", [MID_END, CORNER])
def test_round_trip_over_pool(cfg):
    for x in np.linspace(1, POOL.length, 9):
        for y in np.linspace(0.5, POOL.width - 0.5, 7):
            az, el = position_to_angles(x, y, cfg)
            got = angles_to_position(az, el, cfg)
            assert got is not None
            assert got == pytest.approx((x, y), abs=1e-6)


def test_straight_ahead_is_zero_azimuth():
    az, _ = position_to_angles(10.0, MID_END.y, MID_END)
    assert az == pytest.approx(0.0, abs=1e-9)


def test_elevation_is_relative_to_tilted_boresight():
    # Boresight (15 deg down from 2.5 m) hits the water at 2.5 / tan(15 deg) = 9.33 m.
    hit = MID_END.height / np.tan(np.radians(MID_END.tilt_deg))
    assert position_to_angles(hit, MID_END.y, MID_END)[1] == pytest.approx(0.0, abs=1e-9)
    assert position_to_angles(5.0, MID_END.y, MID_END)[1] < 0
    assert position_to_angles(20.0, MID_END.y, MID_END)[1] > 0


def test_left_of_boresight_is_positive_azimuth():
    az, _ = position_to_angles(10.0, MID_END.y + 3, MID_END)
    assert az > 0


def test_az_sign_flip():
    flipped = AnchorConfig(**{**MID_END.__dict__, "az_sign": -1.0})
    az, _ = position_to_angles(10.0, MID_END.y + 3, flipped)
    assert az < 0
    assert angles_to_position(az, position_to_angles(10.0, MID_END.y + 3, flipped)[1],
                              flipped) == pytest.approx((10.0, MID_END.y + 3))


def test_offsets_round_trip():
    cfg = AnchorConfig(**{**CORNER.__dict__, "az_offset_deg": 2.0, "el_offset_deg": -1.5})
    az, el = position_to_angles(12.0, 6.0, cfg)
    assert angles_to_position(az, el, cfg) == pytest.approx((12.0, 6.0))


def test_ray_above_horizon_returns_none():
    assert angles_to_position(0.0, 16.0, MID_END) is None


def test_near_horizontal_ray_beyond_range_returns_none():
    # 14 deg above a 15 deg-down boresight = 1 deg below horizontal: hits ~143 m away.
    assert angles_to_position(0.0, 14.0, MID_END) is None


def test_angle_error_meets_lane_spec_at_far_wall():
    """Design report: 2.45 deg of azimuth error must stay under 1.07 m at 25 m."""
    x, y = POOL.length, MID_END.y
    az, el = position_to_angles(x, y, MID_END)
    p = angles_to_position(az + 2.45, el, MID_END)
    assert np.hypot(p[0] - x, p[1] - y) <= 1.07 + 0.01


@pytest.mark.parametrize("y,lane", [(0.0, 1), (2.12, 1), (2.14, 2), (POOL.width - 0.01, 8)])
def test_lane_of(y, lane):
    assert lane_of(y, POOL) == lane


@pytest.mark.parametrize("y", [-0.1, POOL.width])
def test_lane_of_outside(y):
    assert lane_of(y, POOL) is None


def test_in_pool():
    assert in_pool(10, 5, POOL)
    assert not in_pool(-1, 5, POOL)
    assert in_pool(-0.5, 5, POOL, margin=1.0)
