import numpy as np
import pytest

from swimtrack.ekf import EkfConfig, SwimmerEKF
from swimtrack.filters import AngleFilter
from swimtrack.geometry import AnchorConfig, PoolConfig, position_to_angles
from swimtrack.ublox import AngleReading

POOL = PoolConfig()
ANCHOR = AnchorConfig(x=0.0, y=POOL.width / 2, height=2.5, yaw_deg=0.0, tilt_deg=15.0)
SPEED = 1.4


def lane_y(lane):
    return (lane - 0.5) * POOL.lane_width


def noisy_angles(x, y, rng, noise=1.0):
    az, el = position_to_angles(x, y, ANCHOR)
    return round(az + rng.normal(0, noise)), round(el + rng.normal(0, noise))


def swim(lane=4, seconds=15.0, drop=0.1, seed=0, skip=lambda t: False):
    """Straight swim away from the anchor.
    Returns (ekf, truth, est): truth rows are (t, x, y); est rows are (t, px, py, vx, vy)."""
    rng = np.random.default_rng(seed)
    ekf, filt, truth, est = SwimmerEKF(ANCHOR), AngleFilter(), [], []
    for k in range(int(seconds * 20)):
        t = k * 50
        x, y = 1.0 + SPEED * t / 1000, lane_y(lane)
        truth.append((t, x, y))
        if not (rng.random() < drop or skip(t)):
            az, el = noisy_angles(x, y, rng)
            smoothed = filt.update(AngleReading("T", -60, az, el, 0, "", t, k))
            ekf.update(*smoothed, t)
        state = ekf.state_at(t)
        est.append((t, *(state if state is not None else [np.nan] * 4)))
    return ekf, np.array(truth), np.array(est)


# Seen from an end-wall anchor, across-lane (y) error stays small, but along-lane (x)
# error grows with distance: 1 deg of elevation at 20 m is ~3 m of range at 2.5 m height.
# So the checks average over the last 5 s rather than trusting one instant.
@pytest.mark.parametrize("lane", [1, 4, 8])
@pytest.mark.parametrize("seed", range(5))
def test_tracks_straight_swim(lane, seed):
    _, truth, est = swim(lane=lane, seed=seed)
    last = truth[:, 0] >= truth[-1, 0] - 5000
    assert np.mean(np.abs(est[last, 2] - truth[last, 2])) < 0.5     # across lanes
    assert np.mean(np.abs(est[last, 1] - truth[last, 1])) < 2.0     # along the lane
    assert np.mean(est[last, 3]) == pytest.approx(SPEED, abs=0.3)
    assert abs(np.mean(est[last, 4])) < 0.2


def test_lateral_error_within_lane_spec():
    _, truth, est = swim(lane=2, seconds=20)
    settled = truth[:, 0] >= 3000
    assert np.percentile(np.abs(est[settled, 2] - truth[settled, 2]), 95) < 1.07


@pytest.mark.parametrize("seed", range(5))
def test_fills_gap_while_underwater(seed):
    _, truth, est = swim(seconds=12, seed=seed, skip=lambda t: 10_000 <= t < 11_000)
    gap = (truth[:, 0] >= 10_000) & (truth[:, 0] < 11_000)
    assert np.max(np.abs(est[gap, 2] - truth[gap, 2])) < 1.07     # still in the right lane
    assert np.max(np.abs(est[gap, 1] - truth[gap, 1])) < 3.0


def test_single_outlier_rejected():
    ekf, truth, _ = swim(seconds=10, drop=0.0)
    t = truth[-1, 0] + 50
    before = ekf.position
    assert ekf.update(60.0, 40.0, t) is False
    assert ekf.rejected == 1
    assert np.hypot(ekf.position[0] - before[0], ekf.position[1] - before[1]) < 0.5


def test_restarts_after_many_rejections():
    ekf, truth, _ = swim(seconds=8, drop=0.0)
    t = truth[-1, 0]
    az, el = position_to_angles(20.0, lane_y(1), ANCHOR)
    for i in range(EkfConfig().max_rejects + 1):
        ekf.update(az, el, t + 50 * (i + 1))
    assert ekf.restarts == 1
    assert ekf.position == pytest.approx((20.0, lane_y(1)), abs=0.5)


def test_restarts_after_long_silence():
    ekf, truth, _ = swim(seconds=5, drop=0.0)
    t = truth[-1, 0] + 6_000
    az, el = position_to_angles(3.0, lane_y(7), ANCHOR)
    assert ekf.update(az, el, t)
    assert ekf.position == pytest.approx((3.0, lane_y(7)), abs=1e-6)
    assert ekf.speed == 0.0


def test_no_start_when_ray_misses_water():
    ekf = SwimmerEKF(ANCHOR)
    assert ekf.update(0.0, 30.0, 0) is False
    assert ekf.position is None


def test_state_at_does_not_change_filter():
    ekf, truth, _ = swim(seconds=5, drop=0.0)
    x0 = ekf.x.copy()
    ekf.state_at(truth[-1, 0] + 2_000)
    assert np.array_equal(ekf.x, x0)


def test_predict_moves_with_velocity():
    ekf, truth, _ = swim(seconds=10, drop=0.0)
    t = truth[-1, 0]
    now, later = ekf.state_at(t), ekf.state_at(t + 1_000)
    assert later[0] - now[0] == pytest.approx(now[2], rel=1e-9)