from pathlib import Path

import pytest

from swimtrack.config import Settings, load_settings
from swimtrack.main import Tracker

CONFIG = Path(__file__).resolve().parent.parent / "config.yaml"


def write(tmp_path, text):
    p = tmp_path / "c.yaml"
    p.write_text(text)
    return str(p)


def test_no_file_gives_defaults():
    s = load_settings(None)
    assert s.pool.length == 25.0 and s.uplink_url is None


def test_partial_file_keeps_other_defaults(tmp_path):
    s = load_settings(write(tmp_path, "anchor:\n  height: 4.0\n"))
    assert s.anchor.height == 4.0
    assert s.anchor.tilt_deg == 15.0 and s.anchor.y == pytest.approx(8.52)   # rest of the placeholder kept
    assert s.pool.lanes == 8


def test_empty_file(tmp_path):
    assert load_settings(write(tmp_path, "")) == Settings()


def test_unknown_setting_rejected(tmp_path):
    with pytest.raises(ValueError, match="hieght"):
        load_settings(write(tmp_path, "anchor:\n  hieght: 4.0\n"))


def test_unknown_section_rejected(tmp_path):
    with pytest.raises(ValueError, match="anchr"):
        load_settings(write(tmp_path, "anchr:\n  height: 4.0\n"))


@pytest.mark.skipif(not CONFIG.exists(), reason="pi/config.yaml not present")
def test_shipped_config_loads():
    s = load_settings(str(CONFIG))
    assert s.uplink_url.startswith("ws://")
    assert s.anchor.height > 0


def test_tracker_uses_settings(tmp_path):
    s = load_settings(write(tmp_path, "filters:\n  min_rssi: -60\nstrokes:\n  min_interval_s: 1.5\n"))
    tr = Tracker(settings=s)
    st = tr._state("T")
    assert st.angle_filter.cfg.min_rssi == -60
    assert st.detector.cfg.min_interval_s == 1.5


def test_no_config_uses_placeholder_mounting():
    """Running main.py without --config must use the same anchor as tools/sim.py."""
    tr = Tracker(settings=load_settings(None))
    assert (tr.anchor.y, tr.anchor.height, tr.anchor.tilt_deg) == pytest.approx((8.52, 2.5, 15.0))