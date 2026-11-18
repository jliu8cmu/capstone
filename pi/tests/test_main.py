import numpy as np
import pytest

from swimtrack import packet as pk
from swimtrack.main import Tracker


def adv_hex(seq, t0=None, n=5, flags=0):
    p = pk.Packet(1, seq & 0xFFFF, t0 if t0 is not None else seq * 50,
                  np.zeros((n, 6), np.int16), 200, flags)
    body = bytes([0xFF, 0xFF, 0xFF]) + pk.encode(p)
    return (bytes([len(body)]) + body).hex()


def test_in_order_packets():
    tr = Tracker()
    for s in range(10):
        tr.handle_payload("A", adv_hex(s))
    st = tr.tags["A"]
    assert (st.packets, st.dups, st.lost) == (10, 0, 0)


def test_duplicate_dropped():
    tr = Tracker()
    for s in (0, 1, 1, 2):
        tr.handle_payload("A", adv_hex(s))
    st = tr.tags["A"]
    assert (st.packets, st.dups) == (3, 1)


def test_gap_counted_across_wrap():
    tr = Tracker()
    for s in (65534, 65535, 2):
        tr.handle_payload("A", adv_hex(s, t0=1_000_000 + (s % 65536) * 50))
    st = tr.tags["A"]
    assert (st.gaps, st.lost) == (1, 2)


def test_late_old_packet_ignored():
    tr = Tracker()
    for s in (10, 11, 9):
        tr.handle_payload("A", adv_hex(s))
    assert tr.tags["A"].packets == 2


def test_reboot_restarts_tag():
    tr = Tracker()
    for s in range(100, 110):
        tr.handle_payload("A", adv_hex(s, t0=60_000 + s * 50))
    tr.handle_payload("A", adv_hex(0, t0=100))
    st = tr.tags["A"]
    assert st.reboots == 1 and st.packets == 11 and st.last_seq == 0


def test_bad_payload_counted():
    tr = Tracker()
    tr.handle_payload("A", "zz")
    tr.handle_payload("A", adv_hex(0)[:-2] + "00")
    assert tr.tags["A"].bad == 2


def test_tags_are_independent():
    tr = Tracker()
    tr.handle_payload("A", adv_hex(5))
    tr.handle_payload("B", adv_hex(5))
    assert tr.tags["A"].packets == tr.tags["B"].packets == 1


def test_flags_kept():
    tr = Tracker()
    tr.handle_payload("A", adv_hex(0, flags=pk.FLAG_IMU_ERR))
    assert tr.tags["A"].flags == pk.FLAG_IMU_ERR
    assert "IMU" in next(tr.status_lines())


def angle_line(tag, counter, az=-12, el=-1, rssi=-71):
    return f'+UUDF:{tag},{rssi},{az},{el},0,11,"80A197B210EC","",123456,{counter}\r\n'


def test_angle_line_stored():
    tr = Tracker()
    tr.handle_line(angle_line("000000000001", 10, az=5, el=-3))
    a = tr.tags["000000000001"].angle
    assert (a.azimuth, a.elevation) == (5, -3)


def test_missed_events_counted_across_wrap():
    tr = Tracker()
    for c in (65534, 65535, 2):
        tr.handle_line(angle_line("T", c))
    assert tr.tags["T"].missed_events == 2


def test_repeated_angle_event_ignored():
    tr = Tracker()
    tr.handle_line(angle_line("T", 5))
    tr.handle_line(angle_line("T", 5))
    assert tr.tags["T"].angle_events == 1


def test_angle_then_payload_same_tag():
    tr = Tracker()
    tr.handle_line(angle_line("T", 1))
    tr.handle_line(f"+UUDFP:T,{adv_hex(0)}\r\n")
    st = tr.tags["T"]
    assert st.angle_events == 1 and st.packets == 1


def test_malformed_angle_line_counted():
    tr = Tracker()
    tr.handle_line("+UUDF:T,-71")
    assert tr.bad_lines == 1 and not tr.tags


def test_other_lines_ignored():
    tr = Tracker()
    for line in ("OK", "", "AT"):
        tr.handle_line(line)
    assert not tr.tags and tr.bad_lines == 0


def test_angles_are_filtered():
    tr = Tracker()
    for c, az in enumerate((10, 10, 70)):
        tr.handle_line(angle_line("T", c, az=az))
    assert tr.tags["T"].filtered[0] == 10.0


def test_weak_angle_not_used():
    tr = Tracker()
    tr.handle_line(angle_line("T", 1, az=10))
    tr.handle_line(angle_line("T", 2, az=50, rssi=-95))
    st = tr.tags["T"]
    assert st.filtered[0] == 10.0 and st.angle_filter.rejected_weak == 1


def test_position_tracked_from_angles():
    from swimtrack.geometry import position_to_angles
    tr = Tracker()
    for c in range(40):
        x, y = 5.0 + 0.07 * c, (3 - 0.5) * tr.pool.lane_width
        az, el = position_to_angles(x, y, tr.anchor)
        tr.handle_line(angle_line("T", c, az=round(az), el=round(el)).replace(",123456,", f",{c * 50},"))
    px, py = tr.tags["T"].ekf.position
    assert abs(py - y) < 0.5 and abs(px - x) < 1.0
    assert "lane 3" in next(tr.status_lines())