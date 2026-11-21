import argparse
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, Iterator, Optional, Tuple

from swimtrack import packet as pk
from swimtrack import ublox
from swimtrack.strokes import SAMPLE_PERIOD_MS, StrokeDetector
from swimtrack.filters import AngleFilter

from swimtrack.ekf import SwimmerEKF
from swimtrack.geometry import AnchorConfig, PoolConfig, lane_of

from swimtrack.config import Settings, load_settings


from swimtrack.uplink import Uplink, build_message

from swimtrack.geometry import lane_of

from collections import deque
import numpy as np
from swimtrack.walls import WallDetector

REBOOT_BACKSTEP_MS = 1000
UPLINK_INTERVAL_S = 0.1
STALE_AFTER_S = 2.0


@dataclass
class TagState:
    detector: StrokeDetector = field(default_factory=StrokeDetector)
    last_seq: Optional[int] = None
    last_t0: int = 0
    last_ms: float = 0.0
    flags: int = 0
    packets: int = 0
    dups: int = 0
    gaps: int = 0
    lost: int = 0
    bad: int = 0
    reboots: int = 0
    angle: Optional[ublox.AngleReading] = None
    angle_events: int = 0
    missed_events: int = 0
    last_counter: Optional[int] = None
    angle_filter: AngleFilter = field(default_factory=AngleFilter)
    filtered: Optional[Tuple[float, float]]= None
    ekf: Optional[SwimmerEKF] = None
    last_seen: float = 0.0
    last_accel: Tuple[float, float, float] = (0.0,0.0,0.0)
    last_gyro: Tuple[float, float, float] = (0.0,0.0,0.0)
    uplink_seq: int = 0
    walls:  Optional[WallDetector] = None
    clock_offsets: deque = field(default_factory=lambda: deque(maxlen=50))
    length_strokes: list = field(default_factory=list)


class Tracker:
    def __init__(self, settings: Optional[Settings] = None):
        self.settings = settings or Settings()
        self.anchor = self.settings.anchor
        self.pool = self.settings.pool
        self.tags: Dict[str, TagState] = {}
        self.bad_lines = 0


    def _state(self, tag: str)->TagState:
        if tag not in self.tags:
            self.tags[tag] = TagState(detector = StrokeDetector(self.settings.strokes), angle_filter = AngleFilter(self.settings.filters), walls = WallDetector(self.pool, self.anchor, self.settings.walls))
        return self.tags[tag]


    def handle_line(self, line: str)->None:
        try:
            reading = ublox.parse_angle_line(line)
        except pk.PacketError:
            self.bad_lines += 1
            return
        if reading:
            self.handle_angle(reading)
            return
        payload = ublox.parse_payload_line(line)
        if payload:
            self.handle_payload(*payload)

    def handle_angle(self, r: ublox.AngleReading) ->None:
        st = self._state(r.tag)
        st.last_seen = time.monotonic()
        if st.last_counter is not None:
            step = (r.counter - st.last_counter) & 0xFFFF
            if step==0:
                return
            if step < 0x8000:
                st.missed_events += step -1
        st.last_counter = r.counter
        st.angle_events += 1
        st.angle = r

        smoothed = st.angle_filter.update(r)
        if smoothed:
            st.filtered = smoothed
            if st.ekf is None:
                st.ekf = SwimmerEKF(self.anchor, self.settings.ekf)
            st.ekf.update(*smoothed, r.timestamp_ms)
            if st.ekf.x is not None and st.clock_offsets:
                wearable_ms = r.timestamp_ms - float(np.median(st.clock_offsets))
                ev = st.walls.update_position(wearable_ms, st.ekf.x[0], st.ekf.x[2])
                if ev:
                    st.detector.discard_between(ev.suppress_from_ms, ev.suppress_to_ms)
                    st.length_strokes.append(st.walls.strokes_in_length(e.t_ms for e in st.detector.events))

    def handle_payload(self, tag: str, hex_payload: str) -> None:
        st = self._state(tag)
        try:
            p = ublox.decode_adv_hex(hex_payload)
        except pk.PacketError:
            st.bad +=1 
            return
        if not self._accept_seq(st,p):
            return
        st.packets += 1
        st.last_seen = time.monotonic()
        if p.n:
            st.last_accel = tuple(float(v) for v in p.accel_g[-1])
            st.last_gyro = tuple(float(v) for v in p.gyro_dps[-1])
        st.flags = p.flags
        st.last_t0 = p.t0_ms
        st.last_ms = p.t0_ms + p.n * SAMPLE_PERIOD_MS
        st.detector.update_packet(p)
        if p.n:
            st.walls.update_imu(p.t0_ms + np.arange(p.n) * SAMPLE_PERIOD_MS, p.accel_g)

        if st.angle is not None:
            st.clock_offsets.append(st.angle.timestamp_ms- st.last_ms)
        if st.walls.events:
            ev = st.walls.events[-1]
            if st.last_ms <= ev.suppress_to_ms + 5000:
                st.detector.discard_between(ev.suppress_from_ms, ev.suppress_to_ms)

    def _accept_seq(self, st: TagState, p: pk.Packet) -> bool:
        if st.last_seq is None:
            st.last_seq = p.seq
            return True
        step = (p.seq - st.last_seq) & 0xFFFF
        if step==0:
            st.dups += 1
            return False
        if step > 0x8000:
            if p.t0_ms < st.last_t0 - REBOOT_BACKSTEP_MS:
                st.reboots += 1
                st.detector = StrokeDetector(self.settings.strokes)
                st.walls = WallDetector(self.pool, self.anchor, self.settings.walls)
                st.clock_offsets.clear()
                st.length_strokes = []
                st.last_seq = p.seq
                return True
            st.dups += 1
            return False
        if step > 1:
            st.gaps += 1
            st.lost += step - 1
        st.last_seq = p.seq
        return True



    def uplink_messages(self) ->Iterator[dict]:
        now_mono, now_wall = time.monotonic(), time.time()
        for tag, st in sorted(self.tags.items()):
            if st.ekf is None or st.ekf.position is None or now_mono - st.last_seen > STALE_AFTER_S:
                continue
            x,y = st.ekf.position
            st.uplink_seq += 1
            yield build_message(tag, st.uplink_seq, now_wall, x,y,st.ekf.speed, lane_of(y, self.pool), st.detector.count, st.detector.rate_spm(st.last_ms), st.last_accel, st.last_gyro, lap = st.walls.lengths)


    def status_lines(self ) -> Iterator[str]:
        for tag, st in sorted(self.tags.items()):
            rate = st.detector.rate_spm(st.last_ms)
            rate_txt = f"{rate:4.0f}/min" if rate else "    -   "
            flag_txt = ",".join(n for b, n in ((pk.FLAG_IMU_ERR, "IMU"), (pk.FLAG_OVERRUN, "OVR"), (pk.FLAG_LOW_BATT, "LOW")) if st.flags & b) or "-"


            angle_txt = (f"az {st.angle.azimuth:+3d} el {st.angle.elevation:+3d} "
                         f"rssi {st.angle.rssi:4d} weak {st.angle_filter.rejected_weak}"
            ) if st.filtered else "no angles yet"

            if st.ekf and st.ekf.position:
                x,y = st.ekf.position
                lane = lane_of(y, self.pool)
                pos_txt = (f"x {x:5.1f} y {y:5.1f} lane {lane if lane else '-'} "
                           f"speed {st.ekf.speed:4.2f}")
            else:
                pos_txt = "no positon data"
            if time.monotonic() - st.last_seen > STALE_AFTER_S:
                pos_txt = "STALE " + pos_txt
            last_len = st.length_strokes[-1] if st.length_strokes else "-"
            yield (f"{tag} {pos_txt} strokes {st.detector.count:4d} rate {rate_txt} "
                   f"pkts {st.packets:6d} lost {st.lost:4d} dup {st.dups:4d} "
                   f"bad {st.bad:3d} flags {flag_txt} | {angle_txt} "
                   f"missed events {st.missed_events} "
                   f"lengths {st.walls.lengths} last_len_strokes {last_len} ")

            

def open_lines(source: str, baud:int) -> Iterator[str]:
    if source == "-":
        yield from sys.stdin
    elif source.startswith("/dev"):
        import serial
        with serial.Serial(source, baud, timeout = 0.5) as ser:
            while True:
                raw = ser.readline()
                if raw:
                    yield raw.decode("ascii", errors = "replace")

    else:
        with open(source) as f:
            yield from f

def main(argv = None) -> None:
    ap = argparse.ArgumentParser(description = __doc__, formatter_class = argparse.RawDescriptionHelpFormatter)
    ap.add_argument("source", help = "'-' for input")
    ap.add_argument("--baud", type=int, default = 115200)
    ap.add_argument("--log", help = "log raw liens")
    ap.add_argument("--uplink", help="dashboard WebSocket, e.g. ws://localhost:8000/ws/pool/1/")
    ap.add_argument("--config", help = "import config settings")
    args = ap.parse_args(argv)

    settings  = load_settings(args.config)
    tracker = Tracker(settings=settings)
    uplink_url = args.uplink or settings.uplink_url

    log = open(args.log, "a") if args.log else None

    uplink = Uplink(uplink_url) if uplink_url else None
    last_print = last_send = time.monotonic()

    try:
        for line in open_lines(args.source, args.baud):
            if log:
                log.write(line if line.endswith("\n") else line + "\n")

            tracker.handle_line(line)

            if uplink and time.monotonic() - last_send >= UPLINK_INTERVAL_S:
                for msg in tracker.uplink_messages():
                    uplink.send(msg)
                last_send = time.monotonic()

            if time.monotonic() - last_print >= 1.0 :
                print("\n".join(tracker.status_lines()), "\n", flush=True)
                last_print = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        if log:
            log.close()
        if uplink:
            uplink.close()
            print(f"uplink: sent {uplink.sent}, dropped {uplink.dropped}")
    print("Final:")
    print("\n".join(tracker.status_lines()))

if __name__ == "__main__":
    main()