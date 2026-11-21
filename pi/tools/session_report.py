#!/usr/bin/env python3
import argparse
import json
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from read_frames import FrameParser
from swimtrack import packet as pk
from swimtrack import ublox
from swimtrack.config import load_settings
from swimtrack.geometry import lane_of
from swimtrack.main import Tracker

FLAG_NAMES = ((pk.FLAG_IMU_ERR, "imu_err"), (pk.FLAG_OVERRUN, "overrun"), (pk.FLAG_LOW_BATT, "low_batt"))


class TagLog:
    def __init__(self):
        self.ts = []
        self.rssi = []
        self.az = []
        self.el = []
        self.lanes = []
        self.xs = []
        self.speeds = []
        self.seqs = set()
        self.samples = 0
        self.flags = Counter()
        self.batt = []


def collect(lines, tracker):
    logs = defaultdict(TagLog)
    for line in lines:
        tracker.handle_line(line)
        try:
            r = ublox.parse_angle_line(line)
        except pk.PacketError:
            continue
        if r:
            log = logs[r.tag]
            log.ts.append(r.timestamp_ms)
            log.rssi.append(r.rssi)
            log.az.append(r.azimuth)
            log.el.append(r.elevation)
            st = tracker.tags.get(r.tag)
            if st and st.ekf and st.ekf.position:
                x, y = st.ekf.position
                log.xs.append(x)
                log.lanes.append(lane_of(y, tracker.pool))
                log.speeds.append(st.ekf.speed)
            continue
        payload = ublox.parse_payload_line(line)
        if not payload:
            continue
        tag, hex_part = payload
        try:
            p = ublox.decode_adv_hex(hex_part)
        except pk.PacketError:
            continue
        log = logs[tag]
        if p.seq not in log.seqs:
            log.seqs.add(p.seq)
            log.samples += p.n
            log.batt.append(p.batt_mv)
            for bit, name in FLAG_NAMES:
                if p.flags & bit:
                    log.flags[name] += 1
    return logs


def flash_seqs(path):
    packets = FrameParser().feed(Path(path).read_bytes())
    return [p.seq for p in packets], sum(p.n for p in packets)


def pct(a, b):
    return round(100.0 * a / b, 2) if b else None


def lane_changes(lanes):
    known = [l for l in lanes if l is not None]
    return sum(1 for a, b in zip(known, known[1:]) if a != b)


def tag_report(tag, st, log, flash):
    duration_s = (log.ts[-1] - log.ts[0]) / 1000 if len(log.ts) > 1 else 0.0
    expected_events = st.angle_events + st.missed_events
    received_pkts = st.packets
    expected_pkts = st.packets + st.lost
    lanes = Counter(l for l in log.lanes if l is not None)
    total_lane = sum(lanes.values())
    main_lane, main_count = lanes.most_common(1)[0] if lanes else (None, 0)
    rate = st.detector.rate_spm(st.last_ms)
    rep = {
        "tag": tag,
        "duration_s": round(duration_s, 1),
        "angle_events": st.angle_events,
        "missed_events": st.missed_events,
        "event_loss_pct": pct(st.missed_events, expected_events),
        "event_rate_hz": round(st.angle_events / duration_s, 2) if duration_s else None,
        "weak_rejected": st.angle_filter.rejected_weak,
        "rssi_mean": round(float(np.mean(log.rssi)), 1) if log.rssi else None,
        "rssi_min": min(log.rssi) if log.rssi else None,
        "rssi_max": max(log.rssi) if log.rssi else None,
        "az_range": [min(log.az), max(log.az)] if log.az else None,
        "el_range": [min(log.el), max(log.el)] if log.el else None,
        "packets_received": received_pkts,
        "packets_lost": st.lost,
        "packet_loss_pct": pct(st.lost, expected_pkts),
        "duplicates": st.dups,
        "bad_payloads": st.bad,
        "reboots": st.reboots,
        "imu_samples": log.samples,
        "imu_rate_hz": round(log.samples / duration_s, 1) if duration_s else None,
        "flags": dict(log.flags),
        "batt_mv_first_last": [log.batt[0], log.batt[-1]] if log.batt else None,
        "strokes": st.detector.count,
        "last_stroke_rate_spm": round(rate, 1) if rate else None,
        "main_lane": main_lane,
        "time_in_main_lane_pct": pct(main_count, total_lane),
        "lane_distribution": {str(k): v for k, v in sorted(lanes.items())},
        "lane_changes": lane_changes(log.lanes),
        "x_range_m": [round(min(log.xs), 1), round(max(log.xs), 1)] if log.xs else None,
        "mean_speed_mps": round(float(np.mean(log.speeds)), 2) if log.speeds else None,
    }
    if flash is not None:
        seqs, flash_samples = flash
        sent = set(seqs)
        delivered = len(sent & log.seqs)
        rep["flash_packets"] = len(seqs)
        rep["flash_samples"] = flash_samples
        rep["end_to_end_delivered"] = delivered
        rep["end_to_end_delivery_pct"] = pct(delivered, len(sent))
        rep["not_in_flash"] = len(log.seqs - sent)
    return rep


def print_report(reports, bad_lines):
    for rep in reports:
        print(f"=== tag {rep['tag']} ===")
        width = max(len(k) for k in rep)
        for k, v in rep.items():
            if k != "tag":
                print(f"  {k:<{width}}  {v}")
        print()
    print(f"malformed angle lines: {bad_lines}")


def main():
    ap = argparse.ArgumentParser(description="Summarise a logged anchor session for testing.")
    ap.add_argument("raw", help="raw anchor log written by swimtrack.main --log")
    ap.add_argument("--config", help="settings file used for the session")
    ap.add_argument("--flash", help="flash-log .bin from the same tag and power-on session")
    ap.add_argument("--tag", help="tag the --flash file belongs to (default: the only tag)")
    ap.add_argument("--json", help="also write the report to this file")
    args = ap.parse_args()

    tracker = Tracker(load_settings(args.config))
    with open(args.raw, errors="replace") as f:
        logs = collect(f, tracker)
    if not logs:
        sys.exit("no tag lines found in the log")

    flash = flash_seqs(args.flash) if args.flash else None
    flash_tag = args.tag
    if flash is not None and flash_tag is None:
        if len(logs) != 1:
            sys.exit("several tags in the log: say which one the flash file is with --tag")
        flash_tag = next(iter(logs))

    reports = [tag_report(tag, tracker.tags[tag], logs[tag], flash if tag == flash_tag else None)
               for tag in sorted(logs) if tag in tracker.tags]
    print_report(reports, tracker.bad_lines)
    if args.json:
        Path(args.json).write_text(json.dumps({"tags": reports, "bad_lines": tracker.bad_lines}, indent=2))
        print(f"wrote {args.json}")


if __name__ == "__main__":
    main()