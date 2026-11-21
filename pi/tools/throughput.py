#!/usr/bin/env python3
import argparse
import os
import random
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import sim
from swimtrack import packet as pk
from swimtrack.main import Tracker

EVENTS_PER_S = 20
UPLINK_EVERY_MS = 100


def make_lines(packets, tags, seconds, seed):
    rng = random.Random(seed)
    n_events = int(seconds * EVENTS_PER_S)
    lines = []
    for k in range(tags):
        tag = f"{k + 1:012X}"
        lane = k % sim.POOL.lanes + 1
        offset = rng.randrange(len(packets))
        counter0 = rng.randrange(65536)
        for i in range(n_events):
            p = packets[(offset + i) % len(packets)]
            t_ms = i * 1000 // EVENTS_PER_S
            p = pk.Packet(p.version, i & 0xFFFF, t_ms, p.samples, p.batt, p.flags)
            adv = sim.wrap(pk.encode(p)).hex().upper()
            lines.append((t_ms, sim.angle_line(tag, t_ms, lane, counter0 + i, rng)))
            lines.append((t_ms, f"+UUDFP:{tag},{adv}\r\n"))
    lines.sort(key=lambda x: x[0])
    return lines


def run(lines):
    tracker = Tracker()
    next_uplink = 0
    messages = 0
    cpu0, wall0 = time.process_time(), time.perf_counter()
    for t_ms, line in lines:
        tracker.handle_line(line)
        if t_ms >= next_uplink:
            messages += sum(1 for _ in tracker.uplink_messages())
            next_uplink = t_ms + UPLINK_EVERY_MS
    return time.process_time() - cpu0, time.perf_counter() - wall0, messages, tracker


def main():
    ap = argparse.ArgumentParser(description="Measure how many tags the pipeline can keep up with.")
    ap.add_argument("recording", help=".bin capture used as the IMU data for every simulated tag")
    ap.add_argument("--tags", type=int, nargs="+", default=[1, 4, 8, 16])
    ap.add_argument("--seconds", type=float, default=60.0, help="simulated session length")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    packets = sim.load(args.recording)
    print(f"machine: {os.uname().machine}, {os.cpu_count()} cores; simulated {args.seconds:.0f} s per run, "
          f"{EVENTS_PER_S} events/s per tag")
    print(f"{'tags':>4} {'packets':>8} {'cpu_s':>7} {'us/pkt':>7} {'max_pkt/s':>10} "
          f"{'real_pkt/s':>10} {'cpu_load':>8} {'max_tags':>8}")
    for tags in args.tags:
        lines = make_lines(packets, tags, args.seconds, args.seed)
        cpu, wall, messages, tracker = run(lines)
        n_pkts = sum(st.packets for st in tracker.tags.values())
        us_per_pkt = cpu / n_pkts * 1e6
        max_rate = n_pkts / cpu
        real_rate = tags * EVENTS_PER_S
        load = 100.0 * real_rate / max_rate
        print(f"{tags:>4} {n_pkts:>8} {cpu:>7.2f} {us_per_pkt:>7.0f} {max_rate:>10.0f} "
              f"{real_rate:>10} {load:>7.1f}% {int(max_rate / EVENTS_PER_S):>8}")


if __name__ == "__main__":
    main()