#!/usr/bin/env python3
"""Simulate ANT-B10 output from recorded .bin captures, one tag per file.

For every packet the simulated anchor prints an angle line, then a payload line
(format from the anchor teammate):
    +UUDF:<tag>,<rssi>,<az>,<el>,0,<channel>,"<anchor id>","",<timestamp ms>,<counter>
    +UUDFP:<tag>,<[len][FF][FF FF][packet] as hex>
Angles come from a fake swim path (each tag swims lengths in its own lane) run
through swimtrack.geometry, rounded to whole degrees with noise added.

Usage (from pi/):
    python3 tools/sim.py data/session7.bin
    python3 tools/sim.py data/session7.bin data/run_s2.bin --speed 4
    python3 tools/sim.py data/session7.bin --drop 0.1 --dup 0.1 --corrupt 0.02 > sim.txt
    python3 tools/sim.py data/session7.bin | python3 -m swimtrack.main -
    python3 tools/sim.py data/session7.bin --config config.yaml | python3 -m swimtrack.main - --config config.yaml
"""
import argparse
import heapq
import math
import random
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from read_frames import FrameParser  # noqa: E402
from swimtrack import packet as pk  # noqa: E402
from swimtrack.config import default_anchor, load_settings  # noqa: E402
from swimtrack.geometry import PoolConfig, position_to_angles  # noqa: E402

AD_HEADER = bytes([0xFF, 0xFF, 0xFF])          # type 0xFF, company FF FF
ANCHOR_ID = "80A197B210EC"
POOL = PoolConfig()
ANCHOR = default_anchor()
SWIM_SPEED = 1.4                               # m/s
WALL_GAP = 1.0                                 # turn this far from each wall (m)


def wrap(packet_bytes: bytes) -> bytes:
    body = AD_HEADER + packet_bytes
    return bytes([len(body)]) + body


def swim_position(t_s: float, lane: int):
    """Back-and-forth along the lane centre at SWIM_SPEED."""
    span = POOL.length - 2 * WALL_GAP
    d = (t_s * SWIM_SPEED) % (2 * span)
    x = WALL_GAP + (d if d <= span else 2 * span - d)
    y = (lane - 0.5) * POOL.lane_width
    return x, y


def angle_line(tag, t_ms, lane, counter, rng):
    x, y = swim_position(t_ms / 1000, lane)
    az, el = position_to_angles(x, y, ANCHOR)
    az = max(-90, min(90, round(az + rng.gauss(0, 1.0))))
    el = max(-90, min(90, round(el + rng.gauss(0, 1.0))))
    dist = math.dist((x, y, 0.0), (ANCHOR.x, ANCHOR.y, ANCHOR.height))
    rssi = round(-45 - 20 * math.log10(dist) + rng.gauss(0, 2))
    channel = rng.randrange(37)
    return (f'+UUDF:{tag},{rssi},{az},{el},0,{channel},"{ANCHOR_ID}","",'
            f'{123456 + int(t_ms)},{counter & 0xFFFF}\r\n')


def load(path):
    packets = FrameParser().feed(Path(path).read_bytes())
    if not packets:
        sys.exit(f"{path}: no valid frames")
    return packets


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="+", help=".bin captures, one per simulated tag")
    ap.add_argument("--speed", type=float, default=1.0, help="1 = real time, 4 = 4x, 0 = no waiting")
    ap.add_argument("--drop", type=float, default=0.0, help="probability the anchor misses an event")
    ap.add_argument("--dup", type=float, default=0.0, help="probability a payload line is repeated")
    ap.add_argument("--corrupt", type=float, default=0.0, help="probability a payload byte is flipped")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--config", help="use the pool and anchor from this settings file")
    args = ap.parse_args()
    if args.config:
        global POOL, ANCHOR
        settings = load_settings(args.config)
        POOL, ANCHOR = settings.pool, settings.anchor
    rng = random.Random(args.seed)

    queue = []
    for k, path in enumerate(args.files):
        tag = f"{k + 1:012X}"
        lane = k + 1
        packets = load(path)
        t_start = packets[0].t0_ms
        counter0 = rng.randrange(65536)
        for i, p in enumerate(packets):
            heapq.heappush(queue, (p.t0_ms - t_start, k, i, tag, lane, counter0 + i, pk.encode(p)))

    wall0 = time.monotonic()
    out = sys.stdout
    try:
        while queue:
            t_ms, _, _, tag, lane, counter, data = heapq.heappop(queue)
            if args.speed > 0:
                delay = wall0 + t_ms / 1000 / args.speed - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
            if rng.random() < args.drop:
                continue
            adv = bytearray(wrap(data))
            if rng.random() < args.corrupt:
                adv[rng.randrange(4, len(adv))] ^= 0xFF
            payload = f"+UUDFP:{tag},{adv.hex().upper()}\r\n"
            out.write(angle_line(tag, t_ms, lane, counter, rng))
            out.write(payload)
            if rng.random() < args.dup:
                out.write(payload)
            out.flush()
    except (KeyboardInterrupt, BrokenPipeError):
        pass


if __name__ == "__main__":
    main()