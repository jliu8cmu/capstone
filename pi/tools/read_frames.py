#!/usr/bin/env python3
"""Read Pico UART frames (AA 55 len packet), decode them, and check the stream.

Usage (from pi/):
    python3 tools/read_frames.py /dev/cu.usbserial-BG03GWMH
    python3 tools/read_frames.py --file capture.bin
    python3 tools/read_frames.py PORT --log capture.bin --quiet
"""
import argparse
import sys
import time
from collections import Counter
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from swimtrack import packet as pk  # noqa: E402

SYNC = b"\xAA\x55"
HDR_LEN = 3
MIN_LEN = 12
MAX_LEN = 84


class FrameParser:
    """Incremental parser: feed() arbitrary byte chunks, get decoded packets back."""

    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0
        self.format_errors = 0
        self.skipped_bytes = 0

    def feed(self, chunk: bytes):
        self.buf += chunk
        out = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                keep = 1 if self.buf.endswith(SYNC[:1]) else 0
                self.skipped_bytes += len(self.buf) - keep
                del self.buf[:len(self.buf) - keep]
                return out
            if i:
                self.skipped_bytes += i
                del self.buf[:i]
            if len(self.buf) < HDR_LEN:
                return out
            n_bytes = self.buf[2]
            if not MIN_LEN <= n_bytes <= MAX_LEN:
                self.format_errors += 1
                self._drop_one()
                continue
            if len(self.buf) < HDR_LEN + n_bytes:
                return out
            payload = bytes(self.buf[HDR_LEN:HDR_LEN + n_bytes])
            try:
                p = pk.decode(payload)
            except pk.CrcError:
                self.crc_errors += 1
                self._drop_one()
                continue
            except pk.PacketError:
                self.format_errors += 1
                self._drop_one()
                continue
            del self.buf[:HDR_LEN + n_bytes]
            out.append(p)

    def _drop_one(self):
        del self.buf[:1]
        self.skipped_bytes += 1


class StreamStats:
    def __init__(self):
        self.frames = 0
        self.gaps = 0
        self.lost = 0
        self.dups = 0
        self.last_seq = None
        self.n_hist = Counter()
        self.flag_counts = Counter()

    def update(self, p):
        self.frames += 1
        self.n_hist[p.n] += 1
        for bit, name in ((pk.FLAG_IMU_ERR, "IMU_ERR"), (pk.FLAG_OVERRUN, "OVERRUN"),
                          (pk.FLAG_LOW_BATT, "LOW_BATT")):
            if p.flags & bit:
                self.flag_counts[name] += 1
        note = ""
        if self.last_seq is not None:
            step = (p.seq - self.last_seq) & 0xFFFF
            if step == 0:
                self.dups += 1
                note = "DUP"
            elif step != 1:
                self.gaps += 1
                self.lost += step - 1
                note = f"GAP {step - 1}"
        self.last_seq = p.seq
        return note


def flag_str(flags):
    names = [n for b, n in ((pk.FLAG_IMU_ERR, "IMU"), (pk.FLAG_OVERRUN, "OVR"),
                            (pk.FLAG_LOW_BATT, "LOW")) if flags & b]
    return ",".join(names) or "-"


def frame_line(p, note):
    if p.n:
        a = p.accel_g.mean(axis=0)
        acc = f"a=({a[0]:+.2f},{a[1]:+.2f},{a[2]:+.2f})g |a|={np.linalg.norm(a):.2f}"
    else:
        acc = "a=(no samples)"
    return (f"seq={p.seq:5d} n={p.n} t0={p.t0_ms:8d} batt={p.batt_mv}mV "
            f"flags={flag_str(p.flags):11s} {acc} {note}")


def summary(stats, parser, elapsed):
    rate = stats.frames / elapsed if elapsed > 0 else 0.0
    n_dist = " ".join(f"n{k}:{v}" for k, v in sorted(stats.n_hist.items()))
    flags = " ".join(f"{k}:{v}" for k, v in stats.flag_counts.items()) or "none"
    return (f"[{elapsed:6.1f}s] frames={stats.frames} ({rate:.1f}/s) gaps={stats.gaps} "
            f"lost={stats.lost} dups={stats.dups} crc_err={parser.crc_errors} "
            f"fmt_err={parser.format_errors} skipped={parser.skipped_bytes}B "
            f"| {n_dist} | flags {flags}")


def run(read_chunk, quiet, log_file):
    parser, stats = FrameParser(), StreamStats()
    start = last_summary = time.monotonic()
    try:
        while True:
            chunk = read_chunk()
            if chunk is None:
                break
            if log_file and chunk:
                log_file.write(chunk)
            for p in parser.feed(chunk):
                note = stats.update(p)
                if not quiet or note:
                    print(frame_line(p, note))
            now = time.monotonic()
            if now - last_summary >= 1.0:
                if quiet:
                    print(summary(stats, parser, now - start))
                last_summary = now
    except KeyboardInterrupt:
        pass
    print(summary(stats, parser, time.monotonic() - start))
    return stats, parser


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", help="serial port, e.g. /dev/cu.usbserial-BG03GWMH")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--file", help="read a raw capture file instead of a serial port")
    ap.add_argument("--log", help="save raw bytes to this file for later --file replay")
    ap.add_argument("--quiet", action="store_true", help="one summary line per second instead of every frame")
    args = ap.parse_args(argv)
    if not args.port and not args.file:
        ap.error("give a serial port or --file")

    log_file = open(args.log, "wb") if args.log else None
    try:
        if args.file:
            data = Path(args.file).read_bytes()
            chunks = iter([data[i:i + 64] for i in range(0, len(data), 64)])
            run(lambda: next(chunks, None), args.quiet, log_file)
        else:
            import serial
            with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
                ser.reset_input_buffer()
                print(f"reading {args.port} @ {args.baud}, Ctrl+C to stop")
                run(lambda: ser.read(ser.in_waiting or 1), args.quiet, log_file)
    finally:
        if log_file:
            log_file.close()


if __name__ == "__main__":
    main()