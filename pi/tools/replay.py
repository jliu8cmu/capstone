#!/usr/bin/env python3
"""Run stroke detection on a recorded .bin capture.

Usage (from pi/):
    python3 tools/replay.py demo_s1.bin
    python3 tools/replay.py demo_s1.bin --plot
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from read_frames import FrameParser  # noqa: E402
from swimtrack.strokes import SAMPLE_PERIOD_MS, StrokeDetector  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file")
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    packets = FrameParser().feed(Path(args.file).read_bytes())
    if not packets:
        sys.exit("no valid frames in file")

    det = StrokeDetector()
    t_all, g_all = [], []
    for p in packets:
        for ev in det.update_packet(p):
            print(f"stroke {det.count:3d} at {ev.t_ms / 1000:7.2f} s")
        t_all.append(p.t0_ms + np.arange(p.n) * SAMPLE_PERIOD_MS)
        g_all.append(p.gyro_dps)

    t = np.concatenate(t_all) / 1000
    g = np.linalg.norm(np.concatenate(g_all), axis=1)
    rate = det.rate_spm(t[-1] * 1000)
    rate_txt = f"{rate:.0f} /min" if rate else "-"
    print(f"\n{len(packets)} packets, {t[-1] - t[0]:.1f} s, "
          f"{det.count} strokes, last rate: {rate_txt}")

    if args.plot:
        import matplotlib.pyplot as plt
        stroke_t = np.array([e.t_ms for e in det.events]) / 1000
        plt.plot(t, g, lw=0.6, label="|gyro|")
        plt.vlines(stroke_t, 0, g.max(), colors="r", lw=0.8, label="stroke")
        plt.xlabel("time (s)")
        plt.ylabel("dps")
        plt.title(f"{Path(args.file).name}: {det.count} strokes")
        plt.legend()
        plt.show()


if __name__ == "__main__":
    main()