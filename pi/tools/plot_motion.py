#!/usr/bin/env python3
"""Plot the motion recorded in a .bin capture: accel, gyro, and estimated orientation.

Usage (from pi/):
    python3 tools/plot_motion.py data/run_s2.bin
    python3 tools/plot_motion.py data/run_s2.bin --save run_s2.png
For an animated 3D replay of the same file:  python3 tools/live3d.py --file data/run_s2.bin
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from read_frames import FrameParser  # noqa: E402
from live3d import Mahony  # noqa: E402
from swimtrack.strokes import SAMPLE_PERIOD_MS  # noqa: E402


def euler_deg(q):
    w, x, y, z = q
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
    yaw = np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return roll, pitch, yaw


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file")
    ap.add_argument("--save", help="write the figure to this image file instead of showing it")
    args = ap.parse_args()

    parser = FrameParser()
    packets = parser.feed(Path(args.file).read_bytes())
    if not packets:
        sys.exit("no valid frames in file")

    t = np.concatenate([p.t0_ms + np.arange(p.n) * SAMPLE_PERIOD_MS for p in packets]) / 1000
    t -= t[0]
    accel = np.concatenate([p.accel_g for p in packets])
    gyro = np.concatenate([p.gyro_dps for p in packets])

    filt = Mahony()
    angles = []
    for g, a in zip(gyro, accel):
        filt.update(g, a)
        angles.append(euler_deg(filt.q))
    angles = np.array(angles)

    fig, ax = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for i, (name, color) in enumerate(zip("xyz", ["tab:red", "tab:green", "tab:blue"])):
        ax[0].plot(t, accel[:, i], lw=0.7, color=color, label=name)
        ax[1].plot(t, gyro[:, i], lw=0.7, color=color, label=name)
    for i, name in enumerate(["roll", "pitch", "yaw (drifts)"]):
        ax[2].plot(t, angles[:, i], lw=0.9, label=name)

    ax[0].set_ylabel("accel (g)")
    ax[1].set_ylabel("gyro (dps)")
    ax[2].set_ylabel("orientation (deg)")
    ax[2].set_xlabel("time (s)")
    for a in ax:
        a.legend(loc="upper right", fontsize=8)
        a.grid(alpha=0.3)
    fig.suptitle(f"{Path(args.file).name}: {len(packets)} packets, {t[-1]:.1f} s, "
                 f"crc_err {parser.crc_errors}")
    fig.tight_layout()

    if args.save:
        fig.savefig(args.save, dpi=120)
        print(f"saved {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()