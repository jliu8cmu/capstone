#!/usr/bin/env python3
"""Live 3D view of the wearable's orientation, from the Pico's UART frames.

A dot on a unit sphere shows where one board axis points; a short trail shows recent motion.
A counter underneath shows strokes detected by swimtrack.strokes.
Orientation comes from a Mahony filter (gyro + accel). There is no magnetometer, so heading
(rotation about vertical) slowly drifts; tilt stays correct.

Usage (from pi/):
    python3 tools/live3d.py /dev/cu.usbmodem11201 --usb      (Pico USB port, hex frame lines)
    python3 tools/live3d.py /dev/cu.usbserial-XXXX           (FTDI on Pico GP4, raw frames)
    python3 tools/live3d.py --file ../data/logs/run_s2.bin          (replay a capture)
    python3 tools/live3d.py PORT --axis z                    (track the z axis instead of x)
"""
import argparse
import sys
from collections import deque
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from read_frames import FrameParser  # noqa: E402
from swimtrack.strokes import StrokeDetector  # noqa: E402

DT = 1.0 / 104.0
KP = 0.5            # how strongly gravity corrects gyro drift
TRAIL = 104         # samples of trail (~1 s)
AXES = {"x": 0, "y": 1, "z": 2}


class Mahony:
    """Minimal Mahony orientation filter. q = (w, x, y, z), body -> world."""

    def __init__(self):
        self.q = np.array([1.0, 0.0, 0.0, 0.0])

    def init_from_accel(self, accel_g):
        """Start with the tilt the accelerometer reports (heading = 0)."""
        ax, ay, az = accel_g / np.linalg.norm(accel_g)
        roll, pitch = np.arctan2(ay, az), np.arctan2(-ax, np.hypot(ay, az))
        cr, sr, cp, sp = np.cos(roll / 2), np.sin(roll / 2), np.cos(pitch / 2), np.sin(pitch / 2)
        self.q = np.array([cr * cp, sr * cp, cr * sp, -sr * sp])
        self.started = True

    def update(self, gyro_dps, accel_g):
        if not getattr(self, "started", False):
            self.init_from_accel(accel_g)
        w, x, y, z = self.q
        g = np.radians(gyro_dps)
        a_norm = np.linalg.norm(accel_g)
        if 0.8 < a_norm < 1.2:                      # only trust accel when not shaking hard
            a = accel_g / a_norm
            v = np.array([2 * (x * z - w * y), 2 * (w * x + y * z), w * w - x * x - y * y + z * z])
            g = g + KP * np.cross(a, v)
        gx, gy, gz = g
        dq = 0.5 * np.array([-x * gx - y * gy - z * gz,
                             w * gx + y * gz - z * gy,
                             w * gy - x * gz + z * gx,
                             w * gz + x * gy - y * gx])
        self.q = self.q + dq * DT
        self.q /= np.linalg.norm(self.q)

    def rotation(self):
        w, x, y, z = self.q
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ])


def make_source(args):
    if args.file:
        data = Path(args.file).read_bytes()
        state = {"pos": 0}

        def read():
            chunk = data[state["pos"]:state["pos"] + 50]   # ~real time at 30 fps
            state["pos"] += 50
            return chunk
        return read
    import serial
    ser = serial.Serial(args.port, args.baud, timeout=0)
    ser.reset_input_buffer()
    if not args.usb:
        return lambda: ser.read(ser.in_waiting or 1)

    pending = bytearray()

    def read_hex_lines():
        pending.extend(ser.read(ser.in_waiting or 1))
        out = bytearray()
        while b"\n" in pending:
            line, _, rest = pending.partition(b"\n")
            pending[:] = rest
            line = line.strip()
            if line.startswith(b"F "):
                try:
                    out += bytes.fromhex(line[2:].decode())
                except ValueError:
                    pass
        return bytes(out)
    return read_hex_lines


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--file")
    ap.add_argument("--usb", action="store_true", help="port is the Pico's USB (hex 'F ...' lines)")
    ap.add_argument("--axis", choices=AXES, default="x", help="board axis to show as the dot")
    args = ap.parse_args()
    if not args.port and not args.file:
        ap.error("give a serial port or --file")

    read = make_source(args)
    parser = FrameParser()
    filt = Mahony()
    col = AXES[args.axis]
    trail = deque(maxlen=TRAIL)
    strokes = StrokeDetector()

    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(projection="3d")
    u, v = np.mgrid[0:2 * np.pi:30j, 0:np.pi:15j]
    ax.plot_wireframe(np.cos(u) * np.sin(v), np.sin(u) * np.sin(v), np.cos(v), color="0.85", lw=0.5)
    for i, c in enumerate("rgb"):
        ax.plot(*np.eye(3)[i][:, None] * [[0, 1.2]], color=c, lw=1)
    (trail_line,) = ax.plot([], [], [], lw=1.5, color="tab:blue", alpha=0.6)
    (stem,) = ax.plot([], [], [], lw=1, color="k")
    (dot,) = ax.plot([], [], [], "o", ms=10, color="tab:red")
    ax.set(xlim=(-1.2, 1.2), ylim=(-1.2, 1.2), zlim=(-1.2, 1.2), xlabel="X", ylabel="Y", zlabel="Z up")
    ax.set_box_aspect((1, 1, 1))
    title = ax.set_title(f"board {args.axis}-axis")
    counter = fig.text(0.5, 0.04, "strokes: 0", ha="center", fontsize=20)

    def tick(_):
        for p in parser.feed(read()):
            strokes.update_packet(p)
            rate = strokes.rate_spm(p.t0_ms)
            counter.set_text(f"strokes: {strokes.count}   rate: {f'{rate:.0f}/min' if rate else '-'}")
            for g_dps, a_g in zip(p.gyro_dps, p.accel_g):
                filt.update(g_dps, a_g)
                trail.append(filt.rotation()[:, col])
            title.set_text(f"board {args.axis}-axis   seq {p.seq}   crc_err {parser.crc_errors}")
        if trail:
            pts = np.array(trail)
            trail_line.set_data(pts[:, 0], pts[:, 1])
            trail_line.set_3d_properties(pts[:, 2])
            d = pts[-1]
            dot.set_data([d[0]], [d[1]])
            dot.set_3d_properties([d[2]])
            stem.set_data([0, d[0]], [0, d[1]])
            stem.set_3d_properties([0, d[2]])
        return trail_line, dot, stem, title, counter

    anim = FuncAnimation(fig, tick, interval=33, cache_frame_data=False)  # noqa: F841
    plt.show()


if __name__ == "__main__":
    main()