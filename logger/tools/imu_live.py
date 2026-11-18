#!/usr/bin/env python3
"""
imu_live.py - live scrolling plot of the Pico IMU logger (USB mode).

  python3 imu_live.py                       # auto-detect the Pico
  python3 imu_live.py --port /dev/cu.usbmodemXXXX --seconds 10

Shows the last N seconds of accel (g) and gyro (deg/s), all three axes.
Close the window (or Ctrl-C) to stop; the Pico returns to its menu.
Nothing is written to flash.

Needs: pip3 install pyserial matplotlib
Written with assistance from Claude (Anthropic).
"""
import argparse, collections, glob, sys, time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial

ACC = 0.000244     # g per count   (+/-8 g)
GYR = 0.070        # dps per count (+/-2000 dps)
RATE = 104


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*"))
    if len(ports) != 1:
        sys.exit(f"Found {len(ports)} candidate ports {ports}; pass --port explicitly.")
    return ports[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--seconds", type=float, default=8)
    a = ap.parse_args()

    s = serial.Serial(a.port or find_port(), 115200, timeout=0)
    time.sleep(1.0)
    s.reset_input_buffer()
    s.write(b"l")                                   # start live stream

    n = int(a.seconds * RATE)
    t = collections.deque(maxlen=n)
    acc = [collections.deque(maxlen=n) for _ in range(3)]
    gyr = [collections.deque(maxlen=n) for _ in range(3)]
    buf = b""

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    colors = ("#C0392B", "#27AE60", "#2E6DB4")
    la = [ax1.plot([], [], lw=1, color=c, label=l)[0] for c, l in zip(colors, ("ax", "ay", "az"))]
    lg = [ax2.plot([], [], lw=1, color=c, label=l)[0] for c, l in zip(colors, ("gx", "gy", "gz"))]
    ax1.set_ylabel("accel (g)"); ax2.set_ylabel("gyro (deg/s)"); ax2.set_xlabel("time (s)")
    ax1.legend(loc="upper left", ncol=3); ax2.legend(loc="upper left", ncol=3)
    for ax in (ax1, ax2):
        ax.grid(alpha=0.3)
    title = ax1.set_title("waiting for data...")

    def update(_):
        nonlocal buf
        buf += s.read(s.in_waiting or 1)
        *lines, buf = buf.split(b"\n")
        for ln in lines:
            f = ln.decode(errors="ignore").strip().split(",")
            if len(f) != 8 or f[0] != "0":
                continue
            try:
                v = [int(x) for x in f[2:]]
                t.append(int(f[1]) / 1000.0)
            except ValueError:
                continue
            for i in range(3):
                acc[i].append(v[i] * ACC)
                gyr[i].append(v[3 + i] * GYR)
        if t:
            for i in range(3):
                la[i].set_data(t, acc[i])
                lg[i].set_data(t, gyr[i])
            ax1.set_xlim(t[-1] - a.seconds, t[-1])
            for ax, d in ((ax1, acc), (ax2, gyr)):
                lo = min(min(x) for x in d); hi = max(max(x) for x in d)
                pad = 0.1 * (hi - lo) + 1e-3
                ax.set_ylim(lo - pad, hi + pad)
            title.set_text(f"live IMU  |accel| last = "
                           f"{(acc[0][-1]**2 + acc[1][-1]**2 + acc[2][-1]**2) ** 0.5:.2f} g")
        return la + lg + [title]

    anim = FuncAnimation(fig, update, interval=50, cache_frame_data=False)  # ~20 fps
    try:
        plt.show()
    finally:
        s.write(b"x")                               # any key stops the stream
        s.close()


if __name__ == "__main__":
    main()