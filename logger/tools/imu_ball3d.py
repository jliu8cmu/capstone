#!/usr/bin/env python3
"""
imu_ball3d.py - animate a logger CSV as a ball moving (and turning) in 3D.

  python3 imu_ball3d.py session_007_....csv
  python3 imu_ball3d.py session.csv --start 10 --end 30         # only part of it
  python3 imu_ball3d.py session.csv --speed 0.5                  # half speed
  python3 imu_ball3d.py session.csv --save ball.gif              # save instead of showing (.gif or .mp4)

HOW IT WORKS (and why the path is approximate):
  1. Orientation: the gyro is integrated into a 3D rotation, and gravity in the
     accelerometer gently corrects tilt drift (Mahony complementary filter).
     Orientation is reliable; it is shown as red/green/blue axes on the ball
     (the IMU's x/y/z).
  2. Position: acceleration is rotated into world coordinates, gravity is
     removed, and it is integrated twice. Double-integrating a consumer IMU
     drifts by metres within seconds, so velocity and position are high-pass
     filtered ("leaky") and velocity is reset whenever the IMU is still. The
     ball therefore shows the SHAPE of each movement around a fixed centre -
     not true distance travelled. Real pool position comes from the ANT-B10.

Needs: pip3 install numpy matplotlib   (mp4 saving also needs ffmpeg)
Written with assistance from Claude (Anthropic).
"""
import argparse, csv, sys

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

G = 9.81


# ---------------- data ----------------
def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows or "ax_g" not in rows[0]:
        sys.exit("Expected a CSV from imu_dump.py (columns t_ms, ax_g..gz_dps).")
    t = np.array([float(r["t_ms"]) for r in rows]) / 1000.0
    acc = np.array([[float(r[k]) for k in ("ax_g", "ay_g", "az_g")] for r in rows])
    gyr = np.array([[float(r[k]) for k in ("gx_dps", "gy_dps", "gz_dps")] for r in rows])
    return t - t[0], acc, gyr


# ---------------- quaternion helpers (w, x, y, z) ----------------
def q_mul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2,
                     w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2])


def q_to_R(q):
    w, x, y, z = q
    return np.array([[1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
                     [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
                     [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)]])


def q_from_accel(a):
    """Initial tilt so that measured gravity points along world +z."""
    a = a / np.linalg.norm(a)
    v = np.cross(a, [0, 0, 1.0]); s = np.linalg.norm(v); c = a[2]
    if s < 1e-8:
        return np.array([1.0, 0, 0, 0]) if c > 0 else np.array([0, 1.0, 0, 0])
    ang = np.arctan2(s, c); v /= s
    return np.r_[np.cos(ang / 2), np.sin(ang / 2) * v]


# ---------------- estimation ----------------
def estimate(t, acc, gyr, kp=0.3, v_tau=0.3, p_tau=0.8):
    n = len(t)
    still = (np.linalg.norm(gyr, axis=1) < 10) & (np.abs(np.linalg.norm(acc, axis=1) - 1) < 0.05)
    first = still[: min(n, 300)]
    bias = gyr[: len(first)][first].mean(axis=0) if first.sum() > 20 else np.zeros(3)

    q = q_from_accel(acc[:20].mean(axis=0))
    v = np.zeros(3); p = np.zeros(3)
    pos = np.zeros((n, 3)); rot = np.zeros((n, 3, 3))
    for i in range(n):
        dt = t[i] - t[i - 1] if i else 1 / 104
        dt = min(max(dt, 1e-4), 0.05)
        w = np.radians(gyr[i] - bias)
        a = acc[i]
        an = np.linalg.norm(a)
        if 0.9 < an < 1.1:                              # Mahony tilt correction (only when not accelerating hard)
            R = q_to_R(q)
            g_body = R.T @ np.array([0, 0, 1.0])        # where gravity should be in body frame
            w = w + kp * np.cross(a / an, g_body)
        q = q + 0.5 * q_mul(q, np.r_[0, w]) * dt
        q /= np.linalg.norm(q)
        R = q_to_R(q)
        a_world = (R @ a - np.array([0, 0, 1.0])) * G  # m/s^2, gravity removed
        v = v + a_world * dt
        v *= max(0.0, 1 - dt / v_tau)                   # leaky velocity (high-pass)
        if still[i]:
            v[:] = 0                                    # zero-velocity update
        p = p + v * dt
        p *= max(0.0, 1 - dt / p_tau)                   # leaky position: stays near centre
        pos[i] = p; rot[i] = R
    return pos, rot


# ---------------- animation ----------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--start", type=float, default=0)
    ap.add_argument("--end", type=float, default=None)
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed (1 = real time)")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--trail", type=float, default=1.5, help="trail length in seconds")
    ap.add_argument("--save", help="write .gif or .mp4 instead of opening a window")
    a = ap.parse_args()

    t, acc, gyr = load(a.csv)
    pos, rot = estimate(t, acc, gyr)          # run on the full file so the filter settles
    end = a.end if a.end is not None else t[-1]
    sel = np.where((t >= a.start) & (t <= end))[0]
    if len(sel) < 2:
        sys.exit("No samples in that time range.")

    frame_t = np.arange(t[sel[0]], t[sel[-1]], a.speed / a.fps)
    idx = np.clip(np.searchsorted(t, frame_t), 0, len(t) - 1)
    trail_n = int(a.trail * 104)

    lim = np.percentile(np.abs(pos[sel]), 99.5) * 1.3 + 0.02
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(projection="3d")
    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-lim, lim)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z, up (m)")
    ax.set_box_aspect((1, 1, 1))
    trail, = ax.plot([], [], [], lw=1.5, color="#2E6DB4", alpha=0.6)
    ball, = ax.plot([], [], [], "o", ms=14, color="#D4A03A", markeredgecolor="#333")
    axes_lines = [ax.plot([], [], [], lw=3, color=c)[0] for c in ("#C0392B", "#27AE60", "#2E6DB4")]
    title = ax.set_title("")
    L = lim * 0.35

    def draw(k):
        i = idx[k]
        j0 = max(sel[0], i - trail_n)
        trail.set_data(pos[j0:i + 1, 0], pos[j0:i + 1, 1]); trail.set_3d_properties(pos[j0:i + 1, 2])
        ball.set_data([pos[i, 0]], [pos[i, 1]]); ball.set_3d_properties([pos[i, 2]])
        for d, line in enumerate(axes_lines):
            tip = pos[i] + rot[i][:, d] * L                  # IMU x / y / z axis in world frame
            line.set_data([pos[i, 0], tip[0]], [pos[i, 1], tip[1]])
            line.set_3d_properties([pos[i, 2], tip[2]])
        title.set_text(f"t = {t[i]:6.2f} s   |accel| = {np.linalg.norm(acc[i]):.2f} g   "
                       f"|gyro| = {np.linalg.norm(gyr[i]):4.0f} deg/s\n"
                       f"red/green/blue = IMU x/y/z axes (path is approximate)")
        return [trail, ball, title, *axes_lines]

    anim = FuncAnimation(fig, draw, frames=len(frame_t), interval=1000 / a.fps, blit=False)
    if a.save:
        if a.save.endswith(".gif"):
            anim.save(a.save, writer="pillow", fps=a.fps)
        else:
            anim.save(a.save, fps=a.fps)
        print("saved", a.save)
    else:
        plt.show()


if __name__ == "__main__":
    main()
