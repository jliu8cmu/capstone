#!/usr/bin/env python3
"""
imu_dump.py - download recordings from the Pico IMU logger (or capture a live stream).

  pip3 install pyserial
  python3 imu_dump.py dump            # save every stored session to its own CSV
  python3 imu_dump.py live out.csv    # stream live data to a CSV until Ctrl-C
  python3 imu_dump.py info            # sessions / minutes used / free

Options: --port /dev/cu.usbmodemXXXX  (auto-detected if only one Pico is plugged in)
         --out  folder for dump files (default: ./imu_data)

CSV columns: session, t_ms (ms since that session's power-up), raw counts
ax..gz, plus converted units ax_g..az_g (g) and gx_dps..gz_dps (deg/s).

Written with assistance from Claude (Anthropic).
"""
import argparse, csv, glob, os, sys, time
from datetime import datetime

import serial  # pyserial

ACC_G_PER_COUNT = 0.000244    # +/-8 g full scale
GYR_DPS_PER_COUNT = 0.070     # +/-2000 dps full scale
HEADER = ["session", "t_ms", "ax", "ay", "az", "gx", "gy", "gz",
          "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps"]


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*"))
    if len(ports) != 1:
        sys.exit(f"Found {len(ports)} candidate ports {ports}; pass --port explicitly.")
    return ports[0]


def open_pico(port):
    s = serial.Serial(port, 115200, timeout=2)
    time.sleep(1.0)                 # let the Pico print its banner/menu
    s.reset_input_buffer()
    return s


def readline(s):
    return s.readline().decode(errors="replace").strip()


def convert(row):
    """row = [session, t_ms, ax, ay, az, gx, gy, gz] as strings -> full output row."""
    raw = [int(v) for v in row[2:8]]
    acc = [f"{v * ACC_G_PER_COUNT:.4f}" for v in raw[:3]]
    gyr = [f"{v * GYR_DPS_PER_COUNT:.2f}" for v in raw[3:]]
    return row[:8] + acc + gyr


def wait_for(s, token, timeout_s=10):
    end = time.time() + timeout_s
    while time.time() < end:
        if readline(s) == token:
            return True
    return False


def cmd_dump(s, outdir):
    os.makedirs(outdir, exist_ok=True)
    s.write(b"d")
    if not wait_for(s, "BEGIN"):
        sys.exit("No response to dump command - is the logger firmware running in USB mode?")
    readline(s)                                   # skip the device's CSV header
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    files, writers, counts = {}, {}, {}
    s.timeout = 5
    while True:
        line = readline(s)
        if line == "END":
            break
        if not line:
            sys.exit("Timed out mid-dump; try again (keep the cable still).")
        row = line.split(",")
        if len(row) != 8:
            continue
        sess = row[0]
        if sess not in writers:
            path = os.path.join(outdir, f"session_{int(sess):03d}_{stamp}.csv")
            files[sess] = open(path, "w", newline="")
            writers[sess] = csv.writer(files[sess])
            writers[sess].writerow(HEADER)
            counts[sess] = 0
        writers[sess].writerow(convert(row))
        counts[sess] += 1
        total = sum(counts.values())
        if total % 10000 == 0:
            print(f"  {total} samples...", end="\r")
    for f in files.values():
        f.close()
    if not counts:
        print("No recordings stored.")
    for sess, n in counts.items():
        print(f"session {sess}: {n} samples ({n / 104:.1f} s)")
    print(f"saved to {os.path.abspath(outdir)}")


def cmd_live(s, path):
    s.write(b"l")
    readline(s)                                   # echoed command
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(HEADER)
        n = 0
        print("Recording live data - Ctrl-C to stop.")
        try:
            while True:
                row = readline(s).split(",")
                if len(row) == 8 and row[0] == "0":
                    w.writerow(convert(row))
                    n += 1
                    if n % 104 == 0:
                        print(f"  {n / 104:.0f} s", end="\r")
        except KeyboardInterrupt:
            pass
        finally:
            s.write(b"x")                         # any key stops the stream
    print(f"\n{n} samples saved to {path}")


def cmd_info(s):
    s.write(b"i")
    readline(s)
    print(readline(s))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("command", choices=["dump", "live", "info"])
    ap.add_argument("outfile", nargs="?", default="live.csv")
    ap.add_argument("--port")
    ap.add_argument("--out", default="imu_data")
    a = ap.parse_args()
    s = open_pico(a.port or find_port())
    if a.command == "dump":
        cmd_dump(s, a.out)
    elif a.command == "live":
        cmd_live(s, a.outfile)
    else:
        cmd_info(s)


if __name__ == "__main__":
    main()
