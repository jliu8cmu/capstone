#!/usr/bin/env python3
"""Pull the flash log off the Pico (wearable_pico firmware), one .bin file per power-on session.

Usage (from pi/), Pico on USB:
    python3 tools/dump_log.py /dev/cu.usbmodem21201 ../data/logs/run       -> ../data/logs/run_s1.bin, ...
    python3 tools/dump_log.py /dev/cu.usbmodem21201 --info
    python3 tools/dump_log.py /dev/cu.usbmodem21201 --erase
Then:  python3 tools/read_frames.py --file ../data/logs/run_s1.bin
"""
import argparse
import sys
import time
from pathlib import Path

import serial

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from swimtrack import packet as pk  # noqa: E402

SYNC = b"\xAA\x55"


def split_sessions(data: bytes):
    """Cut the log where t0_ms goes backwards: the Pico rebooted, so a new session began."""
    starts, prev_t0, pos = [0], None, 0
    while True:
        i = data.find(SYNC, pos)
        if i < 0 or i + 3 > len(data):
            break
        n = data[i + 2]
        try:
            p = pk.decode(data[i + 3:i + 3 + n])
        except pk.PacketError:
            pos = i + 1
            continue
        if prev_t0 is not None and p.t0_ms < prev_t0:
            starts.append(i)
        prev_t0 = p.t0_ms
        pos = i + 3 + n
    bounds = starts + [len(data)]
    return [data[a:b] for a, b in zip(bounds, bounds[1:])]


def read_line(ser, deadline):
    while time.monotonic() < deadline:
        line = ser.readline()
        if line:
            return line.decode("ascii", errors="replace").strip()
    raise TimeoutError("no reply from Pico (is wearable_pico running and on USB?)")


def wait_for(ser, prefix, timeout):
    deadline = time.monotonic() + timeout
    while True:
        line = read_line(ser, deadline)
        if line.startswith(prefix):
            return line


def dump(ser, stem):
    ser.write(b"d")
    begin = wait_for(ser, "LOGBEGIN", timeout=5)
    expected = int(begin.split()[1])
    data = bytearray()
    while True:
        line = read_line(ser, time.monotonic() + 5)   # 5 s of silence = stalled
        if line == "LOGEND":
            break
        if line.startswith("LOG "):
            data += bytes.fromhex(line[4:])
            if len(data) % (64 * 1024) < 32:
                print(f"\r{len(data)}/{expected} bytes", end="", flush=True)
    print(f"\r{len(data)}/{expected} bytes")
    if len(data) != expected:
        sys.exit(f"incomplete dump: got {len(data)} of {expected} bytes")
    for k, chunk in enumerate(split_sessions(bytes(data)), start=1):
        path = Path(f"{stem}_s{k}.bin")
        path.write_bytes(chunk)
        print(f"saved {path} ({len(chunk)} bytes)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port")
    ap.add_argument("out", nargs="?", default="capture", help="output name stem")
    ap.add_argument("--info", action="store_true", help="print how much of the log is used")
    ap.add_argument("--erase", action="store_true", help="erase the log (dump it first!)")
    args = ap.parse_args()

    with serial.Serial(args.port, 115200, timeout=0.2) as ser:
        time.sleep(0.3)
        ser.reset_input_buffer()
        if args.info:
            ser.write(b"i")
            print(wait_for(ser, "log ", timeout=3))
        elif args.erase:
            if input("Erase the Pico log? Type yes: ") != "yes":
                sys.exit("cancelled")
            ser.write(b"E")
            print(wait_for(ser, "erased", timeout=60))
        else:
            dump(ser, args.out)


if __name__ == "__main__":
    main()