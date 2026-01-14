#!/usr/bin/env python3
"""Log CSV frames from an Arduino/UNO serial port to a .csv file.

Designed for the ICM42688 RS485->UNO receiver sketch output:
- First CSV header line: t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,temp_C
- Then one CSV row per sample.

This tool is intentionally forgiving:
- Ignores non-CSV noise lines
- Can optionally prepend a host timestamp column
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import os
import struct
import sys
import time
from typing import Optional

try:
    import serial  # type: ignore
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "Missing dependency 'pyserial'. Install it with: python3 -m pip install -r tools/requirements.txt"
    ) from exc


DEFAULT_BAUD = 115200
DEFAULT_HEADER = [
    "t_ms",
    "ax_g",
    "ay_g",
    "az_g",
    "gx_dps",
    "gy_dps",
    "gz_dps",
    "temp_C",
]


def _default_output_path() -> str:
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(os.getcwd(), f"imu_log_{stamp}.csv")


def _looks_like_csv_row(line: str) -> bool:
    # Basic heuristic: must contain a structured delimiter.
    # Some sketches use commas, others use tabs.
    stripped = line.strip()
    return bool(stripped) and ("," in stripped or "\t" in stripped)


def _split_fields(line: str) -> list[str]:
    if "," in line:
        parts = line.split(",")
    else:
        parts = line.split("\t")
    return [p.strip() for p in parts if p.strip() != ""]


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Log CSV frames from serial to a .csv file",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Serial port device, e.g. /dev/ttyACM0 or /dev/ttyUSB0",
    )
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud rate")
    parser.add_argument(
        "--reset-wait",
        type=float,
        default=1.5,
        help="Seconds to wait after opening the port (UNO often auto-resets)",
    )
    parser.add_argument(
        "--flush-input",
        action="store_true",
        help="Discard any bytes already received after the reset wait",
    )
    parser.add_argument(
        "--toggle-dtr",
        action="store_true",
        help="Toggle DTR low->high after opening the port (can trigger Arduino auto-reset)",
    )
    parser.add_argument(
        "--out",
        default=None,
        help="Output .csv path. If omitted, creates imu_log_YYYYmmdd_HHMMSS.csv in the current directory.",
    )
    parser.add_argument(
        "--add-host-time",
        action="store_true",
        help="Prepend a host timestamp column 'host_time_iso' to each row",
    )
    parser.add_argument(
        "--echo",
        action="store_true",
        help="Also echo accepted CSV lines to stdout",
    )
    parser.add_argument(
        "--require-header",
        action="store_true",
        help="Only start logging after a CSV header line is seen",
    )
    parser.add_argument(
        "--binary-icm42688",
        action="store_true",
        help="Decode fixed binary frames (A5 5A + 18B payload + XOR) from the ATmega FIFO sender",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Print every received line and why it was accepted/ignored",
    )
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = _parse_args(argv)

    out_path = args.out or _default_output_path()

    # Open serial
    ser = serial.Serial(args.port, args.baud, timeout=1)

    if args.toggle_dtr:
        try:
            ser.dtr = False
            time.sleep(0.2)
            ser.dtr = True
        except Exception:
            pass

    time.sleep(max(0.0, args.reset_wait))  # allow Arduino auto-reset
    if args.flush_input:
        ser.reset_input_buffer()

    header: Optional[list[str]] = None
    header_written = False

    # Open output
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    out_fp = open(out_path, "w", newline="")
    writer = csv.writer(out_fp)

    print(f"Logging from {args.port} @ {args.baud} -> {out_path}")
    if args.flush_input:
        print("Note: input flush enabled (startup banner may be discarded)")
    if args.toggle_dtr:
        print("Note: DTR toggled (Arduino may reset)")
    print("Press Ctrl-C to stop.")

    accepted_rows = 0
    accepted_headers = 0
    ignored_lines = 0

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            # Skip warnings or other non-CSV lines unless they still look like CSV.
            if not _looks_like_csv_row(line):
                if args.debug:
                    print(f"IGNORED (no delimiter): {line}")
                ignored_lines += 1
                continue

            fields = _split_fields(line)

            if args.debug:
                print(f"RECV: {line}")

            # Detect header (contains non-numeric field names)
            if any(not _is_numberish(f) for f in fields):
                header = fields
                if args.add_host_time:
                    header = ["host_time_iso", *header]
                writer.writerow(header)
                out_fp.flush()
                header_written = True
                accepted_headers += 1
                if args.debug:
                    print("ACCEPTED header")
                continue

            if args.require_header and not header_written:
                if args.debug:
                    print("IGNORED (waiting for header)")
                ignored_lines += 1
                continue

            # If numeric rows arrive before any header, write a sensible default.
            if not header_written and len(fields) == len(DEFAULT_HEADER):
                header = DEFAULT_HEADER
                if args.add_host_time:
                    header = ["host_time_iso", *header]
                writer.writerow(header)
                out_fp.flush()
                header_written = True
                accepted_headers += 1
                if args.debug:
                    print("WROTE default header")

            if args.add_host_time:
                host_time = dt.datetime.now(dt.timezone.utc).isoformat()
                writer.writerow([host_time, *fields])
            else:
                writer.writerow(fields)

            out_fp.flush()
            accepted_rows = 0
            accepted_headers = 0
            ignored_lines = 0

            if args.binary_icm42688:
                bin_header = [
                    "t_ms",
                    "ax_raw",
                    "ay_raw",
                    "az_raw",
                    "gx_raw",
                    "gy_raw",
                    "gz_raw",
                ]
                if args.add_host_time:
                    bin_header = ["host_time_iso", *bin_header]
                writer.writerow(bin_header)
                out_fp.flush()
                accepted_headers += 1
                header_written = True

                SYNC = b"\xA5\x5A"
                PAYLOAD_LEN = 16
                FRAME_LEN = 2 + PAYLOAD_LEN + 1
                unpacker = struct.Struct("<Ihhhhhh")
                buf = bytearray()

                try:
                    while True:
                        chunk = ser.read(512)
                        if not chunk:
                            continue
                        buf.extend(chunk)

                        while True:
                            idx = buf.find(SYNC)
                            if idx < 0:
                                # keep last byte in case it's the start of sync
                                if len(buf) > 1:
                                    del buf[:-1]
                                break

                            if len(buf) < idx + FRAME_LEN:
                                # not enough bytes yet; drop leading garbage but keep sync
                                if idx > 0:
                                    del buf[:idx]
                                break

                            frame = bytes(buf[idx : idx + FRAME_LEN])
                            payload = frame[2 : 2 + PAYLOAD_LEN]
                            checksum = frame[-1]
                            calc = 0
                            for b in payload:
                                calc ^= b

                            if calc != checksum:
                                # bad frame; drop one byte and rescan
                                del buf[: idx + 1]
                                continue

                            # good frame; consume it
                            del buf[: idx + FRAME_LEN]

                            t_ms, ax, ay, az, gx, gy, gz = unpacker.unpack(payload)
                            if args.add_host_time:
                                host_time = dt.datetime.now(dt.timezone.utc).isoformat()
                                writer.writerow([host_time, t_ms, ax, ay, az, gx, gy, gz])
                            else:
                                writer.writerow([t_ms, ax, ay, az, gx, gy, gz])

                            out_fp.flush()
                            accepted_rows += 1
                            if args.echo:
                                if args.add_host_time:
                                    print(f"{host_time},{t_ms},{ax},{ay},{az},{gx},{gy},{gz}")
                                else:
                                    print(f"{t_ms},{ax},{ay},{az},{gx},{gy},{gz}")

                except KeyboardInterrupt:
                    print("\nStopped.")
                    print(f"Summary: headers={accepted_headers}, rows={accepted_rows}, ignored={ignored_lines}")
                    return 0

            # Text/CSV mode
            if args.debug:
                print("ACCEPTED row")

    except KeyboardInterrupt:
        print("\nStopped.")
        print(f"Summary: headers={accepted_headers}, rows={accepted_rows}, ignored={ignored_lines}")
        if accepted_headers == 0 and accepted_rows == 0:
            print(
                "No data was written. Common causes: wrong baud, UNO not sending, RS485 link not running, or the port isn't actually attached. Try --debug or increase --reset-wait."
            )
        return 0
    finally:
        try:
            ser.close()
        except Exception:
            pass
        try:
            out_fp.close()
        except Exception:
            pass


def _is_numberish(token: str) -> bool:
    # Accept ints/floats in typical Arduino formatting.
    try:
        float(token)
        return True
    except ValueError:
        return False


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
