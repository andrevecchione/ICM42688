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
import math
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


def _is_reasonably_printable_ascii(line: str) -> bool:
    # Text-mode CSV from Arduino should be plain ASCII. If we see the unicode
    # replacement char (\ufffd) or lots of control characters, it's very likely
    # we're accidentally looking at binary data.
    if "\ufffd" in line:
        return False
    for ch in line:
        o = ord(ch)
        if ch in ("\t",):
            continue
        if o < 0x20 or o > 0x7E:
            return False
    return True


def _crc16_ccitt(data: bytes) -> int:
    # CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no xorout
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


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
        help="Decode fixed binary frames (A5 5A + 16B payload + XOR) from the ATmega FIFO sender",
    )

    parser.add_argument(
        "--binary-include-raw",
        action="store_true",
        help="In binary mode, include raw accel/gyro counts columns in the CSV",
    )
    parser.add_argument(
        "--binary-include-fsync",
        action="store_true",
        help="In binary batch mode, include per-packet FSYNC timestamp columns (fsync_ts, fsync_ts_unwrapped)",
    )
    parser.add_argument(
        "--binary-include-imu-ts",
        action="store_true",
        help="In binary mode, include imu_ts and imu_ts_unwrapped columns in the CSV",
    )

    parser.add_argument(
        "--imu-ts-tick-us",
        type=float,
        default=None,
        help=(
            "IMU FIFO timestamp tick period in microseconds (used to derive time_s from imu_ts_unwrapped). "
            "If omitted, the logger estimates it using --imu-odr-hz and the smallest observed imu_ts delta."
        ),
    )
    parser.add_argument(
        "--imu-odr-hz",
        type=float,
        default=500.0,
        help=(
            "IMU FIFO sample rate in Hz (used to estimate --imu-ts-tick-us when not provided). "
            "Only used in binary batch mode (A5 5A 42)."
        ),
    )

    # Unit conversion for binary logs
    parser.add_argument(
        "--accel-fs-g",
        type=float,
        default=8.0,
        choices=[2.0, 4.0, 8.0, 16.0],
        help="Accel full-scale range in g (used to convert raw counts to m/s^2)",
    )
    parser.add_argument(
        "--gyro-fs-dps",
        type=float,
        default=1000.0,
        choices=[15.625, 31.25, 62.5, 125.0, 250.0, 500.0, 1000.0, 2000.0],
        help="Gyro full-scale range in dps (used to convert raw counts to rad/s)",
    )
    parser.add_argument(
        "--no-unit-convert",
        action="store_true",
        help="Disable writing converted SI columns (m/s^2 and rad/s) in binary mode",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Print every received line and why it was accepted/ignored",
    )
    return parser.parse_args(argv)


def _acc_counts_to_mss(raw: int, accel_fs_g: float) -> float:
    # For ICM42688 16-bit accel output, full-scale maps to +/-32768 counts.
    # LSB/g = 32768 / FS_g
    g = raw / (32768.0 / accel_fs_g)
    return g * 9.80665


def _gyro_counts_to_rads(raw: int, gyro_fs_dps: float) -> float:
    dps = raw / (32768.0 / gyro_fs_dps)
    return dps * (math.pi / 180.0)


def _fmt_float(x: float, ndp: int) -> str:
    # Avoid long binary-float artifacts like 0.07200000000000001 in CSV.
    return f"{x:.{ndp}f}"


def main(argv: list[str]) -> int:
    args = _parse_args(argv)

    if args.binary_icm42688 and args.no_unit_convert and not args.binary_include_raw:
        raise SystemExit(
            "In --binary-icm42688 mode, --no-unit-convert would produce a CSV with no sensor columns unless you also pass --binary-include-raw."
        )

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
    # On Windows, the default encoding is often cp1252 which can crash if we
    # accidentally decode binary into Unicode replacement characters.
    # UTF-8 is a safe default for cross-platform CSV.
    out_fp = open(out_path, "w", newline="", encoding="utf-8")
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

    if args.binary_icm42688:
        bin_header = ["time_s"]
        if args.binary_include_imu_ts:
            bin_header += ["imu_ts", "imu_ts_unwrapped"]
        if args.binary_include_fsync:
            bin_header += ["fsync_ts", "fsync_ts_unwrapped"]
        if args.binary_include_raw:
            bin_header += [
                "ax_raw",
                "ay_raw",
                "az_raw",
                "gx_raw",
                "gy_raw",
                "gz_raw",
            ]
        if not args.no_unit_convert:
            bin_header += ["ax_mss", "ay_mss", "az_mss", "gx_rads", "gy_rads", "gz_rads"]
        if args.add_host_time:
            bin_header = ["host_time_iso", *bin_header]
        writer.writerow(bin_header)
        out_fp.flush()
        accepted_headers += 1
        header_written = True

        SYNC = b"\xA5\x5A"
        SYNC_BATCH = b"\xA5\x5A\x42"
        PAYLOAD_LEN = 16
        # Support both formats:
        # - legacy: SYNC + payload + XOR(1)  -> 19 bytes
        # - current: SYNC + payload + CRC16(2) -> 20 bytes
        FRAME_LEN_XOR = 2 + PAYLOAD_LEN + 1
        FRAME_LEN_CRC = 2 + PAYLOAD_LEN + 2
        unpacker = struct.Struct("<Ihhhhhh")
        buf = bytearray()
        bad_checksum = 0
        ok_xor = 0
        ok_crc = 0
        crc_mode_seen = False
        # For batch packets that contain a 16-bit IMU FIFO timestamp, unwrap it
        # to a monotonic counter.
        last_imu_ts: Optional[int] = None
        imu_wrap_base = 0
        t0_imu_unwrapped: Optional[int] = None
        last_imu_unwrapped: Optional[int] = None
        min_dt_ticks: Optional[int] = None
        imu_tick_us: Optional[float] = args.imu_ts_tick_us
        tick_info_printed = False
        # FSYNC packet timestamp unwrapping
        last_fsync_ts: Optional[int] = None
        fsync_wrap_base = 0
        t0_fsync_unwrapped: Optional[int] = None
        last_fsync_unwrapped: Optional[int] = None

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

                    # If we have a batch marker, decode batch packets.
                    if len(buf) >= idx + 3 and buf[idx : idx + 3] == SYNC_BATCH:
                        # Need at least: sync3 + count + fsync(2) + crc16
                        if len(buf) < idx + 3 + 1 + 2 + 2:
                            if idx > 0:
                                del buf[:idx]
                            break

                        count = buf[idx + 3]
                        if count == 0 or count > 50:
                            bad_checksum += 1
                            del buf[: idx + 1]
                            continue

                        # Body is: count + fsync_ts(u16 LE) + (count * sample)
                        # Each sample is: imu_ts(u16 LE) + 6 axes(int16 LE) => 14 bytes
                        body_len = 1 + 2 + (count * 14)
                        pkt_len = 3 + body_len + 2
                        if len(buf) < idx + pkt_len:
                            if idx > 0:
                                del buf[:idx]
                            break

                        pkt = bytes(buf[idx : idx + pkt_len])
                        body = pkt[3 : 3 + body_len]
                        got_crc = pkt[-2] | (pkt[-1] << 8)
                        calc_crc = _crc16_ccitt(body)
                        if got_crc != calc_crc:
                            bad_checksum += 1
                            del buf[: idx + 1]
                            continue

                        ok_crc += 1
                        crc_mode_seen = True
                        # consume packet
                        del buf[: idx + pkt_len]

                        # Extract FSYNC timestamp (little-endian) placed after count
                        fsync_lo = pkt[4]
                        fsync_hi = pkt[5]
                        fsync_ts = fsync_lo | (fsync_hi << 8)

                        samples = pkt[6 : 6 + (count * 14)]
                        # unwrap fsync timestamp
                        if last_fsync_ts is not None and fsync_ts < last_fsync_ts:
                            fsync_wrap_base += 0x10000
                        last_fsync_ts = fsync_ts
                        fsync_ts_unwrapped = fsync_wrap_base + fsync_ts
                        if t0_fsync_unwrapped is None:
                            t0_fsync_unwrapped = fsync_ts_unwrapped
                        last_fsync_unwrapped = fsync_ts_unwrapped
                        for i in range(count):
                            off = i * 14
                            imu_ts, ax, ay, az, gx, gy, gz = struct.unpack_from("<Hhhhhhh", samples, off)
                            if last_imu_ts is not None and imu_ts < last_imu_ts:
                                imu_wrap_base += 0x10000
                            last_imu_ts = imu_ts
                            imu_ts_unwrapped = imu_wrap_base + imu_ts

                            if t0_imu_unwrapped is None:
                                t0_imu_unwrapped = imu_ts_unwrapped

                            if last_imu_unwrapped is not None:
                                dt_ticks = imu_ts_unwrapped - last_imu_unwrapped
                                if dt_ticks > 0:
                                    if min_dt_ticks is None or dt_ticks < min_dt_ticks:
                                        min_dt_ticks = dt_ticks
                            last_imu_unwrapped = imu_ts_unwrapped

                            # If user did not supply tick period, estimate it from the smallest
                            # observed delta (assumed to correspond to one FIFO sample period).
                            if imu_tick_us is None and min_dt_ticks is not None and min_dt_ticks > 0:
                                imu_tick_us = (1e6 / float(args.imu_odr_hz)) / float(min_dt_ticks)
                                if not tick_info_printed:
                                    tick_info_printed = True
                                    print(
                                        f"Estimated IMU timestamp tick: {imu_tick_us:.6f} us (from min dt_ticks={min_dt_ticks} @ odr={args.imu_odr_hz} Hz). "
                                        "If this looks wrong, set --imu-odr-hz or --imu-ts-tick-us.",
                                        file=sys.stderr,
                                    )

                            # Always start the timebase at exactly 0.0 for the first
                            # observed sample, even if we haven't inferred tick rate yet.
                            time_s = 0.0
                            if imu_tick_us is not None and t0_imu_unwrapped is not None:
                                time_s = (imu_ts_unwrapped - t0_imu_unwrapped) * (imu_tick_us * 1e-6)

                            row: list[object] = [_fmt_float(time_s, 6)]
                            if args.binary_include_imu_ts:
                                row += [imu_ts, imu_ts_unwrapped]
                            if args.binary_include_fsync:
                                row += [fsync_ts, fsync_ts_unwrapped]
                            if args.binary_include_raw:
                                row += [ax, ay, az, gx, gy, gz]
                            if not args.no_unit_convert:
                                row += [
                                    _fmt_float(_acc_counts_to_mss(ax, args.accel_fs_g), 6),
                                    _fmt_float(_acc_counts_to_mss(ay, args.accel_fs_g), 6),
                                    _fmt_float(_acc_counts_to_mss(az, args.accel_fs_g), 6),
                                    _fmt_float(_gyro_counts_to_rads(gx, args.gyro_fs_dps), 6),
                                    _fmt_float(_gyro_counts_to_rads(gy, args.gyro_fs_dps), 6),
                                    _fmt_float(_gyro_counts_to_rads(gz, args.gyro_fs_dps), 6),
                                ]
                            if args.add_host_time:
                                host_time = dt.datetime.now(dt.timezone.utc).isoformat()
                                writer.writerow([host_time, *row])
                            else:
                                writer.writerow(row)
                            accepted_rows += 1
                            if args.echo:
                                if args.add_host_time:
                                    print(",".join(str(x) for x in [host_time, *row]))
                                else:
                                    print(",".join(str(x) for x in row))
                        out_fp.flush()
                        continue

                    need = min(FRAME_LEN_XOR, FRAME_LEN_CRC)
                    if len(buf) < idx + need:
                        # not enough bytes yet; drop leading garbage but keep sync
                        if idx > 0:
                            del buf[:idx]
                        break

                    # Try CRC16 format first (stronger, fewer false positives)
                    if len(buf) >= idx + FRAME_LEN_CRC:
                        frame_crc = bytes(buf[idx : idx + FRAME_LEN_CRC])
                        payload = frame_crc[2 : 2 + PAYLOAD_LEN]
                        crc_lo = frame_crc[2 + PAYLOAD_LEN]
                        crc_hi = frame_crc[2 + PAYLOAD_LEN + 1]
                        got_crc = crc_lo | (crc_hi << 8)
                        calc_crc = _crc16_ccitt(payload)
                        if got_crc == calc_crc:
                            del buf[: idx + FRAME_LEN_CRC]
                            ok_crc += 1
                            crc_mode_seen = True
                            t_ms, ax, ay, az, gx, gy, gz = unpacker.unpack(payload)

                            row2: list[object] = [_fmt_float((t_ms / 1000.0), 6)]
                            if args.binary_include_imu_ts:
                                row2 += ["", ""]
                            if args.binary_include_raw:
                                row2 += [ax, ay, az, gx, gy, gz]
                            if not args.no_unit_convert:
                                row2 += [
                                    _fmt_float(_acc_counts_to_mss(ax, args.accel_fs_g), 6),
                                    _fmt_float(_acc_counts_to_mss(ay, args.accel_fs_g), 6),
                                    _fmt_float(_acc_counts_to_mss(az, args.accel_fs_g), 6),
                                    _fmt_float(_gyro_counts_to_rads(gx, args.gyro_fs_dps), 6),
                                    _fmt_float(_gyro_counts_to_rads(gy, args.gyro_fs_dps), 6),
                                    _fmt_float(_gyro_counts_to_rads(gz, args.gyro_fs_dps), 6),
                                ]
                            if args.add_host_time:
                                host_time = dt.datetime.now(dt.timezone.utc).isoformat()
                                writer.writerow([host_time, *row2])
                            else:
                                writer.writerow(row2)
                            out_fp.flush()
                            accepted_rows += 1
                            if args.echo:
                                if args.add_host_time:
                                    print(",".join(str(x) for x in [host_time, *row2]))
                                else:
                                    print(",".join(str(x) for x in row2))
                            continue

                    # Fallback: legacy XOR format
                    if crc_mode_seen:
                        # Once we know the stream contains CRC16 frames, treat
                        # any XOR-looking frames as garbage. This avoids false
                        # positives when bytes are dropped.
                        bad_checksum += 1
                        del buf[: idx + 1]
                        continue

                    if len(buf) < idx + FRAME_LEN_XOR:
                        if idx > 0:
                            del buf[:idx]
                        break

                    frame_xor = bytes(buf[idx : idx + FRAME_LEN_XOR])
                    payload = frame_xor[2 : 2 + PAYLOAD_LEN]
                    checksum = frame_xor[-1]
                    calc = 0
                    for b in payload:
                        calc ^= b

                    if calc != checksum:
                        bad_checksum += 1
                        # bad frame; drop one byte and rescan
                        del buf[: idx + 1]
                        continue

                    del buf[: idx + FRAME_LEN_XOR]
                    ok_xor += 1

                    t_ms, ax, ay, az, gx, gy, gz = unpacker.unpack(payload)

                    row3: list[object] = [_fmt_float((t_ms / 1000.0), 6)]
                    if args.binary_include_imu_ts:
                        row3 += ["", ""]
                    if args.binary_include_raw:
                        row3 += [ax, ay, az, gx, gy, gz]
                    if not args.no_unit_convert:
                        row3 += [
                            _fmt_float(_acc_counts_to_mss(ax, args.accel_fs_g), 6),
                            _fmt_float(_acc_counts_to_mss(ay, args.accel_fs_g), 6),
                            _fmt_float(_acc_counts_to_mss(az, args.accel_fs_g), 6),
                            _fmt_float(_gyro_counts_to_rads(gx, args.gyro_fs_dps), 6),
                            _fmt_float(_gyro_counts_to_rads(gy, args.gyro_fs_dps), 6),
                            _fmt_float(_gyro_counts_to_rads(gz, args.gyro_fs_dps), 6),
                        ]
                    if args.add_host_time:
                        host_time = dt.datetime.now(dt.timezone.utc).isoformat()
                        writer.writerow([host_time, *row3])
                    else:
                        writer.writerow(row3)

                    out_fp.flush()
                    accepted_rows += 1
                    if args.echo:
                        if args.add_host_time:
                            print(",".join(str(x) for x in [host_time, *row3]))
                        else:
                            print(",".join(str(x) for x in row3))

        except KeyboardInterrupt:
            print("\nStopped.")
            print(
                f"Summary: headers={accepted_headers}, rows={accepted_rows}, ok_crc={ok_crc}, ok_xor={ok_xor}, bad_checksum={bad_checksum}"
            )
            if accepted_rows == 0:
                print(
                    "No binary frames decoded. Common causes: UNO not in binary bridge mode, ATmega not sending binary frames, wrong baud, or RS485 link not running. Try running with --debug on the UNO (CSV mode) to sanity-check the link, or verify the sender is using SYNC A5 5A."
                )
            return 0

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            if not _is_reasonably_printable_ascii(line):
                if args.debug:
                    print("IGNORED (non-ascii; looks like binary). If using FIFO binary mode, add --binary-icm42688")
                ignored_lines += 1
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
            accepted_rows += 1

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
