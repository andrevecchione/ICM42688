# Serial CSV Logger

Logs the UNO's USB serial output into a `.csv` file.

## Install

```bash
python3 -m pip install -r tools/requirements.txt
```

If you see `No module named pip`, install pip first (Ubuntu/Debian):

```bash
sudo apt update
sudo apt install -y python3-pip
```

Or try Python's built-in bootstrap (may not be present on all distros):

```bash
python3 -m ensurepip --user
```

## Run

Find your UNO port (Linux examples: `/dev/ttyACM0`, `/dev/ttyUSB0`). Then:

```bash
python3 tools/serial_csv_logger.py --port /dev/ttyACM0 --baud 115200
```

Note: `--baud` must match the UNO's USB `Serial.begin(...)` baud (UNO->PC). This is independent of the RS485 baud.

If you're using the high-speed binary bridge mode, the UNO sketch may be set to a higher USB baud (e.g. 500000). In that case, pass the same value here.

### Windows

cd \\wsl.localhost\Ubuntu-22.04\home\andre\ICM42688

If you're using Windows (or WSL USBIP is flaky), run the logger on Windows directly using the COM port:

```powershell
python -m pip install -r tools/requirements.txt
python tools\serial_csv_logger.py --port COM13 --baud 115200
```

If you're running the FIFO *binary* sketches, add `--binary-icm42688`:

```powershell
python tools\serial_csv_logger.py --port COM13 --baud 115200 --binary-icm42688
```

It will create a timestamped file like `imu_log_YYYYmmdd_HHMMSS.csv` in the current directory.

### Options

- Add a host timestamp column:

```bash
python3 tools/serial_csv_logger.py --port /dev/ttyACM0 --add-host-time
```

- Echo accepted CSV frames to the console while logging:

```bash
python3 tools/serial_csv_logger.py --port /dev/ttyACM0 --echo
```

- Require seeing a CSV header before logging numeric rows:

```bash
python3 tools/serial_csv_logger.py --port /dev/ttyACM0 --require-header
```

## If the CSV file is empty

Run with debug enabled to see what the script is receiving and why lines are ignored:

```bash
python3 tools/serial_csv_logger.py --port /dev/ttyACM0 --baud 115200 --debug
```

If you only see data in Arduino IDE Serial Monitor but not here, try forcing an Arduino reset and/or not discarding the startup banner:

```bash
python3 tools/serial_csv_logger.py --port /dev/ttyACM0 --baud 115200 --toggle-dtr --reset-wait 3 --debug --echo
```

If you *do* want to discard any startup text after reset:

```bash
python3 tools/serial_csv_logger.py --port /dev/ttyACM0 --baud 115200 --flush-input
```

The logger accepts both comma-separated and tab-separated telemetry lines.

## Binary mode (500 Hz+)

If you switch the sketches to the FIFO binary packet mode, run:

```bash
python3 tools/serial_csv_logger.py --port /dev/ttyACM0 --baud 500000 --binary-icm42688
```

By default, this writes a minimal CSV that’s easy to plot:
- `time_s` (starts at 0.0)
- accel in `m/s^2`: `ax_mss..az_mss`
- gyro in `rad/s`: `gx_rads..gz_rads`

If you also want the raw counts in the CSV, add:

```bash
python3 tools/serial_csv_logger.py --port /dev/ttyACM0 --baud 500000 --binary-icm42688 --binary-include-raw
```

If you want the IMU FIFO timestamp columns (`imu_ts`, `imu_ts_unwrapped`) too, add:

```bash
python3 tools/serial_csv_logger.py --port /dev/ttyACM0 --baud 500000 --binary-icm42688 --binary-include-imu-ts
```

The conversion depends on the full-scale ranges used in your sender sketch.
Defaults are `--accel-fs-g 8` and `--gyro-fs-dps 1000` (matching `gpm8` and `dps1000`).

To override:

```bash
python3 tools/serial_csv_logger.py --port /dev/ttyACM0 --baud 115200 --binary-icm42688 --accel-fs-g 16 --gyro-fs-dps 2000
```

Notes:
- The logger accepts both the legacy XOR checksum frames and the newer CRC16 frames.
- If you see lots of `bad_checksum`, the RS485/AltSoftSerial link is dropping bytes; CRC16 prevents most *false* rows, but you may still need to improve wiring/termination or reduce throughput.
