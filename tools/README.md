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

Note: `--baud` must match the `Serial.begin(...)` baud in your UNO receiver sketch.

### Windows

If you're using Windows (or WSL USBIP is flaky), run the logger on Windows directly using the COM port:

```powershell
py -m pip install -r tools/requirements.txt
py tools/serial_csv_logger.py --port COM13 --baud 115200
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
python3 tools/serial_csv_logger.py --port /dev/ttyACM0 --baud 115200 --binary-icm42688
```

This writes decoded *raw* counts to CSV columns: `ax_raw..gz_raw`.
