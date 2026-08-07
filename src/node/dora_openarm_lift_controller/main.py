# Copyright 2026 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Dora node that converts Quest VR left-controller X/Y button states
into serial commands for the Pico-based lift controller.

Button mapping:
  Left X held → up
  Left Y held → down
  Both held   → ignored (safety: no conflict)
  Neither     → stop

Protocol: text lines terminated by \n over USB serial at 115200 baud.
Matching the Pico firmware expected by lift/control.py.
"""

import argparse
import re
import time

import dora
import pyarrow as pa
import serial


def _send(ser: serial.Serial, cmd: str) -> None:
    ser.write((cmd + "\n").encode())
    ser.flush()


def _read_available(ser: serial.Serial) -> str | None:
    if ser.in_waiting:
        line = ser.readline().decode(errors="replace").strip()
        if line:
            return line
    return None


def _parse_height(line: str) -> float | None:
    m = re.search(r"[\d.]+", line)
    if m:
        try:
            return float(m.group())
        except ValueError:
            return None
    return None


def _open_serial(
    port: str,
    baud: int,
    retries: int = 5,
    retry_delay: float = 0.5,
) -> serial.Serial:
    """Open the Pico serial port, tolerating USB reset and DTR quirks."""
    last_error: OSError | serial.SerialException | None = None

    for attempt in range(1, retries + 1):
        ser: serial.Serial | None = None
        try:
            ser = serial.Serial(port, baud, timeout=0.2)

            # Some Pico USB CDC firmware does not implement modem-control
            # requests. Failure to change DTR/RTS does not make serial I/O
            # unusable, so continue with the state selected during open.
            try:
                ser.setDTR(False)
                ser.setRTS(False)
            except (AttributeError, OSError, serial.SerialException) as exc:
                print(f"[lift-controller] DTR/RTS unavailable: {exc}")

            # Opening or toggling DTR can briefly reset/re-enumerate the Pico.
            time.sleep(0.25)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            return ser
        except (OSError, serial.SerialException) as exc:
            last_error = exc
            if ser is not None and ser.is_open:
                ser.close()
            if attempt == retries:
                break
            print(
                f"[lift-controller] Serial open failed "
                f"({attempt}/{retries}): {exc}; retrying ..."
            )
            time.sleep(retry_delay)

    raise serial.SerialException(
        f"Could not open {port} after {retries} attempts: {last_error}"
    )


def _run(args: argparse.Namespace) -> None:
    ser = _open_serial(
        args.port,
        args.baud,
        retries=args.open_retries,
        retry_delay=args.retry_delay,
    )
    print(f"[lift-controller] Opened {args.port} @ {args.baud}")

    node = dora.Node()
    node.send_output("status", pa.array(["ready"]))

    prev_button_x = False
    prev_button_y = False

    # Track when we last sent a command to avoid flooding the serial line
    last_send_time = 0.0
    _SEND_COOLDOWN = 0.05  # 50 ms

    # Height polling state
    last_height_line = None
    last_poll_time = 0.0
    poll_interval = args.poll_interval

    # Height limits (mm)
    _HEIGHT_MIN = 0
    _HEIGHT_MAX = 600

    try:
        for event in node:
            if event["type"] == "INPUT":
                if event["id"] == "button_x":
                    prev_button_x = bool(event["value"][0].as_py())
                elif event["id"] == "button_y":
                    prev_button_y = bool(event["value"][0].as_py())
                elif event["id"] != "tick":
                    continue

            if event["type"] == "STOP":
                break

            if event["id"] != "tick":
                continue

            now = time.monotonic()

            # ── periodic height poll ──
            while (line := _read_available(ser)) is not None:
                h = _parse_height(line)
                if h is not None and h != last_height_line:
                    print(f"[lift] Height: {h} mm")
                    last_height_line = h

            if now - last_poll_time >= poll_interval:
                _send(ser, "get")
                last_poll_time = now

            if now - last_send_time < _SEND_COOLDOWN:
                continue

            if prev_button_x and prev_button_y:
                # Both pressed simultaneously — no-op for safety
                pass
            elif prev_button_x:
                if last_height_line is not None and last_height_line >= _HEIGHT_MAX:
                    _send(ser, "stop")
                    last_send_time = now
                else:
                    _send(ser, "up")
                    last_send_time = now
            elif prev_button_y:
                if last_height_line is not None and last_height_line <= _HEIGHT_MIN:
                    _send(ser, "stop")
                    last_send_time = now
                else:
                    _send(ser, "down")
                    last_send_time = now
            else:
                _send(ser, "stop")
                last_send_time = now

    finally:
        try:
            _send(ser, "stop")
        except (OSError, serial.SerialException):
            pass
        ser.close()
        print("[lift-controller] Serial closed")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Lift teleop: X/Y buttons → serial commands (dora node)",
    )
    parser.add_argument("--port", default="/dev/ttyACM0",
                        help="Serial port for Pico lift controller")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baud rate")
    parser.add_argument("--poll-interval", type=float, default=0.5,
                        help="Interval in seconds between get height polls")
    parser.add_argument("--open-retries", type=int, default=5,
                        help="Number of serial open attempts")
    parser.add_argument("--retry-delay", type=float, default=0.5,
                        help="Delay between serial open attempts in seconds")
    args = parser.parse_args()
    _run(args)


if __name__ == "__main__":
    main()
