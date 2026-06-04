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
Dora node that converts Quest VR thumbstick axes into SocketCAN frames
for chassis teleoperation.  No ROS 2 dependency — writes directly to can0.

Gear logic:
  Both sticks neutral (|value| < deadzone) → gear 1 (park), zero velocity
  Either stick active                        → gear 6 (four-wheel steer)

Joystick mapping:
  left_stick_y  (Quest lsy)  →  ctrl_cmd_x_linear  (forward/backward, m/s)
  right_stick_x (Quest rsx)  →  ctrl_cmd_z_angular  (steering, deg/s)

CAN frame format (matching can_control_node ctrl_cmd_callback):
  CAN ID  0x98C4D1D0, DLC 8
  Byte 0: gear(4b) | x_linear[3:0](4b)
  Byte 1: x_linear[11:4]
  Byte 2: x_linear[15:12](4b) | z_angular[3:0](4b)
  Byte 3: z_angular[11:4]
  Byte 4: z_angular[15:12](4b) | y_linear[3:0](4b)
  Byte 5: y_linear[11:4]
  Byte 6: y_linear[15:12](4b) | rolling_counter(4b)
  Byte 7: XOR of bytes 0–6
"""

import argparse
import os
import socket
import struct

import dora
import numpy as np
import pyarrow as pa

_GEAR_PARK = 1
_GEAR_FOUR_WHEEL_STEER = 6

_CAN_ID_CTRL_CMD = 0x98C4D1D0

# struct can_frame on 64-bit Linux (little-endian, no inter-element padding)
# canid_t can_id (u32) + __u8 len + __u8 __pad + __u8 __res0 + __u8 len8_dlc + data[8]
_CAN_FRAME_FMT = "<IB3x8s"


# ── helpers for SocketCAN ────────────────────────────────────────────────────

AF_CAN = 29  # Linux AF_CAN
PF_CAN = AF_CAN
CAN_RAW = 1


def _open_can_socket(if_name: str) -> socket.socket:
    sock = socket.socket(PF_CAN, socket.SOCK_RAW, CAN_RAW)
    sock.bind((if_name,))
    return sock


# ── CAN frame encoding ───────────────────────────────────────────────────────

_count = 0


def _encode_ctrl_cmd(gear: int, x_linear: float, y_linear: float, z_angular: float) -> bytes:
    """Pack gear + velocities into an 8-byte CAN payload matching ctrl_cmd_callback."""
    global _count

    x = int(x_linear * 1000)   # m/s  → mm/s
    y = int(y_linear * 1000)
    z = int(z_angular * 100)   # deg/s → 0.01 deg/s

    d = bytearray(8)

    # Byte 0: gear(lo nibble) | x[3:0](hi nibble)
    d[0] = (gear & 0x0F) | ((x & 0x0F) << 4)
    # Byte 1: x[11:4]
    d[1] = (x >> 4) & 0xFF
    # Byte 2: x[15:12](lo nibble) | z[3:0](hi nibble)
    d[2] = ((x >> 12) & 0x0F) | ((z & 0x0F) << 4)
    # Byte 3: z[11:4]
    d[3] = (z >> 4) & 0xFF
    # Byte 4: z[15:12](lo nibble) | y[3:0](hi nibble)
    d[4] = ((z >> 12) & 0x0F) | ((y & 0x0F) << 4)
    # Byte 5: y[11:4]
    d[5] = (y >> 4) & 0xFF
    # Byte 6: y[15:12](lo nibble) | counter(hi nibble)
    _count = (_count + 1) & 0x0F
    d[6] = ((y >> 12) & 0x0F) | (_count << 4)
    # Byte 7: XOR checksum
    d[7] = d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[5] ^ d[6]

    return bytes(d)


# ── main ─────────────────────────────────────────────────────────────────────

def _run(args: argparse.Namespace) -> None:
    deadzone = args.deadzone
    max_linear = args.max_linear
    max_angular = args.max_angular
    can_if = args.can_if

    can_sock = _open_can_socket(can_if)
    print(f"[chassis-controller] Opened {can_if}")

    node = dora.Node()
    node.send_output("status", pa.array(["ready"]))

    prev_left_stick_y = 0.0
    prev_right_stick_x = 0.0

    try:
        for event in node:
            if event["type"] == "INPUT":
                if event["id"] == "left_stick_y":
                    prev_left_stick_y = float(event["value"][0].as_py())
                elif event["id"] == "right_stick_x":
                    prev_right_stick_x = float(event["value"][0].as_py())
                elif event["id"] != "tick":
                    continue

            if event["type"] == "STOP":
                break

            if event["id"] != "tick":
                continue

            left_val = prev_left_stick_y
            right_val = prev_right_stick_x

            if abs(left_val) < deadzone and abs(right_val) < deadzone:
                gear = _GEAR_PARK
                x_lin = 0.0
                z_ang = 0.0
            else:
                gear = _GEAR_FOUR_WHEEL_STEER
                x_lin = float(np.clip(left_val * max_linear, -max_linear, max_linear))
                z_ang = float(np.clip(right_val * max_angular, -max_angular, max_angular))

            payload = _encode_ctrl_cmd(gear, x_lin, 0.0, z_ang)
            frame = struct.pack(_CAN_FRAME_FMT, _CAN_ID_CTRL_CMD, 8, payload)
            can_sock.send(frame)

    finally:
        # Send stop frame (park, zero velocity)
        payload = _encode_ctrl_cmd(_GEAR_PARK, 0.0, 0.0, 0.0)
        frame = struct.pack(_CAN_FRAME_FMT, _CAN_ID_CTRL_CMD, 8, payload)
        try:
            can_sock.send(frame)
        except OSError:
            pass
        can_sock.close()
        print("[chassis-controller] CAN socket closed")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Chassis teleop: joysticks → SocketCAN (dora node, no ROS 2)",
    )
    parser.add_argument("--can-if", default="can0",
                        help="SocketCAN interface name")
    parser.add_argument("--max-linear", type=float, default=0.3,
                        help="Max forward speed (m/s)")
    parser.add_argument("--max-angular", type=float, default=40.0,
                        help="Max steering speed (deg/s)")
    parser.add_argument("--deadzone", type=float, default=0.05,
                        help="Joystick deadzone threshold")
    args = parser.parse_args()
    _run(args)


if __name__ == "__main__":
    main()
