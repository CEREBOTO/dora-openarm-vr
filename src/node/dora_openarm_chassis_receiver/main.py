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
Minimal UDP receiver for chassis teleoperation.
Extracts left-stick Y (forward/back) and right-stick X (steering)
from the Quest VR JSON stream on each dora tick.
"""

import argparse
import time

import dora
import pyarrow as pa

from utils.udp_receiver import JsonUdpReceiver

_DEFAULT_HOST = "0.0.0.0"
_DEFAULT_PORT = 5006
_DEADZONE = 0.05


def _clamp_deadzone(value: float, deadzone: float = _DEADZONE) -> float:
    if abs(value) < deadzone:
        return 0.0
    return value


def _run(args: argparse.Namespace) -> None:
    receiver = JsonUdpReceiver(args.host, args.port)

    node = dora.Node()
    node.send_output("status", pa.array(["ready"]))

    for event in node:
        if event["type"] != "INPUT" or event["id"] != "tick":
            continue

        msg = receiver.latest()
        if msg is None:
            continue

        ts = {"timestamp": time.time_ns()}

        lsy = float(msg.get("lsy", 0.0))
        rsx = float(msg.get("rsx", 0.0))

        node.send_output(
            "left_stick_y",
            pa.array([_clamp_deadzone(lsy)], type=pa.float32()),
            ts,
        )
        node.send_output(
            "right_stick_x",
            pa.array([_clamp_deadzone(rsx)], type=pa.float32()),
            ts,
        )

    receiver.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Chassis teleop: extract Quest joystick axes (dora node)"
    )
    parser.add_argument("--host", default=_DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=_DEFAULT_PORT)
    args = parser.parse_args()
    _run(args)


if __name__ == "__main__":
    main()
