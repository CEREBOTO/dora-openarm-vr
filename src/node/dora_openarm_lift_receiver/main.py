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
Minimal UDP receiver for lift teleoperation.
Extracts left-controller X and Y button states from the Quest VR JSON stream.
"""

import argparse
import time

import dora
import pyarrow as pa

from utils.udp_receiver import JsonUdpReceiver

_DEFAULT_HOST = "0.0.0.0"
_DEFAULT_PORT = 5006


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

        node.send_output(
            "button_x",
            pa.array([bool(msg.get("x", False))], type=pa.bool_()),
            ts,
        )
        node.send_output(
            "button_y",
            pa.array([bool(msg.get("y", False))], type=pa.bool_()),
            ts,
        )

    receiver.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Lift teleop: extract Quest X/Y buttons (dora node)"
    )
    parser.add_argument("--host", default=_DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=_DEFAULT_PORT)
    args = parser.parse_args()
    _run(args)


if __name__ == "__main__":
    main()
