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

"""Node to control OpenArm."""

import argparse
import dataclasses
import dora
try:
    import openarm_driver
except ImportError:
    import local_openarm_driver as openarm_driver
import os
import pathlib
import pyarrow as pa
import numpy as np


@dataclasses.dataclass
class AlignState:
    """State for alignment."""

    align_target: np.ndarray = None
    step_limit: float = 0.001


def _align(arm, state, new_position, name, threshold, trigger=None):
    """Safety: Align OpenArm with the position."""
    if trigger == "gripper":
        # v1: prismatic finger (slide), range [0, 0.044] m
        # gripping = gripper near 0 (closed position, < 5 mm)
        gripper_position = new_position[-1]  # Last value is gripper's position
        is_gripping = gripper_position.as_py() < 0.005
        if not is_gripping:
            return False

    def current_position():
        return np.array(arm.fetch_position(), dtype=np.float32)

    if state.align_target is None:
        state.align_target = current_position()

    def is_aligned(position1, position2):
        return np.all(np.abs(position1 - position2) < threshold)

    # If OpenArm is already aligned, we do nothing.
    if is_aligned(new_position, current_position()):
        return True

    diff = new_position - state.align_target
    step_move = np.clip(diff, -state.step_limit, state.step_limit)
    state.align_target += step_move

    arm.send_position(state.align_target)

    return is_aligned(new_position, current_position())


def _env_flag(name, default=False):
    v = os.getenv(name)
    if v is None:
        return default
    return v.strip().lower() in ("1", "true", "yes", "on")


def main():
    """Move to the given position and output the current position."""
    parser = argparse.ArgumentParser(description="Control OpenArm")
    parser.add_argument(
        "--side",
        choices=["right", "left"],
        default="right",
        help="right or left",
    )
    parser.add_argument(
        "--config",
        default=None,
        help="The configuration file for this OpenArm",
        type=pathlib.Path,
    )
    parser.add_argument(
        "--align-trigger",
        choices=["gripper"],
        default=None,
        help="Alignment trigger: gripper (default: None)",
    )
    parser.add_argument(
        "--align-threshold",
        default=0.1,
        help="Alignment threshold [rad] (default: 0.1)",
        type=float,
    )
    parser.add_argument(
        "--stop",
        action=argparse.BooleanOptionalAction,
        default=_env_flag("STOP", True),
        help="Stop the arm on exit.",
    )
    parser.add_argument(
        "--refresh-every-request",
        action=argparse.BooleanOptionalAction,
        default=_env_flag("REFRESH", True),
        help="Refresh OpenArm on every request to make it more accurate.",
    )
    args = parser.parse_args()
    node = dora.Node()
    name = f"{args.side}_arm"
    config = openarm_driver.Config(args.config)
    arm = openarm_driver.SingleArmDriver(name, config)
    arm.start()

    initialized = False
    last_sent = None  # track last position actually sent to hardware
    init_tick = 0     # count ticks since init for gradual acceleration
    rate_limit = 0.03  # current rate limit (rad/tick), gently ramps up
    align_state = AlignState()
    for event in node:
        if event["type"] != "INPUT":
            continue

        # Main process
        event_id = event["id"]
        if event_id == "request_position":
            current_position = arm.fetch_position(
                refresh=args.refresh_every_request,
            )

            node.send_output(
                "position",
                pa.array(current_position, type=pa.float32()),
            )
        elif event_id == "request_state":
            state = arm.fetch_state(refresh=args.refresh_every_request)
            node.send_output(
                "state",
                pa.StructArray.from_arrays(
                    [
                        pa.array(state["qpos"], type=pa.float32()),
                        pa.array(state["qvel"], type=pa.float32()),
                        pa.array(state["qtorque"], type=pa.float32()),
                    ],
                    names=["qpos", "qvel", "qtorque"],
                ),
            )
        elif event_id == "move_position":
            value = event["value"]
            if isinstance(value, pa.StructArray):
                new_position = value.field("new_position")
                # TODO: We use this for safety check later.
                # other_arm_position = value.field("other_arm_position")
            else:
                new_position = value
                # other_arm_position = None
            if not initialized:
                if args.align_trigger is not None:
                    initialized = _align(
                        arm,
                        align_state,
                        new_position,
                        name,
                        args.align_threshold,
                        trigger=args.align_trigger,
                    )
                    if initialized:
                        node.send_output("status", pa.array(["ready"]))
                    continue
                else:
                    # No align trigger: seed last_sent with current motor position.
                    # Start with a tiny rate limit and gently ramp it up over ~2 seconds
                    # to avoid any sudden velocity change on startup.
                    initialized = True
                    init_tick = 0
                    rate_limit = 0.03  # ≈1.5 rad/s
                    last_sent = np.array(arm.fetch_position(), dtype=np.float32)
                    node.send_output("status", pa.array(["ready"]))
                    # fall through to normal tracking below

            target = np.array(new_position, dtype=np.float32)

            # If any joint jumped > 0.3 rad (likely STALE→OK or intermittent tracking glitch),
            # reset the acceleration ramp to avoid a velocity spike.
            if last_sent is not None:
                jump = np.max(np.abs(target - last_sent))
                if jump > 0.3:
                    init_tick = 0
                    rate_limit = 0.03

            # Gradually ramp rate_limit from 0.03 → 0.15 rad/tick over 40 ticks (~0.8s at 50Hz)
            # This prevents a velocity jump on startup: if your hand is far from home,
            # the arm accelerates smoothly instead of lurching.
            if init_tick < 40:
                init_tick += 1
                target_limit = 0.03 + 0.12 * (init_tick / 40)  # 0.03 → 0.15
                if target_limit > rate_limit:
                    rate_limit = target_limit
            else:
                rate_limit = 0.15  # full speed after warmup

            # Proportional rate-limit with gradual velocity cap
            if last_sent is not None:
                diff = target - last_sent
                max_step = np.max(np.abs(diff))
                if max_step > rate_limit:
                    target = last_sent + diff * (rate_limit / max_step)

            arm.send_position(target)
            last_sent = target.copy()
    if args.stop:
        arm.stop()
    else:
        arm.on_start()


if __name__ == "__main__":
    main()
