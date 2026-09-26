#!/usr/bin/env python
"""Stand-in for the AlohaMini Host (alohamini_host.py) so the bridge's robot mode can be tested
without hardware. Speaks the same ZMQ protocol: PULL commands on 5555, ROUTER observations on
5556, and uses the real CommandOwner for single-writer control.

    python hybrid_twin/web/mock_host.py
    python hybrid_twin/web/server.py --robot-ip 127.0.0.1
    python hybrid_twin/web/test_bridge.py

Physics is idealised: the base follows commanded velocities instantly; the lift runs the real
LiftAxis velocity loop (kp 300 ticks/s per mm, cap 1300 ticks/s, 1 mm on-target band).
"""

import argparse
import importlib.util
import json
import math
import time
from pathlib import Path
from uuid import uuid4

import zmq

REPO = Path(__file__).resolve().parents[2]
_spec = importlib.util.spec_from_file_location(
    "command_owner", REPO / "src" / "lerobot" / "robots" / "alohamini" / "command_owner.py")
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)
CommandOwner = _mod.CommandOwner

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
ARM_KEYS = [f"arm_{side}_{j}.pos" for side in ("left", "right") for j in ARM_JOINTS]
ACTION_KEYS = set(ARM_KEYS) | {"x.vel", "y.vel", "theta.vel", "lift_axis.height_mm"}
KP_VEL, V_MAX, ON_TARGET_MM = 300.0, 1300.0, 1.0


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--cmd-port", type=int, default=5555)
    p.add_argument("--obs-port", type=int, default=5556)
    p.add_argument("--lead-mm-per-rev", type=float, default=84.0)
    p.add_argument("--wave", action="store_true", help="slowly move the arms, as if teleop were running")
    a = p.parse_args()

    ctx = zmq.Context()
    cmd = ctx.socket(zmq.PULL)
    cmd.setsockopt(zmq.CONFLATE, 1)
    cmd.bind(f"tcp://*:{a.cmd_port}")
    obs_sock = ctx.socket(zmq.ROUTER)
    obs_sock.bind(f"tcp://*:{a.obs_port}")

    session = uuid4().hex
    owner = CommandOwner()
    mm_per_tick = a.lead_mm_per_rev / 4096
    height, lift_v_ticks = 0.0, 0.0
    vel = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
    arms = dict.fromkeys(ARM_KEYS, 0.0)
    arms["arm_left_gripper.pos"] = arms["arm_right_gripper.pos"] = 10.0
    last_cmd_t, has_cmd, watchdog_active, target_source = time.monotonic(), False, False, "none"
    metadata = {
        "schema_version": 1, "robot_model": "alohamini1", "cameras": [],
        "motors": {k.removesuffix(".pos"): {"id": i + 1, "model": "sts3215",
                                             "normalization": "range_0_100" if k.endswith("gripper.pos") else "range_m100_100",
                                             "drive_mode": 0, "range_min": 1024, "range_max": 3072}
                   for i, k in enumerate(ARM_KEYS)},
        "lift_axis": {"soft_min_mm": 0.0, "soft_max_mm": 600.0, "descent_floor_mm": 5.0},
    }
    print(f"mock AlohaMini Host on :{a.cmd_port} (cmd) / :{a.obs_port} (obs), session {session[:8]}")

    dt, t0 = 1 / 50, time.monotonic()
    while True:
        loop_t = time.monotonic()
        if has_cmd and loop_t - last_cmd_t > 1.0 and not watchdog_active:
            print("watchdog: stopping motion and releasing control")
            vel = dict.fromkeys(vel, 0.0)
            lift_v_ticks = 0.0
            watchdog_active, has_cmd, target_source = True, False, "watchdog"
            owner.release()
        try:
            data = json.loads(cmd.recv_string(zmq.NOBLOCK))
            meta = data.pop("_command", {})
            action = {k: float(v) for k, v in data.items()}
            if not all(k in ACTION_KEYS and math.isfinite(v) for k, v in action.items()):
                raise ValueError(f"invalid keys {set(action) - ACTION_KEYS}")
            if owner.accept(meta, session):
                vel = {k: action.get(k, 0.0) for k in vel}
                if "lift_axis.height_mm" in action:
                    err = min(max(action["lift_axis.height_mm"], 0.0), 600.0) - height
                    lift_v_ticks = 0.0 if abs(err) <= ON_TARGET_MM else max(-V_MAX, min(V_MAX, KP_VEL * err))
                arms.update({k: v for k, v in action.items() if k in arms})
                last_cmd_t, has_cmd, watchdog_active, target_source = loop_t, True, False, "command"
        except zmq.Again:
            pass
        except Exception as e:  # the real Host logs and carries on
            print("rejected command:", e)

        height = min(max(height + lift_v_ticks * mm_per_tick * dt, 0.0), 600.0)
        if a.wave:
            ph = loop_t - t0
            arms["arm_left_shoulder_pan.pos"] = 30 * math.sin(ph * 0.8)
            arms["arm_right_shoulder_pan.pos"] = -30 * math.sin(ph * 0.8)
            arms["arm_left_elbow_flex.pos"] = 20 * math.sin(ph * 0.5)

        try:
            while True:
                identity, token = obs_sock.recv_multipart(zmq.NOBLOCK)[:2]
                state = {**arms, **vel, "lift_axis.height_mm": height, "_image_encoding": "jpeg", "_images": [],
                         "_robot_metadata": metadata,
                         "_safety": {"version": 1, "host_session_id": session, "watchdog_active": watchdog_active,
                                     "command_watchdog_timeout_s": 1.0, "control_owner": owner.owner,
                                     "control_epoch": owner.epoch, "target_source": target_source}}
                obs_sock.send_multipart([identity, token, json.dumps(state).encode()], zmq.NOBLOCK)
        except zmq.Again:
            pass
        time.sleep(max(0.0, dt - (time.monotonic() - loop_t)))


if __name__ == "__main__":
    main()
