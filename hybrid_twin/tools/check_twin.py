#!/usr/bin/env python
"""Smoke-test the generated twin: load it, drive it with real AlohaMini action keys, assert motion.

    python hybrid_twin/tools/check_twin.py            # headless checks
    python hybrid_twin/tools/check_twin.py --viewer   # also open the MuJoCo viewer (WASD-style demo)
"""

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from twin_driver import TwinDriver  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
MJCF = ROOT / "mujoco" / "alohamini_twin.xml"
URDF = ROOT / "urdf" / "alohamini_twin.urdf"


def check_mjcf() -> None:
    model = mujoco.MjModel.from_xml_path(str(MJCF))
    data = mujoco.MjData(model)
    twin = TwinDriver(model, data)
    print(f"MJCF ok: {model.njnt} joints, {model.nu} actuators, {model.nmesh} meshes")

    # 1) drive forward 0.2 m/s for 2 s (key "w" at medium speed).
    twin.run({"x.vel": 0.2}, seconds=2.0)
    x, y, yaw = twin.base_pose()
    assert abs(x - 0.4) < 0.03 and abs(y) < 0.01, f"forward drive wrong: {x=:.3f} {y=:.3f}"

    # 2) rotate 45 deg/s for 2 s (key "a"), theta.vel is in deg/s like the real robot.
    twin.run({"theta.vel": 45.0}, seconds=2.0)
    x, y, yaw = twin.base_pose()
    assert abs(math.degrees(yaw) - 90) < 3, f"rotation wrong: {math.degrees(yaw):.1f} deg"

    # 3) forward again: now facing +y in the world, so the body-frame command must move along world y.
    twin.run({"x.vel": 0.2}, seconds=1.0)
    x2, y2, _ = twin.base_pose()
    assert abs(x2 - x) < 0.03 and abs((y2 - y) - 0.2) < 0.03, "body->world frame conversion wrong"

    # 4) lift to 300 mm (the lift lever).
    twin.run({"lift_axis.height_mm": 300.0}, seconds=2.0)
    h = twin.lift_height_mm()
    assert abs(h - 300) < 10, f"lift wrong: {h:.1f} mm"

    # 5) arms: joint targets in radians track (teleop feeds these on the real robot).
    targets = {"left_shoulder_lift": -0.6, "right_elbow_flex": 0.8, "left_gripper": 1.0}
    twin.set_arm_radians(targets)
    twin.run({}, seconds=1.5)
    for joint, target in targets.items():
        actual = twin.joint(joint)
        assert abs(actual - target) < 0.1, f"{joint}: {actual:.2f} vs {target}"

    # Lift/arm-only commands carry no base keys, so the base must not have drifted.
    x3, y3, _ = twin.base_pose()
    assert abs(x3 - x2) < 0.01 and abs(y3 - y2) < 0.01, f"base drifted during lift/arm moves: {x3=:.3f} {y3=:.3f}"

    # 6) watchdog: no command -> base stops (mirrors the host's 1 s watchdog).
    twin.run({"x.vel": 0.2}, seconds=0.5)
    twin.run({}, seconds=1.5, send=False)
    assert np.allclose(twin.base_velocity(), 0, atol=1e-3), "base kept moving after watchdog"

    print(f"MJCF drive checks passed: pose={tuple(round(v, 3) for v in twin.base_pose())}, lift={h:.0f} mm")


def check_urdf() -> None:
    root = ET.parse(URDF).getroot()
    links = {lk.get("name") for lk in root.findall("link")}
    children = {j.find("child").get("link") for j in root.findall("joint")}
    roots = links - children
    assert roots == {"world"}, f"URDF must have one root, found {roots}"
    missing = [m.get("filename") for m in root.iter("mesh") if not (URDF.parent / m.get("filename")).is_file()]
    assert not missing, f"missing meshes: {missing[:3]}"
    movable = [j.get("name") for j in root.findall("joint") if j.get("type") != "fixed"]
    print(f"URDF ok: {len(links)} links, {len(movable)} movable joints, all "
          f"{sum(1 for _ in root.iter('mesh'))} mesh refs resolve")


def run_viewer() -> None:
    import mujoco.viewer

    model = mujoco.MjModel.from_xml_path(str(MJCF))
    data = mujoco.MjData(model)
    twin = TwinDriver(model, data)
    # Scripted tour: square drive, spin, lift up/down, wave the arms.
    script = [({"x.vel": 0.2}, 2), ({"y.vel": 0.2}, 2), ({"theta.vel": 60.0}, 3),
              ({"lift_axis.height_mm": 450.0}, 2), ({"lift_axis.height_mm": 50.0}, 2)]
    with mujoco.viewer.launch_passive(model, data) as v:
        while v.is_running():
            for action, seconds in script:
                t = 0.0
                while t < seconds and v.is_running():
                    twin.set_arm_radians({"left_shoulder_pan": 0.6 * math.sin(data.time),
                                          "right_shoulder_pan": -0.6 * math.sin(data.time)})
                    twin.run(action, seconds=1 / 60, realtime=True)
                    v.sync()
                    t += 1 / 60


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--viewer", action="store_true")
    args = parser.parse_args()
    check_mjcf()
    check_urdf()
    if args.viewer:
        run_viewer()
