"""Drive the MuJoCo twin with AlohaMini action keys, and map real-robot state onto twin joints.

The keys are exactly what the AlohaMini Host accepts on ZMQ port 5555 / returns on 5556
(src/lerobot/robots/alohamini/alohamini_host.py):

    x.vel, y.vel        body-frame m/s   (x forward, y left)
    theta.vel           deg/s            (counter-clockwise)
    lift_axis.height_mm absolute target, 0..600 mm
    arm_{left,right}_<joint>.pos  normalized motor units (see robot_state_to_twin_radians)

So the web UI can send one command format to either the twin or the real robot.
"""

import math
import time

import mujoco
import numpy as np

WATCHDOG_S = 1.0  # AlohaMiniHostConfig.watchdog_timeout_ms
LIFT_MAX_MM = 600.0  # LiftAxisConfig.soft_max_mm
ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")

# VERIFY on the lab robot: flip to -1 for any joint where the twin moves opposite to the real arm.
JOINT_SIGN = {j: 1 for j in ARM_JOINTS}


class TwinDriver:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.m, self.d = model, data
        self._base_qadr = [model.joint(n).qposadr[0] for n in ("base_x", "base_y", "base_yaw")]
        self._base_dadr = [model.joint(n).dofadr[0] for n in ("base_x", "base_y", "base_yaw")]
        self._base_act = [model.actuator(f"{n}_vel").id for n in ("base_x", "base_y", "base_yaw")]
        self._lift_act = model.actuator("lift_pos").id
        self._body_vel = np.zeros(3)  # x m/s, y m/s, theta rad/s
        self._last_cmd_t = 0.0
        self._hold_arm_pose()

    # ---- commands (same keys as the real robot) -------------------------------------------
    def send_action(self, action: dict[str, float]) -> None:
        # Like AlohaMini.send_action: a command without base keys means "base stopped".
        self._body_vel = np.array([action.get("x.vel", 0.0), action.get("y.vel", 0.0),
                                   math.radians(action.get("theta.vel", 0.0))])
        if "lift_axis.height_mm" in action:
            mm = min(max(float(action["lift_axis.height_mm"]), 0.0), LIFT_MAX_MM)
            self.d.ctrl[self._lift_act] = mm / 1000.0
        self._last_cmd_t = self.d.time

    def set_arm_radians(self, targets: dict[str, float]) -> None:
        """Targets keyed by twin joint name, e.g. {"left_elbow_flex": 0.5}."""
        for name, rad in targets.items():
            act = self.m.actuator(name)
            lo, hi = self.m.actuator_ctrlrange[act.id]
            self.d.ctrl[act.id] = min(max(rad, lo), hi)

    def run(self, action: dict[str, float], seconds: float, send: bool = True, realtime: bool = False) -> None:
        """Step for ``seconds``. With ``send``, re-send ``action`` every step like a streaming client;
        without it, no commands arrive and the watchdog stops the base after WATCHDOG_S."""
        for _ in range(round(seconds / self.m.opt.timestep)):
            if send:
                self.send_action(action)
            if self.d.time - self._last_cmd_t > WATCHDOG_S:
                self._body_vel[:] = 0.0  # mirror the Host watchdog
            yaw = self.d.qpos[self._base_qadr[2]]
            c, s = math.cos(yaw), math.sin(yaw)
            vx, vy, w = self._body_vel
            world = (c * vx - s * vy, s * vx + c * vy, w)
            for act, v in zip(self._base_act, world):
                self.d.ctrl[act] = v
            t0 = time.perf_counter()
            mujoco.mj_step(self.m, self.d)
            if self.d.warning[mujoco.mjtWarning.mjWARN_BADQACC].number:
                raise RuntimeError(f"twin simulation went unstable at t={self.d.time:.3f}s")
            if realtime:
                time.sleep(max(0.0, self.m.opt.timestep - (time.perf_counter() - t0)))

    # ---- state -----------------------------------------------------------------------------
    def base_pose(self) -> tuple[float, float, float]:
        return tuple(float(self.d.qpos[a]) for a in self._base_qadr)

    def base_velocity(self) -> np.ndarray:
        return np.array([self.d.qvel[a] for a in self._base_dadr])

    def lift_height_mm(self) -> float:
        return float(self.d.qpos[self.m.joint("lift").qposadr[0]] * 1000.0)

    def joint(self, name: str) -> float:
        return float(self.d.qpos[self.m.joint(name).qposadr[0]])

    def apply_robot_state(self, observation: dict, robot_metadata: dict) -> None:
        """Mirror a real Host observation (port 5556 JSON) onto the twin: arms + lift."""
        self.set_arm_radians(robot_state_to_twin_radians(observation, robot_metadata))
        if "lift_axis.height_mm" in observation:
            self.d.ctrl[self._lift_act] = float(observation["lift_axis.height_mm"]) / 1000.0

    def _hold_arm_pose(self) -> None:
        for side in ("left", "right"):
            for j in ARM_JOINTS:
                name = f"{side}_{j}"
                self.d.ctrl[self.m.actuator(name).id] = self.d.qpos[self.m.joint(name).qposadr[0]]


def robot_state_to_twin_radians(observation: dict, robot_metadata: dict) -> dict[str, float]:
    """Convert Host arm readings to twin joint radians.

    Inverts FeetechMotorsBus._normalize with the calibration the Host publishes in
    ``_robot_metadata.motors`` (range_min/max, drive_mode, normalization), then maps raw
    ticks to radians around the 2048 midpoint. The SO-101 ``new_calib`` model puts each
    joint's zero at the middle of its range, matching LeRobot's mid-pose calibration.
    """
    out = {}
    for motor, meta in robot_metadata.get("motors", {}).items():
        key = f"{motor}.pos"
        if key not in observation or not motor.startswith("arm_"):
            continue
        side, joint = motor[len("arm_"):].split("_", 1)
        if joint not in JOINT_SIGN:
            continue  # e.g. wrist_yaw on alohamini2 has no SO-101 counterpart
        val = float(observation[key])
        lo, hi, mode = meta["range_min"], meta["range_max"], meta["normalization"]
        if mode == "degrees":
            rad = math.radians(val)
        else:
            if mode == "range_m100_100":
                val = -val if meta["drive_mode"] else val
                raw = (val + 100) / 200 * (hi - lo) + lo
            else:  # range_0_100 (gripper)
                val = 100 - val if meta["drive_mode"] else val
                raw = val / 100 * (hi - lo) + lo
            rad = (raw - 2048) * 2 * math.pi / 4096
        out[f"{side}_{joint}"] = JOINT_SIGN[joint] * rad
    return out
