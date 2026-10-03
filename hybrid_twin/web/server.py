#!/usr/bin/env python
"""Web control bridge for the AlohaMini digital twin.

Serves the browser UI and relays base + lift commands to either the MuJoCo twin or the
real robot's Host (src/lerobot/robots/alohamini/alohamini_host.py). Arms stay on leader
teleop; the bridge never sends arm targets.

    python hybrid_twin/web/server.py                          # sim: MuJoCo twin
    python hybrid_twin/web/server.py --robot-ip 192.168.1.50  # real robot Host on 5555/5556

Then open http://localhost:8080.

Safety rules (mirrors AlohaMiniClient):
  * Only one browser tab controls; others are view-only.
  * Commands are sent only while the operator is active; 0.5 s after the last input the
    bridge sends a stop and then goes quiet, so the Host watchdog releases control.
  * Robot mode sends only while Host feedback is fresher than 250 ms.
  * Base speed is clamped to the fast keyboard level; the lift target is ramped at
    150 mm/s and never leads measured height by more than 50 mm.
"""

import argparse
import asyncio
import json
import logging
import math
import os
import sys
import time
from pathlib import Path
from uuid import uuid4

# aiohttp's sendfile path on Windows can send 64 KB blocks out of order when the client reads
# slowly (a browser busy parsing meshes), which corrupts the STLs into NaN/scrambled geometry.
# Must be set before aiohttp is imported.
os.environ.setdefault("AIOHTTP_NOSENDFILE", "1")

from aiohttp import WSMsgType, web  # noqa: E402

TWIN_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TWIN_ROOT / "tools"))

TICK_HZ = 50
MAX_XY = 0.25  # m/s, AlohaMiniClient fast level
MAX_THETA = 75.0  # deg/s
LIFT_MIN_MM, LIFT_MAX_MM = 0.0, 600.0
LIFT_SPEED_MM_S = 150.0  # AlohaMiniClientConfig.lift_target_speed_mm_s
LIFT_MAX_LEAD_MM = 50.0  # AlohaMiniClientConfig.lift_target_max_lead_mm
INPUT_TIMEOUT_S = 0.3  # browser deadman: no message -> zero velocity
ACTIVE_GRACE_S = 0.5  # keep commanding this long after the last non-idle input
FEEDBACK_FRESH_S = 0.25

# Wheel drive directions (deg) and positions, from AlohaMini._body_to_wheel_raw.
WHEEL_DRIVE_DEG = {"left": 150.0, "back": -90.0, "right": 30.0}
ROBOT_SPECS = {
    "alohamini1": {"wheel_radius": 0.05, "base_radius": 0.125, "lead_mm_per_rev": 84.0},
    "alohamini2": {"wheel_radius": 0.063, "base_radius": 0.195, "lead_mm_per_rev": 131.0},
    "alohamini2pro": {"wheel_radius": 0.063, "base_radius": 0.195, "lead_mm_per_rev": 131.0},
}


def clamp(v: float, lo: float, hi: float) -> float:
    return min(max(v, lo), hi)


class Odometry:
    """Integrates body-frame velocities into a planar pose plus visual wheel spin."""

    def __init__(self, wheel_radius: float, base_radius: float):
        self.x = self.y = self.yaw = 0.0
        self.wheels = dict.fromkeys(WHEEL_DRIVE_DEG, 0.0)
        self.wr, self.br = wheel_radius, base_radius

    def step(self, vx: float, vy: float, theta_deg_s: float, dt: float) -> None:
        w = math.radians(theta_deg_s)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (c * vx - s * vy) * dt
        self.y += (s * vx + c * vy) * dt
        self.yaw += w * dt
        self.spin_wheels(vx, vy, w, dt)

    def spin_wheels(self, vx: float, vy: float, w: float, dt: float) -> None:
        for name, deg in WHEEL_DRIVE_DEG.items():
            a = math.radians(deg)
            self.wheels[name] += (math.cos(a) * vx + math.sin(a) * vy + self.br * w) / self.wr * dt


class Operator:
    """Latest browser input with deadman handling."""

    def __init__(self):
        self.vx = self.vy = self.theta = 0.0
        self.lift_goal_mm: float | None = None
        self.estop = False
        self.last_msg_t = 0.0
        self.last_active_t = 0.0

    def update(self, msg: dict) -> None:
        now = time.monotonic()
        self.estop = bool(msg.get("estop", False))
        speed = clamp(float(msg.get("speed", 0.2)), 0.0, MAX_XY)
        turn = clamp(float(msg.get("turn", 60.0)), 0.0, MAX_THETA)
        # Inputs are unit directions in [-1, 1]; the server owns the speed limits.
        self.vx = clamp(float(msg.get("fwd", 0)), -1, 1) * speed
        self.vy = clamp(float(msg.get("left", 0)), -1, 1) * speed
        self.theta = clamp(float(msg.get("ccw", 0)), -1, 1) * turn
        if msg.get("lift_mm") is not None:
            goal = float(msg["lift_mm"])
            if math.isfinite(goal):
                self.lift_goal_mm = clamp(goal, LIFT_MIN_MM, LIFT_MAX_MM)
        if msg.get("active"):
            self.last_active_t = now
        self.last_msg_t = now

    def base_command(self) -> tuple[float, float, float]:
        if self.estop or time.monotonic() - self.last_msg_t > INPUT_TIMEOUT_S:
            return 0.0, 0.0, 0.0
        return self.vx, self.vy, self.theta

    def wants_control(self) -> bool:
        return not self.estop and time.monotonic() - self.last_active_t < ACTIVE_GRACE_S


class LiftRamp:
    """Moves the commanded lift target toward the operator goal like AlohaMiniClient does."""

    def __init__(self):
        self.target_mm: float | None = None

    def step(self, goal_mm: float | None, measured_mm: float, dt: float, hold: bool) -> float:
        if hold or goal_mm is None or self.target_mm is None:
            self.target_mm = measured_mm
        if not hold and goal_mm is not None:
            step = LIFT_SPEED_MM_S * dt
            self.target_mm += clamp(goal_mm - self.target_mm, -step, step)
            self.target_mm = clamp(self.target_mm, measured_mm - LIFT_MAX_LEAD_MM, measured_mm + LIFT_MAX_LEAD_MM)
        self.target_mm = clamp(self.target_mm, LIFT_MIN_MM, LIFT_MAX_MM)
        return self.target_mm


class SimBackend:
    name = "sim"

    def __init__(self, robot_model: str, leader=None):
        import mujoco
        from twin_driver import TwinDriver

        self.mujoco = mujoco
        model = mujoco.MjModel.from_xml_path(str(TWIN_ROOT / "mujoco" / "alohamini_twin.xml"))
        self.data = mujoco.MjData(model)
        self.model = model
        spec = ROBOT_SPECS[robot_model]
        self.twin = TwinDriver(model, self.data, lift_lead_mm_per_rev=spec["lead_mm_per_rev"])
        self.wheels = Odometry(spec["wheel_radius"], spec["base_radius"])
        self._joint_names = [model.joint(i).name for i in range(model.njnt)]
        self.last_action: dict = {}
        self.leader = leader  # LeaderArms or None: the twin's arms follow the laptop's leader arms

    async def start(self) -> None:
        pass

    def measured_lift_mm(self) -> float:
        return self.twin.lift_height_mm()

    def feedback_fresh(self) -> bool:
        return True

    def tick(self, action: dict | None, dt: float) -> None:
        # None = bridge is quiet; the twin's own 1 s watchdog then stops it, like the Host.
        if self.leader is not None:
            self.twin.set_arm_radians(self.leader.radians())
        self.twin.run(action or {}, seconds=dt, send=action is not None)
        self.last_action = action or {}
        vx, vy, w = self.twin.base_velocity()
        yaw = self.twin.base_pose()[2]
        c, s = math.cos(yaw), math.sin(yaw)
        self.wheels.spin_wheels(c * vx + s * vy, -s * vx + c * vy, w, dt)

    def state(self) -> dict:
        x, y, yaw = self.twin.base_pose()
        joints = {n: self.twin.joint(n) for n in self._joint_names if not n.startswith("wheel_")}
        joints |= {f"wheel_{k}_spin": v for k, v in self.wheels.wheels.items()}
        vx, vy, w = self.twin.base_velocity()
        c, s = math.cos(yaw), math.sin(yaw)
        return {
            "link": "sim",
            "online": True,
            "pose": [x, y, yaw],
            "lift_mm": self.twin.lift_height_mm(),
            "vel": [c * vx + s * vy, -s * vx + c * vy, math.degrees(w)],
            "joints": joints,
            "safety": {"target_source": "command" if self.last_action else "none"},
            "arms": {"source": "leader" if self.leader else "none",
                     "leader": self.leader.status() if self.leader else {}},
        }


class RobotBackend:
    """Minimal AlohaMini Host client: PUSH commands to 5555, request ':state' from 5556."""

    name = "robot"

    def __init__(self, ip: str, cmd_port: int, obs_port: int, robot_model: str):
        import zmq
        import zmq.asyncio

        self.zmq = zmq
        self.ctx = zmq.asyncio.Context()
        self.cmd = self.ctx.socket(zmq.PUSH)
        self.cmd.setsockopt(zmq.CONFLATE, 1)
        self.cmd.setsockopt(zmq.LINGER, 0)
        self.cmd.connect(f"tcp://{ip}:{cmd_port}")
        self.obs = self.ctx.socket(zmq.DEALER)
        self.obs.setsockopt(zmq.LINGER, 0)
        self.obs.connect(f"tcp://{ip}:{obs_port}")
        self.address = f"{ip}:{cmd_port}/{obs_port}"
        self.client_id = f"web-{uuid4().hex[:12]}"
        self.sequence = 0
        self.request_id = 0
        self.obs_json: dict = {}
        self.metadata: dict = {}
        self.safety: dict = {}
        self.feedback_t: float | None = None
        spec = ROBOT_SPECS[robot_model]
        self.odom = Odometry(spec["wheel_radius"], spec["base_radius"])
        self._odom_t: float | None = None

    async def start(self) -> None:
        asyncio.create_task(self._poll_observations())

    async def _poll_observations(self) -> None:
        # One outstanding state-only request at a time; the Host answers once per loop.
        while True:
            self.request_id += 1
            token = f"{self.client_id}-{self.request_id}:state".encode()
            try:
                await self.obs.send(token)
                deadline = time.monotonic() + 0.5
                while time.monotonic() < deadline:
                    if await self.obs.poll(timeout=50):
                        parts = await self.obs.recv_multipart()
                        if parts and parts[0] == token and len(parts) >= 2:
                            self._on_observation(json.loads(parts[1]))
                            break
            except Exception:
                logging.exception("observation poll failed")
            await asyncio.sleep(1 / TICK_HZ)

    def _on_observation(self, obs: dict) -> None:
        now = time.monotonic()
        self.metadata = obs.pop("_robot_metadata", self.metadata)
        self.safety = obs.pop("_safety", {})
        self.obs_json = obs
        if self._odom_t is not None:
            dt = min(now - self._odom_t, 0.2)
            self.odom.step(float(obs.get("x.vel", 0)), float(obs.get("y.vel", 0)), float(obs.get("theta.vel", 0)), dt)
        self._odom_t = now
        self.feedback_t = now

    def feedback_fresh(self) -> bool:
        timeout = min(FEEDBACK_FRESH_S, float(self.safety.get("command_watchdog_timeout_s", FEEDBACK_FRESH_S)))
        return self.feedback_t is not None and time.monotonic() - self.feedback_t < timeout

    def measured_lift_mm(self) -> float:
        return float(self.obs_json.get("lift_axis.height_mm", 0.0))

    def tick(self, action: dict | None, dt: float) -> None:
        if action is None or not self.feedback_fresh():
            return
        payload = dict(action)
        if self.safety.get("version") == 1:
            self.sequence += 1
            command = {"client_id": self.client_id, "sequence": self.sequence}
            if "control_owner" in self.safety:
                command["host_session_id"] = self.safety["host_session_id"]
            if "control_epoch" in self.safety:
                command["control_epoch"] = self.safety["control_epoch"]
            payload["_command"] = command
        try:
            self.cmd.send_string(json.dumps(payload), flags=self.zmq.NOBLOCK)
        except self.zmq.Again:
            pass

    def state(self) -> dict:
        from twin_driver import robot_state_to_twin_radians

        joints = robot_state_to_twin_radians(self.obs_json, self.metadata) if self.metadata else {}
        joints |= {f"wheel_{k}_spin": v for k, v in self.odom.wheels.items()}
        lift = self.measured_lift_mm()
        joints["lift"] = lift / 1000.0
        owner = self.safety.get("control_owner")
        return {
            "link": self.address,
            "online": self.feedback_fresh(),
            "pose": [self.odom.x, self.odom.y, self.odom.yaw],
            "lift_mm": lift,
            "vel": [float(self.obs_json.get(k, 0.0)) for k in ("x.vel", "y.vel", "theta.vel")],
            "joints": joints,
            "arms": {"source": "robot", "leader": {}},
            "safety": {
                "target_source": self.safety.get("target_source"),
                "watchdog_active": self.safety.get("watchdog_active"),
                "owner": "this bridge" if owner == self.client_id else owner,
            },
        }


class Bridge:
    def __init__(self, backend):
        self.backend = backend
        self.operator = Operator()
        self.lift = LiftRamp()
        self.clients: dict[web.WebSocketResponse, str] = {}
        self.controller: web.WebSocketResponse | None = None
        self.commanding = False
        self.stop_until = 0.0
        self.sent = 0
        self.rate_hz = 0.0

    async def run(self) -> None:
        await self.backend.start()
        period = 1 / TICK_HZ
        rate_t, rate_n = time.monotonic(), 0
        broadcast_every, n = 2, 0  # 25 Hz UI updates
        last_t = time.monotonic()
        while True:
            t0 = time.monotonic()
            # Advance by measured time: Windows sleep granularity (~15 ms) makes ticks irregular,
            # and the twin must stay real-time for the odometry and lift ramp to match the robot.
            dt, last_t = min(t0 - last_t, 0.1), t0
            action = self._next_action(dt)
            self.backend.tick(action, dt)
            if action is not None:
                rate_n += 1
            if t0 - rate_t >= 1.0:
                self.rate_hz, rate_t, rate_n = rate_n / (t0 - rate_t), t0, 0
            n += 1
            if n % broadcast_every == 0 and self.clients:
                await self._broadcast(action)
            await asyncio.sleep(max(0.0, period - (time.monotonic() - t0)))

    def _next_action(self, dt: float) -> dict | None:
        op, now = self.operator, time.monotonic()
        measured = self.backend.measured_lift_mm()
        active = self.controller is not None and op.wants_control() and self.backend.feedback_fresh()
        if active:
            vx, vy, th = op.base_command()
            target = self.lift.step(op.lift_goal_mm, measured, dt, hold=False)
            self.commanding, self.stop_until = True, now + 0.3
            return {"x.vel": vx, "y.vel": vy, "theta.vel": th, "lift_axis.height_mm": target}
        if self.commanding or now < self.stop_until:
            # Explicit stop for a moment, then go quiet so the Host watchdog releases control.
            self.commanding = False
            self.lift.step(None, measured, dt, hold=True)
            op.lift_goal_mm = None
            return {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0, "lift_axis.height_mm": measured}
        self.lift.step(None, measured, dt, hold=True)
        return None

    async def _broadcast(self, action: dict | None) -> None:
        state = self.backend.state()
        state |= {
            "type": "state",
            "mode": self.backend.name,
            "commanding": action is not None,
            "cmd": action or {},
            "cmd_rate_hz": round(self.rate_hz, 1),
            "estop": self.operator.estop,
            "viewers": len(self.clients),
            "has_controller": self.controller is not None,
        }
        for ws in list(self.clients):
            try:
                await ws.send_str(json.dumps(state | {"you_control": ws is self.controller}))
            except ConnectionError:
                self.clients.pop(ws, None)

    async def websocket(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=5)
        await ws.prepare(request)
        self.clients[ws] = request.remote or "?"
        if self.controller is None:
            self.controller = ws
        try:
            async for msg in ws:
                if msg.type != WSMsgType.TEXT:
                    continue
                data = json.loads(msg.data)
                if data.get("type") == "take_control" and self.controller is None:
                    self.controller = ws
                elif data.get("type") == "release_control" and ws is self.controller:
                    self._release()
                elif data.get("type") == "input" and ws is self.controller:
                    self.operator.update(data)
        finally:
            self.clients.pop(ws, None)
            if ws is self.controller:
                self._release()
        return ws

    def _release(self) -> None:
        self.controller = None
        self.operator = Operator()


def build_app(bridge: Bridge) -> web.Application:
    app = web.Application()
    static = Path(__file__).resolve().parent / "static"

    async def index(_):
        return web.FileResponse(static / "index.html")

    async def config(_):
        return web.json_response({"mode": bridge.backend.name, "lift_max_mm": LIFT_MAX_MM,
                                  "speeds": [{"xy": 0.15, "theta": 45}, {"xy": 0.2, "theta": 60},
                                             {"xy": 0.25, "theta": 75}]})

    async def on_startup(app):
        app["loop_task"] = asyncio.create_task(bridge.run())

    app.router.add_get("/", index)
    app.router.add_get("/ws", bridge.websocket)
    app.router.add_get("/config.json", config)
    app.router.add_static("/static/", static)
    app.router.add_static("/twin/urdf/", TWIN_ROOT / "urdf")
    app.router.add_static("/twin/arms/", TWIN_ROOT / "arms")
    app.on_startup.append(on_startup)
    return app


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot-ip", help="AlohaMini Host IP; omit to drive the MuJoCo twin")
    parser.add_argument("--cmd-port", type=int, default=5555)
    parser.add_argument("--obs-port", type=int, default=5556)
    parser.add_argument("--robot-model", default="alohamini1", choices=sorted(ROBOT_SPECS))
    parser.add_argument("--leader-left", help="leader arm serial port (e.g. COM10) or 'mock'; sim mode only")
    parser.add_argument("--leader-right", help="leader arm serial port (e.g. COM11) or 'mock'; sim mode only")
    parser.add_argument("--host", default="127.0.0.1", help="use 0.0.0.0 to allow other lab machines")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

    if sys.platform == "win32":
        # zmq.asyncio needs add_reader(), which Windows' default Proactor loop lacks.
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    if args.robot_ip and (args.leader_left or args.leader_right):
        # In robot mode the teleop script owns the leader ports and drives the real follower;
        # the twin already shows the follower's measured pose.
        parser.error("--leader-* is for sim mode; with --robot-ip the twin mirrors the real arms")
    if args.robot_ip:
        backend = RobotBackend(args.robot_ip, args.cmd_port, args.obs_port, args.robot_model)
    else:
        leader = None
        if args.leader_left or args.leader_right:
            from leader_arms import LeaderArms

            leader = LeaderArms({"left": args.leader_left, "right": args.leader_right})
            leader.start()
            logging.info("twin arms follow leader arms: %s", leader.ports)
        backend = SimBackend(args.robot_model, leader)
    logging.info("backend=%s  open http://%s:%d", backend.name, args.host, args.port)
    web.run_app(build_app(Bridge(backend)), host=args.host, port=args.port, print=None)


if __name__ == "__main__":
    main()
