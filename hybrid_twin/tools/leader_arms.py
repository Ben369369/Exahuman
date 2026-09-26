"""Background reader that turns the leader arms on this laptop into twin joint angles.

    arms = LeaderArms({"left": "COM10", "right": "COM11"})   # or "mock" for a synthetic wave
    arms.start()
    arms.radians()   # {"left_shoulder_pan": 0.12, ..., "right_gripper": 0.9}
    arms.status()    # per-side connection / error info for the UI

Read-only: never writes registers or changes torque (see feetech_reader.py).
"""

import math
import threading
import time

from feetech_reader import SO_ARM_IDS, FeetechBus

TICKS_PER_REV = 4096
MID_TICKS = 2048  # LeRobot calibration homes the middle pose here
# Twin gripper range (so101_new_calib.xml gripper ctrlrange): closed .. fully open.
TWIN_GRIPPER_RAD = (-0.17453, 1.74533)
# VERIFY on the lab arms: flip to -1 where the twin moves opposite to the leader.
LEADER_SIGN = {j: 1 for j in SO_ARM_IDS.values()}


def raw_to_twin(joint: str, raw: int, cal: dict | None) -> float:
    if joint == "gripper" and cal and cal["range_max"] > cal["range_min"]:
        # Map the gripper through its calibrated travel, as LeRobot's 0-100 scale does.
        frac = min(max((raw - cal["range_min"]) / (cal["range_max"] - cal["range_min"]), 0.0), 1.0)
        lo, hi = TWIN_GRIPPER_RAD
        return lo + frac * (hi - lo)
    return LEADER_SIGN[joint] * (raw - MID_TICKS) * 2 * math.pi / TICKS_PER_REV


class LeaderArms:
    def __init__(self, ports: dict[str, str], hz: float = 50.0, bus_factory=FeetechBus):
        self.ports = {side: p for side, p in ports.items() if p}
        self.period = 1.0 / hz
        self.bus_factory = bus_factory
        self._lock = threading.Lock()
        self._angles: dict[str, float] = {}
        self._status = {side: {"port": p, "ok": False, "error": "starting", "age_s": None}
                        for side, p in self.ports.items()}
        self._stop = threading.Event()

    def start(self) -> None:
        for side, port in self.ports.items():
            target = self._run_mock if port == "mock" else self._run_bus
            threading.Thread(target=target, args=(side, port), daemon=True, name=f"leader-{side}").start()

    def stop(self) -> None:
        self._stop.set()

    def radians(self) -> dict[str, float]:
        with self._lock:
            return dict(self._angles)

    def status(self) -> dict:
        now = time.monotonic()
        with self._lock:
            return {side: {**s, "age_s": None if s["age_s"] is None else round(now - s["age_s"], 3),
                           "ok": s["ok"] and s["age_s"] is not None and now - s["age_s"] < 0.5}
                    for side, s in self._status.items()}

    def _publish(self, side: str, angles: dict[str, float], ok: bool, error: str | None) -> None:
        with self._lock:
            self._angles.update({f"{side}_{j}": v for j, v in angles.items()})
            st = self._status[side]
            st["ok"], st["error"] = ok, error
            if ok:
                st["age_s"] = time.monotonic()

    def _run_bus(self, side: str, port: str) -> None:
        while not self._stop.is_set():
            bus = None
            try:
                bus = self.bus_factory(port)
                cal = bus.calibration(list(SO_ARM_IDS))
                if not cal:
                    raise RuntimeError("no servos answered (arm powered? board jumper on USB?)")
                ids = sorted(cal)
                misses = 0
                while not self._stop.is_set():
                    t0 = time.monotonic()
                    raw = bus.sync_read_positions(ids)
                    if len(raw) == len(ids):
                        misses = 0
                        self._publish(side, {SO_ARM_IDS[i]: raw_to_twin(SO_ARM_IDS[i], v, cal.get(i))
                                             for i, v in raw.items()}, True, None)
                    else:
                        misses += 1
                        if misses > 25:
                            raise RuntimeError(f"lost servos: got {sorted(raw)} of {ids}")
                    time.sleep(max(0.0, self.period - (time.monotonic() - t0)))
            except Exception as e:  # keep retrying: arms get unplugged / powered later
                self._publish(side, {}, False, str(e))
                time.sleep(1.0)
            finally:
                if bus is not None:
                    bus.close()

    def _run_mock(self, side: str, _port: str) -> None:
        t0, sign = time.monotonic(), 1 if side == "left" else -1
        while not self._stop.is_set():
            t = time.monotonic() - t0
            self._publish(side, {
                "shoulder_pan": sign * 0.5 * math.sin(0.6 * t),
                "shoulder_lift": -0.4 + 0.3 * math.sin(0.9 * t),
                "elbow_flex": 0.5 + 0.3 * math.sin(0.9 * t + 1),
                "wrist_flex": 0.3 * math.sin(1.2 * t),
                "wrist_roll": 0.8 * math.sin(0.5 * t),
                "gripper": 0.8 + 0.8 * math.sin(1.5 * t),
            }, True, None)
            time.sleep(self.period)
