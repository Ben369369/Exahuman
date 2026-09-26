#!/usr/bin/env python
"""Offline tests for the leader-arm reader: a fake STS servo bus answers real protocol packets.

    python hybrid_twin/tools/test_leader_arms.py
"""

import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from feetech_reader import FeetechBus, _checksum  # noqa: E402
from leader_arms import TWIN_GRIPPER_RAD, LeaderArms  # noqa: E402


class FakeServoSerial:
    """Emulates STS3215s on one half-duplex bus: answers READ (0x02) and SYNC_READ (0x82)."""

    def __init__(self, servos: dict[int, dict[int, int]]):
        self.servos = servos  # id -> {addr: u16 value}
        self.out = b""

    def _reply(self, motor_id: int, data: bytes) -> bytes:
        body = bytes([motor_id, len(data) + 2, 0]) + data
        return b"\xff\xff" + body + bytes([_checksum(body)])

    def _u16(self, motor_id: int, addr: int) -> bytes:
        v = self.servos[motor_id].get(addr, 0)
        return bytes([v & 0xFF, v >> 8])

    def write(self, pkt: bytes) -> None:
        assert pkt[:2] == b"\xff\xff" and _checksum(pkt[2:-1]) == pkt[-1], "bad packet from reader"
        motor_id, _len, inst, *params = pkt[2:-1]
        if inst == 0x02 and motor_id in self.servos:
            self.out += self._reply(motor_id, self._u16(motor_id, params[0]))
        elif inst == 0x82:
            addr, _n, *ids = params
            for i in ids:
                if i in self.servos:
                    self.out += self._reply(i, self._u16(i, addr))
        assert inst in (0x02, 0x82), f"reader must never write (inst {inst:#x})"

    @property
    def in_waiting(self) -> int:
        return len(self.out)

    def read(self, n: int) -> bytes:
        chunk, self.out = self.out[:n], self.out[n:]
        return chunk

    def reset_input_buffer(self) -> None:
        self.out = b""

    def close(self) -> None:
        pass


def make_arm(positions: dict[int, int]) -> FakeServoSerial:
    # EEPROM as LeRobot calibration leaves it: min/max limits + homing offset; RAM position.
    servos = {i: {9: 1000, 11: 3100, 31: (1 << 11) | 37, 56: p} for i, p in positions.items()}
    servos[6][9], servos[6][11] = 2000, 3000  # gripper travel
    return FakeServoSerial(servos)


def main() -> None:
    fake = make_arm({1: 2048, 2: 2048 + 1024, 3: 2048 - 512, 4: 2048, 5: 2048, 6: 2500})
    bus = FeetechBus("FAKE", ser=fake)
    cal = bus.calibration(list(range(1, 7)))
    assert sorted(cal) == [1, 2, 3, 4, 5, 6] and cal[1]["homing_offset"] == -37, cal[1]
    pos = bus.sync_read_positions([1, 2, 3, 4, 5, 6])
    assert pos == {1: 2048, 2: 3072, 3: 1536, 4: 2048, 5: 2048, 6: 2500}, pos
    fake.servos[2][56] = (1 << 15) | 5  # sign-magnitude negative
    assert bus.sync_read_positions([2])[2] == -5
    print("protocol ok: read, sync read, sign decoding, read-only")

    fake.servos[2][56] = 2048 + 1024
    arms = LeaderArms({"left": "FAKE"}, bus_factory=lambda port: FeetechBus(port, ser=fake))
    arms.start()
    for _ in range(50):
        if arms.status()["left"]["ok"]:
            break
        time.sleep(0.02)
    a = arms.radians()
    assert abs(a["left_shoulder_pan"]) < 1e-9
    assert abs(a["left_shoulder_lift"] - math.pi / 2) < 1e-9  # +1024 ticks = +90 deg
    assert abs(a["left_elbow_flex"] + math.pi / 4) < 1e-9
    assert abs(a["left_gripper"] - sum(TWIN_GRIPPER_RAD) / 2) < 1e-9  # halfway through travel
    print("leader -> twin angles ok:", {k: round(v, 3) for k, v in a.items()})

    dead = LeaderArms({"right": "FAKE"}, bus_factory=lambda port: FeetechBus(port, ser=FakeServoSerial({})))
    dead.start()
    for _ in range(60):  # probing 6 silent servos takes ~1 s of read timeouts
        st = dead.status()["right"]
        if st["error"] != "starting":
            break
        time.sleep(0.05)
    assert not st["ok"] and "no servos answered" in st["error"], st
    print("unpowered arm reported:", st["error"])
    arms.stop()
    dead.stop()
    print("all leader tests passed")


if __name__ == "__main__":
    main()
