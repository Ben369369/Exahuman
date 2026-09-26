"""Read-only Feetech STS bus reader (pyserial only) for mirroring leader arms onto the twin.

Never writes a register and never touches torque, so it is safe to run on arms that another
program might also be using (as long as the COM port is free).

LeRobot's calibration stores its results in each servo's EEPROM (Homing_Offset, Min/Max
position limits), so the reader recovers the same normalisation as FeetechMotorsBus without
the calibration JSON: raw Present_Position is already homing-corrected, with the calibrated
middle pose near 2048 ticks.

    python hybrid_twin/tools/feetech_reader.py COM10 COM11      # probe + live print
"""

import sys
import time

import serial

BAUD = 1_000_000
ADDR_MIN_LIMIT, ADDR_MAX_LIMIT = 9, 11
ADDR_HOMING_OFFSET = 31
ADDR_PRESENT_POSITION = 56
INST_READ, INST_SYNC_READ = 0x02, 0x82
SO_ARM_IDS = {1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex", 4: "wrist_flex", 5: "wrist_roll", 6: "gripper"}


def _checksum(body: bytes) -> int:
    return (~sum(body)) & 0xFF


def _decode_sign(value: int, sign_bit: int) -> int:
    return -(value & ((1 << sign_bit) - 1)) if value & (1 << sign_bit) else value


class FeetechBus:
    def __init__(self, port: str, baudrate: int = BAUD, timeout: float = 0.02, ser=None):
        self.port = port
        self.ser = ser if ser is not None else serial.Serial(port, baudrate, timeout=timeout)
        self._rx = b""

    def close(self) -> None:
        self.ser.close()

    def _send(self, motor_id: int, inst: int, params: bytes) -> None:
        body = bytes([motor_id, len(params) + 2, inst]) + params
        self.ser.reset_input_buffer()
        self._rx = b""
        self.ser.write(b"\xff\xff" + body + bytes([_checksum(body)]))

    def _read_packet(self) -> tuple[int, bytes] | None:
        # Find FF FF header, then ID, LEN, ERR, data..., CHK. Leftover bytes stay in self._rx,
        # because a sync read's replies from several servos can arrive in one serial read.
        deadline = time.monotonic() + 0.05
        while True:
            buf = self._rx
            start = buf.find(b"\xff\xff")
            if start >= 0 and len(buf) >= start + 4:
                motor_id, length = buf[start + 2], buf[start + 3]
                end = start + 4 + length
                if len(buf) >= end:
                    self._rx = buf[end:]
                    body = buf[start + 2 : end - 1]
                    if _checksum(body) != buf[end - 1]:
                        return None
                    return motor_id, buf[start + 5 : end - 1]  # skip ERR byte
            if time.monotonic() >= deadline:
                return None
            self._rx += self.ser.read(max(1, self.ser.in_waiting))

    def read(self, motor_id: int, addr: int, n: int) -> bytes | None:
        self._send(motor_id, INST_READ, bytes([addr, n]))
        pkt = self._read_packet()
        return pkt[1] if pkt and pkt[0] == motor_id and len(pkt[1]) == n else None

    def read_u16(self, motor_id: int, addr: int) -> int | None:
        data = self.read(motor_id, addr, 2)
        return None if data is None else data[0] | (data[1] << 8)  # STS is little-endian

    def sync_read_positions(self, ids: list[int]) -> dict[int, int]:
        self._send(0xFE, INST_SYNC_READ, bytes([ADDR_PRESENT_POSITION, 2, *ids]))
        out = {}
        for _ in ids:
            pkt = self._read_packet()
            if pkt is None:
                break
            motor_id, data = pkt
            if len(data) == 2:
                out[motor_id] = _decode_sign(data[0] | (data[1] << 8), 15)
        return out

    def calibration(self, ids: list[int]) -> dict[int, dict]:
        cal = {}
        for i in ids:
            lo, hi = self.read_u16(i, ADDR_MIN_LIMIT), self.read_u16(i, ADDR_MAX_LIMIT)
            off = self.read_u16(i, ADDR_HOMING_OFFSET)
            if lo is not None and hi is not None:
                cal[i] = {"range_min": lo, "range_max": hi,
                          "homing_offset": None if off is None else _decode_sign(off, 11)}
        return cal


def probe(port: str) -> dict:
    bus = FeetechBus(port)
    try:
        cal = bus.calibration(list(SO_ARM_IDS))
        pos = bus.sync_read_positions(sorted(cal))
        return {"port": port, "ids": sorted(cal), "calibration": cal, "positions": pos}
    finally:
        bus.close()


if __name__ == "__main__":
    for p in sys.argv[1:] or ["COM10", "COM11"]:
        try:
            info = probe(p)
        except serial.SerialException as e:
            print(f"{p}: cannot open ({e})")
            continue
        print(f"{p}: motors {info['ids']}")
        for i in info["ids"]:
            c = info["calibration"][i]
            print(f"  id {i} {SO_ARM_IDS[i]:<14} pos={info['positions'].get(i)!s:>5}  "
                  f"range=[{c['range_min']}, {c['range_max']}]  homing_offset={c['homing_offset']}")
