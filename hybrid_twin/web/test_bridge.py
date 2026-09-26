#!/usr/bin/env python
"""End-to-end check of a running bridge, driving it over the same WebSocket the browser uses.

    python hybrid_twin/web/server.py &            # or with --robot-ip pointing at mock_host.py
    python hybrid_twin/web/test_bridge.py [--url http://127.0.0.1:8080] [--tolerance 0.05]

Do NOT run against the real robot unless it is on blocks or has clear floor space: it drives
0.4 m forward, turns 60 deg and lifts to 300 mm (~11 s at the real ~27 mm/s).
"""

import argparse
import asyncio
import json
import math
import time

import aiohttp

IDLE = {"type": "input", "fwd": 0, "left": 0, "ccw": 0, "speed": 0.2, "turn": 60,
        "lift_mm": None, "active": False, "estop": False}


async def run(url: str, tol: float) -> None:
    async with aiohttp.ClientSession() as s:
        for path in ("/", "/static/app.js", "/config.json", "/twin/urdf/alohamini_twin.urdf",
                     "/twin/arms/so101/assets/base_so101_v2.stl"):
            async with s.get(url + path) as r:
                assert r.status == 200, f"{path}: HTTP {r.status}"
        print("static files ok")

        async with s.ws_connect(url + "/ws") as ws:
            st: dict = {}

            async def recv():
                async for m in ws:
                    st.update(json.loads(m.data))

            reader = asyncio.create_task(recv())

            async def drive(seconds: float, **kw) -> None:
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    await ws.send_str(json.dumps(IDLE | kw))
                    await asyncio.sleep(1 / 30)

            for _ in range(50):  # first state can take a moment while robot feedback warms up
                if st.get("online"):
                    break
                await asyncio.sleep(0.1)
            assert st.get("you_control"), "first client should get control"
            assert st["online"], "backend has no fresh feedback"
            mode = st["mode"]

            x0, y0, yaw0 = st["pose"]
            await drive(2.0, fwd=1, active=True)
            await drive(0.6)  # settle
            dx = st["pose"][0] - x0
            print(f"[{mode}] forward 2 s @ 0.2 m/s: {dx:.3f} m, rate {st['cmd_rate_hz']} Hz")
            assert abs(dx - 0.4) < 0.4 * tol + 0.02, f"forward distance {dx:.3f} != 0.4"

            await drive(1.0, ccw=1, active=True)
            await drive(0.6)
            dyaw = math.degrees(st["pose"][2] - yaw0)
            print(f"[{mode}] ccw 1 s @ 60 deg/s: {dyaw:.1f} deg")
            assert abs(dyaw - 60) < 60 * tol + 2, f"rotation {dyaw:.1f} != 60"

            t0 = time.monotonic()
            while time.monotonic() - t0 < 25 and abs(st["lift_mm"] - 300) > 4:
                await drive(0.1, lift_mm=300, active=True)
            print(f"[{mode}] lift to 300 mm: {st['lift_mm']:.0f} mm in {time.monotonic() - t0:.1f} s")
            assert abs(st["lift_mm"] - 300) <= 5

            await drive(1.5)
            held = st["lift_mm"]
            print(f"[{mode}] lift after 1.5 s idle: {held:.0f} mm, commanding={st['commanding']}")
            assert abs(held - 300) <= 5, "lift drifted while holding"
            assert not st["commanding"], "bridge kept commanding while idle"

            x1 = st["pose"][0]
            await drive(1.0, fwd=1, active=True, estop=True)
            print(f"[{mode}] estop + W: moved {abs(st['pose'][0] - x1):.3f} m, commanding={st['commanding']}")
            assert not st["commanding"] and abs(st["pose"][0] - x1) < 0.005

            await ws.send_str(json.dumps({"type": "release_control"}))
            await asyncio.sleep(0.3)
            assert not st["you_control"] and not st["has_controller"]
            await ws.send_str(json.dumps({"type": "take_control"}))
            await asyncio.sleep(0.3)
            assert st["you_control"]
            reader.cancel()
        print("all bridge checks passed")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--url", default="http://127.0.0.1:8080")
    p.add_argument("--tolerance", type=float, default=0.05, help="relative tolerance for distance/angle")
    a = p.parse_args()
    asyncio.run(run(a.url, a.tolerance))
