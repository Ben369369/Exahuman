# Exahuman

Software for the Exahuman team at the Princeton Robotics Club: a low-latency teleoperated dual-arm robot, plus a digital twin so we can test code before it touches the hardware.

The robot is an [AlohaMini](https://github.com/liyiteng/alohamini) (mobile base, lift, two arms) running on a Raspberry Pi. Operators drive it with a pair of leader arms on a laptop: the follower arms on the robot copy them.

## What's in this repo

| Folder | What it is | Who wrote it |
|---|---|---|
| [`hybrid_twin/`](hybrid_twin/) | MuJoCo digital twin, browser control UI, leader-arm reader | Exahuman (this team) |
| `src/lerobot/`, `alohamini_sim/`, `docs/` | Robot drivers, teleop, dataset and training stack | Imported from [lerobot_alohamini](https://github.com/liyiteng/lerobot_alohamini), built on [HuggingFace LeRobot](https://github.com/huggingface/lerobot) |

Our work lives in `hybrid_twin/`. Everything else is the upstream codebase we build on, kept intact so the twin talks to the real robot through the same interface.

## The digital twin

`hybrid_twin/tools/build_twin.py` generates the whole robot (base, lift, two 6-joint SO-101 arms) as both a MuJoCo model and a URDF from one parameter set, so the physics sim and the web view never drift apart.

The twin accepts exactly the commands the real robot's Host takes, so code tested here runs on the robot unchanged.

**Why MuJoCo for the live twin, not Isaac Sim:** the twin needs only `mujoco` and `numpy`, so it runs on any team laptop. Isaac Sim needs a GPU toolchain most of us don't have, so we keep it for photoreal scene generation and use MuJoCo for day-to-day testing.

## Browser control

```
browser (three.js twin, WASD, lift, STOP)
   │  WebSocket, 30 Hz input
server.py (aiohttp bridge, 50 Hz loop, all safety limits)
   ├── sim backend   → MuJoCo twin
   └── robot backend → Host on the Raspberry Pi over ZeroMQ
```

Safety lives in the server, not the browser:
- **Deadman:** no input for 0.3 s, a lost connection or a hidden tab means zero velocity.
- **One operator:** the first tab gets control; others are view-only.
- **Limits:** speeds are clamped, and the lift target ramps and stays near the measured height.

## Leader-arm mirroring

`hybrid_twin/tools/feetech_reader.py` reads both leader arms' servos at 50 Hz and the twin's arms follow them. It's read-only by design: it never writes registers or changes torque, and `test_leader_arms.py` checks that against a simulated servo bus.

## Run it

```bash
pip install -r hybrid_twin/web/requirements.txt
python hybrid_twin/tools/build_twin.py
python hybrid_twin/tools/check_twin.py           # headless checks
python hybrid_twin/web/server.py                 # twin at http://localhost:8080
python hybrid_twin/web/server.py --leader-left mock --leader-right mock   # no hardware needed
```

More detail: [`hybrid_twin/README.md`](hybrid_twin/README.md) and [`hybrid_twin/web/README.md`](hybrid_twin/web/README.md).

## Open work

- Measure the real base and lift and replace the placeholder geometry in `build_twin.py`.
- Merge web driving and arm teleop into one command sender, since the Host accepts one sender at a time.

## Credits

AlohaMini and lerobot_alohamini are by Li Yiteng and Wu Zhiyong. The SO-101 arm models come from [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) (Apache-2.0). Both build on HuggingFace LeRobot.
