# AlohaMini web control

A browser UI for driving the AlohaMini base and lift, shown on the digital twin. The arms stay
on leader-arm teleop; this UI never commands them, it only displays them.

```
browser (three.js twin, WASD, lift lever, STOP)
   │  WebSocket /ws, 30 Hz input
server.py (aiohttp bridge, 50 Hz loop, all safety limits live here)
   ├── sim backend   → MuJoCo twin (../mujoco/alohamini_twin.xml)
   └── robot backend → AlohaMini Host on the Pi: ZMQ 5555 commands, 5556 state
```

## Run

```bash
pip install -r hybrid_twin/web/requirements.txt

python hybrid_twin/web/server.py                            # MuJoCo twin
python hybrid_twin/web/server.py --robot-ip <pi-ip>         # real robot
python hybrid_twin/web/server.py --robot-ip <pi-ip> --host 0.0.0.0   # reachable from other lab PCs
```

Open http://localhost:8080. The page loads three.js from jsDelivr, so the browser needs
internet access.

### Mirror your leader arms on the twin (sim mode)

```bash
python hybrid_twin/tools/feetech_reader.py COM10 COM11      # probe: should list motors 1-6 per arm
python hybrid_twin/web/server.py --leader-left COM10 --leader-right COM11
python hybrid_twin/web/server.py --leader-left mock --leader-right mock   # no hardware: synthetic motion
```

The twin's arms follow the leader arms plugged into this laptop. Leader positions are read
**read-only** at 50 Hz (`tools/feetech_reader.py`): no register writes and no torque changes.
Calibration comes from what LeRobot stores in each servo's EEPROM, so no calibration JSON is
needed. The Arms panel shows each leader's status and retries every second, so arms can be
plugged in or powered up while the server runs.

If the probe lists no motors:
- Check the servo power supply; USB alone doesn't power the servos.
- Check the Waveshare board's jumper is set to USB.
- Check nothing else, such as a running teleop script, has the COM port open.

Swap `--leader-left`/`--leader-right` if the sides are mirrored. If a joint moves the
wrong way, flip it in `LEADER_SIGN` in `tools/leader_arms.py`.

With `--robot-ip`, don't pass `--leader-*`. The teleop script owns the leader ports there,
and the twin already shows the real follower arms.

Add `--robot-model alohamini2` (or `alohamini2pro`) if that's the lab robot. It sets wheel
geometry and lift speed and must match the model the Pi's Host runs with.

## Controls

| Input | Action |
|---|---|
| W / S (or ↑ / ↓) | forward / back |
| A / D (or ← / →) | strafe left / right |
| Q / E | rotate left / right |
| 1 / 2 / 3 or Slow/Med/Fast | 0.15 / 0.20 / 0.25 m/s, 45 / 60 / 75 °/s |
| Lift track: click or drag | set a height goal (0–600 mm); the lift travels there and holds |
| R / F held | jog the lift up / down; stops on release |
| Space or STOP | stop everything; press again to resume |

The on-screen keys also work with mouse or touch.

## Safety behaviour

- **One operator.** The first tab to connect gets control; other tabs are view-only until it's released.
- **Deadman.** Stopping, a lost connection, a hidden tab or no input for 0.3 s means zero velocity. Keys are released when the window loses focus.
- **Quiet when idle.** 0.5 s after the last input the bridge sends a stop, then stops sending. The Host's 1 s watchdog then releases command ownership.
- **Fresh feedback only.** The robot backend sends only while Host feedback is newer than 250 ms, like `AlohaMiniClient`.
- **Limits.** The server clamps speeds to the fast keyboard level. The lift target ramps at up to 150 mm/s and stays within 50 mm of the measured height. The real lift moves at about 27 mm/s (alohamini1) or 42 mm/s (alohamini2), and the twin models that speed.

## Before driving the real robot: the teleop conflict

The Host accepts commands from **one client at a time** (`command_owner.py`). While the web UI
is driving, arm commands from the leader-arm teleop client are **rejected**: the arms hold still
until the bridge goes quiet and the watchdog releases ownership (about 1.5 s after the last
input). Driving and arm teleop therefore can't happen at the same moment. To fix this
together with the Pi side, either:

1. have the bridge relay the teleop client's arm targets, so one sender carries base + lift + arms; or
2. relax `CommandOwner` on the Host so base/lift and arm keys can come from different clients.

## Test without hardware

```bash
python hybrid_twin/web/mock_host.py                               # fake Host, real CommandOwner
python hybrid_twin/web/server.py --robot-ip 127.0.0.1 --port 8081
python hybrid_twin/web/test_bridge.py --url http://127.0.0.1:8081
```

`python hybrid_twin/tools/test_leader_arms.py` tests the leader reader against a fake servo bus
(real protocol packets, and it asserts the reader never writes).

`test_bridge.py` also runs against the sim backend (default URL). It checks:
- distance and angle accuracy;
- lift travel and hold;
- going quiet when idle;
- STOP;
- control hand-off.

Don't point it at the real robot unless the robot has clear floor space: it drives 0.4 m.
