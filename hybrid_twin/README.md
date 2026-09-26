# hybrid_twin — AlohaMini digital twin

A MuJoCo + URDF digital twin of the AlohaMini, assembled from two repos:

| Part | Source | Status |
|---|---|---|
| SO-101 arm (URDF, MJCF, 26 STL meshes, STS3215 motor params) | `TheRobotStudio/SO-ARM100` → `Simulation/SO101` | **Real CAD geometry** |
| SO-100 arm (URDF + meshes) | `SO-ARM100` → `Simulation/SO100` | Real, kept for reference |
| Base kinematics (wheel radius, base radius, wheel angles), lift travel, action keys | this repo, `src/lerobot/robots/alohamini/` | Real values from the driver |
| Chassis, column, lift carriage, arm mount offsets | `tools/build_twin.py` `GEOM` | **Placeholders (`MEASURE`)** |
| Camera mount poses (alohamini2pro) | `alohamini_sim/video2sim/.../cameras_am2pro.yaml` | Copied as `cameras_am2pro.yaml` |

Why placeholders: the AlohaMini base meshes, URDFs and USDs in `alohamini_sim/` are
**Git LFS pointer stubs** (41 STL + 5 USD, ~130 bytes each), and `*.urdf` / `*.xml` are
in `.gitignore`. Neither repo contains the real base geometry. Get it with
`git lfs pull` from the original clone or from the upstream AlohaMini repo, or measure the
lab robot and edit `GEOM`.

The SO-101 arm has the same six joints as the `alohamini1` arm profile
(`so-arm-5dof`). `alohamini2`/`2pro` arms add a `wrist_yaw` joint the SO-101 model
doesn't have; the twin ignores that joint for now.

## Layout

```
hybrid_twin/
├── arms/so101/            SO-101 URDF + MJCF (+ scene.xml with pickup block) + assets/*.stl
├── arms/so100/            SO-100 URDF + meshes
├── arms/LICENSE-SO-ARM100 Apache-2.0 licence of the copied arm files
├── mujoco/alohamini_twin.xml   GENERATED full robot (base + lift + 2 arms)
├── urdf/alohamini_twin.urdf    GENERATED same robot for the web (three.js / urdf-loaders)
├── cameras_am2pro.yaml
├── web/                    browser control UI + bridge (sim or real robot), see web/README.md
└── tools/
    ├── build_twin.py      regenerates both files from one parameter set
    ├── twin_driver.py     drives the twin with the real robot's action keys; maps real state → twin
    └── check_twin.py      smoke test (+ --viewer demo)
```

## Run

Needs only `mujoco` (3.5.0 verified) and `numpy`:

```bash
python hybrid_twin/tools/build_twin.py [--robot-model alohamini1|alohamini2|alohamini2pro]
python hybrid_twin/tools/check_twin.py            # headless checks
python hybrid_twin/tools/check_twin.py --viewer   # opens MuJoCo viewer with a scripted tour
python -m mujoco.viewer --mjcf=hybrid_twin/arms/so101/scene.xml   # original single-arm sim
```

`check_twin.py` checks that:
- forward drive, rotation and body→world frame conversion are correct;
- the lift reaches 300 mm;
- the arms track their targets;
- lift/arm-only commands leave the base still;
- the 1 s watchdog stops the base.

## Web control

```bash
pip install -r hybrid_twin/web/requirements.txt
python hybrid_twin/web/server.py                     # drive the MuJoCo twin at http://localhost:8080
python hybrid_twin/web/server.py --robot-ip <pi-ip>  # drive the real robot
```

See [web/README.md](web/README.md) for controls, safety behaviour and the teleop ownership
conflict to resolve with the Pi side.

## Control contract (same for twin and real robot)

The twin accepts exactly what the AlohaMini Host takes on ZMQ port 5555:

| Web control | Key | Units | Client default |
|---|---|---|---|
| W / S | `x.vel` | m/s, + forward | ±0.15 / 0.2 / 0.25 |
| strafe | `y.vel` | m/s, + left | same |
| A / D rotate | `theta.vel` | **deg/s**, + CCW | ±45 / 60 / 75 |
| lift lever | `lift_axis.height_mm` | absolute, 0–600 mm; real speed ~27 mm/s (alohamini1) | — |
| arms | `arm_{left,right}_<joint>.pos` | from leader-arm teleop | — |

Rules copied from the Host, which the web client must follow too:
- **Stream commands at 20–50 Hz.** The Host stops the base if no command arrives within 1 s.
- **A command with no base keys means base velocity 0.**
- **Port 5555 accepts only one command sender at a time** (`CommandOwner`), so the web bridge and arm teleop have to be merged into one sender.

`TwinDriver.apply_robot_state(obs, obs["_robot_metadata"])` mirrors a real Host observation
(port 5556) onto the twin, converting normalized motor values to radians with the
calibration the Host publishes.

## To verify on the lab robot

1. Measure the base, column, carriage and arm mount positions, then update `GEOM`.
2. Check which `robot_model` the lab robot is. The keyboard layout in `config_alohamini.py`
   uses W/S forward/back, Z/X strafe, A/D rotate and U/J lift.
3. Check each arm joint's direction against the real arm, and flip `JOINT_SIGN` where it's reversed.
4. Check that the SO-101 gripper angle matches LeRobot's 0–100 scale (see `arms/so101/README.md`).
5. Check the sign conventions of `x.vel` / `y.vel` on the hardware.
