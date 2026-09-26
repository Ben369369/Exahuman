# AM-ARM200 — Full Workflow

> **Prerequisites:** complete [install.md](install.md) first.  
> **Hardware profiles:** see [profiles.md](profiles.md).

Single-arm setup — one PC, no Raspberry Pi required.

---

## 1. Port Configuration

Plug the leader in first, run the finder, note the port. Then plug in the follower and note its port:

```bash
lerobot-find-port
# or check directly:
ls /dev/ttyACM*
```

> Port numbers can change after reconnecting or rebooting. Note each path before moving to the next device.

## 2. Camera Configuration

```bash
lerobot-find-cameras
```

Note each camera index. Use one USB port per camera — do not share a USB hub between multiple cameras.

---

## 3. Calibration

### Calibrate the leader

```bash
lerobot-calibrate \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM0 \
  --teleop.id=my_leader \
  --teleop.arm_profile=am-leader-6dof
```

### Calibrate the follower

```bash
lerobot-calibrate \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM1 \
  --robot.id=my_follower \
  --robot.arm_profile=am-follower-6dof
```

> AM-ARM200 Pro: replace `am-follower-6dof` with `am-follower-6dof-hd`.

Power-cycle both arms after calibration for changes to take effect.

---

## 4. Teleoperation

```bash
lerobot-teleoperate \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM0 \
  --teleop.id=my_leader \
  --teleop.arm_profile=am-leader-6dof \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM1 \
  --robot.id=my_follower \
  --robot.arm_profile=am-follower-6dof
```

Move the leader by hand — the follower mirrors it in real time.

---

## 5. Dataset Recording

Make sure both arms are connected and calibrated before recording. Camera indices come from `lerobot-find-cameras` (see §2).

Create new dataset:

```bash
lerobot-record \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM1 \
  --robot.id=my_follower \
  --robot.arm_profile=am-follower-6dof \
  --robot.cameras="{cam_wrist: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, cam_top: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM0 \
  --teleop.id=my_leader \
  --teleop.arm_profile=am-leader-6dof \
  --dataset.repo_id=$HF_USER/am_arm_test \
  --dataset.num_episodes=50 \
  --dataset.fps=30 \
  --dataset.episode_time_s=45 \
  --dataset.reset_time_s=8 \
  --dataset.single_task "pickup1" \
  --display_data=true
```

Resume existing dataset — `--dataset.root` specifies a local working directory; LeRobot downloads the existing dataset metadata from Hub automatically if the path doesn't exist yet:

```bash
lerobot-record \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM1 \
  --robot.id=my_follower \
  --robot.arm_profile=am-follower-6dof \
  --robot.cameras="{cam_wrist: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, cam_top: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM0 \
  --teleop.id=my_leader \
  --teleop.arm_profile=am-leader-6dof \
  --dataset.repo_id=$HF_USER/am_arm_test \
  --dataset.root=$HOME/lerobot_datasets/$HF_USER/am_arm_test \
  --dataset.num_episodes=50 \
  --dataset.fps=30 \
  --dataset.episode_time_s=45 \
  --dataset.reset_time_s=8 \
  --dataset.single_task "pickup1" \
  --display_data=true \
  --resume=true
```

> AM-ARM200 Pro: replace `am-follower-6dof` with `am-follower-6dof-hd`.  
> Camera names (`cam_wrist`, `cam_top`) are arbitrary but become dataset field names — keep them consistent across all recording sessions.  
> To use a single camera, remove the `cam_top` entry.

---

## 6. Dataset Replay

Replay a recorded episode on the follower arm to verify the data:

```bash
lerobot-replay \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM1 \
  --robot.id=my_follower \
  --robot.arm_profile=am-follower-6dof \
  --dataset.repo_id=$HF_USER/am_arm_test \
  --dataset.episode=0
```

> AM-ARM200 Pro: replace `am-follower-6dof` with `am-follower-6dof-hd`.

---

## 7. Dataset Visualization

```bash
lerobot-dataset-viz \
  --repo-id $HF_USER/am_arm_test \
  --episode-index 0 \
  --display-compressed-images
```

---

## 8. Training

### Local training

```bash
lerobot-train \
  --dataset.repo_id=$HF_USER/am_arm_test \
  --policy.type=act \
  --output_dir=outputs/train/act_your_dataset1 \
  --job_name=act_your_dataset \
  --policy.device=cuda \
  --wandb.enable=false \
  --policy.repo_id=$HF_USER/act_policy \
  --dataset.video_backend=pyav
```

### No local GPU?

Use any cloud GPU provider (e.g. AutoDL, Lambda Labs, Vast.ai). Set up the environment the same way as local, run the same training command, then copy the checkpoint back to your machine for evaluation.

---

## 9. Evaluation

Copy the trained model to your local machine, then run inference on the follower arm:

```bash
lerobot-rollout \
  --strategy.type=base \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM1 \
  --robot.id=my_follower \
  --robot.arm_profile=am-follower-6dof \
  --policy.path=outputs/train/act_your_dataset1/checkpoints/020000/pretrained_model \
  --task="your task description"
```

> AM-ARM200 Pro: replace `am-follower-6dof` with `am-follower-6dof-hd`.

---

## 10. Debug

See [Debug Command Summary](../../examples/debug/README.md) for the full list of debugging utilities.
