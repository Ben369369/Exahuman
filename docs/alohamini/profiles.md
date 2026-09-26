# Hardware Profile Reference

The host-side `--robot_model` flag and PC-side `--robot.robot_model` / `--teleop.arm_profile`
flags select your hardware variant.

Use the same model value for host-side `--robot_model` and PC-side `--robot.robot_model`.
Use `--teleop.arm_profile` only for the leader arm connected to the PC.

## AlohaMini host-side (`--robot_model`)

| `--robot_model` | Follower arm | Base wheels | Lift motor | Lead screw |
|-----------------|--------------|-------------|------------|------------|
| `alohamini1` | `so-arm-5dof` | STS3215 ×3 | STS3215 | 84 mm/rev |
| `alohamini2` | `am-follower-6dof` | STS3215 ×3 | STS3095 | 131 mm/rev |
| `alohamini2pro` | `am-follower-6dof-hd` | STS3250 ×3 | STS3095 | 131 mm/rev |

## AM-ARM200 arm profiles (`--teleop.arm_profile`)

| Product SKU | Role | `--teleop.arm_profile` |
|-------------|------|-----------------|
| AM-ARM200 | Leader (5V) | `am-leader-6dof` |
| AM-ARM200 | Follower (12V) | `am-follower-6dof` |
| AM-ARM200 Pro | Leader (5V) | `am-leader-6dof` |
| AM-ARM200 Pro | Follower (12V HD) | `am-follower-6dof-hd` |
