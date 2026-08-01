# FAST-LIVO2 MID360 LIO Bring-Up

This runbook launches `MID360` and `FAST-LIVO2` in pure `LIO` mode only. It does not use `D435i`.

Assumption: the workspace that provides `fast_livo` and `livox_ros_driver2` is already built and sourced.

### Overlay-only worktree note

If `REPO_ROOT` points to an overlay-only worktree, define `BASE_REPO_ROOT=/path/to/main/checkout` and substitute `$BASE_REPO_ROOT/slam_ws/install/setup.bash` for the missing `$REPO_ROOT/slam_ws/install/setup.bash`. If that worktree also does not contain `slam_ws/src/livox_ros_driver2`, substitute `$BASE_REPO_ROOT/slam_ws/src/livox_ros_driver2/config/MID360_config.json` for the `livox_config_file` path.

## 1. Build the project-owned bring-up package

```bash
REPO_ROOT="$(git rev-parse --show-toplevel)"
source /opt/ros/humble/setup.bash
source "$REPO_ROOT/slam_ws/install/setup.bash"
cd "$REPO_ROOT/overlay_ws"
colcon build --symlink-install --packages-select fast_livo2_mid360_bringup
source "$REPO_ROOT/overlay_ws/install/setup.bash"
```

### Package-local verification

```bash
colcon test --packages-select fast_livo2_mid360_bringup
colcon test-result --verbose
```

## 2. Launch the stack

Edit the Livox JSON config first:

- `slam_ws/src/livox_ros_driver2/config/MID360_config.json`
- Set machine-local host-side IP fields and `lidar_configs[0].ip` for your network before launch.
- Do not commit machine-local IP edits.

`frame_id` caveat: this launch argument only affects Livox point cloud/custom outputs; upstream `livox_ros_driver2` IMU messages remain on `livox_frame`.

Then launch:

```bash
REPO_ROOT="$(git rev-parse --show-toplevel)"
source /opt/ros/humble/setup.bash
source "$REPO_ROOT/slam_ws/install/setup.bash"
source "$REPO_ROOT/overlay_ws/install/setup.bash"
ros2 launch fast_livo2_mid360_bringup mid360_lio.launch.py \
  livox_config_file:="$REPO_ROOT/slam_ws/src/livox_ros_driver2/config/MID360_config.json"
```

## 3. Expected runtime milestones

- `FAST-LIVO2` prints `FIRST LIDAR FRAME!`
- IMU logs include `IMU Initializing` followed by `IMU Initials`
- topics `/cloud_registered`, `/aft_mapped_to_init`, and `/path` appear

## 4. Second-shell checks

Expected behavior: `ros2 topic echo /aft_mapped_to_init --once` returns one odometry message instead of blocking indefinitely.

```bash
REPO_ROOT="$(git rev-parse --show-toplevel)"
source /opt/ros/humble/setup.bash
source "$REPO_ROOT/slam_ws/install/setup.bash"
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic list | grep -E '/cloud_registered|/aft_mapped_to_init|/path'
ros2 topic echo /aft_mapped_to_init --once
```

## 5. First-pass triage order

1. Verify `/livox/lidar` and `/livox/imu` exist and timestamps advance.
2. Verify `img_en: 0` in `config/fast_livo2_mid360_lio.yaml`.
3. Verify the launch uses a valid `livox_config_file`.
4. If the node starts but never finishes initialization, inspect IMU frequency and timestamp continuity first.
5. If odometry publishes but degrades badly under motion, verify the sign of `extrinsic_T` before changing `scan_line`.
6. If motion is still unstable after the extrinsic sign check, retry with `preprocess.scan_line: 4`.
