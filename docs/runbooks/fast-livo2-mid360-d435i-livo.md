# FAST-LIVO2 MID360 D435i LIVO Runbook

This runbook covers MID360 + D435i calibration capture, FAST-Calib extrinsic solve, runtime timing tune, and flight-readiness checks.

Assumption: `slam_ws` and `overlay_ws` dependencies are already built and the helper scripts from this branch are available.

## 1. Calibration Capture

1. Edit `slam_ws/src/livox_ros_driver2/config/MID360_config.json` for machine-local IP settings (`host_*` and `lidar_configs[0].ip`).
2. Run:

```bash
bash scripts/capture_fast_calib_sample.sh
```

3. Hold the FAST-Calib board steady in view of both sensors for 2-3 seconds.
4. Stop bag recording with `Ctrl+C`.
5. Save one matching `infra1` PNG as:
   `/tmp/fast_calib/calibration_sample.png`

Expected capture outputs:
- Bag: `/tmp/fast_calib/calibration_sample`
- Image: `/tmp/fast_calib/calibration_sample.png`

## 2. FAST-Calib

Run FAST-Calib with the project-owned parameter file:

```bash
REPO_ROOT="$(git rev-parse --show-toplevel)"
source /opt/ros/humble/setup.bash
source "$REPO_ROOT/slam_ws/install/setup.bash"
ros2 run fast_calib fast_calib --ros-args \
  -r __node:=fast_calib \
  --params-file "$REPO_ROOT/overlay_ws/src/fast_livo2_mid360_bringup/config/fast_calib_mid360_d435i.yaml"
```

Copy the reported `T_cam_lidar` result into:
`overlay_ws/src/fast_livo2_mid360_bringup/config/fast_livo2_mid360_livo_d435i.yaml`

- Rotation -> `extrin_calib.Rcl`
- Translation -> `extrin_calib.Pcl`

## 3. Runtime Timing Tune

1. Start from `img_time_offset: 0.0` in `fast_livo2_mid360_livo_d435i.yaml`.
2. Run:

```bash
bash scripts/run_mid360_livo_d435i.sh
```

3. Perform hand-carried yaw and translation motion.
4. Between runs, adjust only `time_offset.img_time_offset`.
5. Keep the first value that clearly reduces map tearing and pose lag.

## 4. Flight Readiness

- Pure `LIO` still passes.
- `FAST-Calib` result reproduced twice.
- `/rgb_img`, `/cloud_visual_map`, `/path`, and `/aft_mapped_to_init` appear.
- Ground hand-carried `LIVO` is at least as stable as pure `LIO`.
- Only then move to static-on-airframe and low-risk flight tests.
