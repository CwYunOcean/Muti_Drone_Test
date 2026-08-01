# FAST-LIVO2 MID360 + D435i LIVO Design

## Goal

Bring `FAST-LIVO2` to a field-usable `LIVO` setup on real hardware using:

- `Livox MID360` for LiDAR and IMU
- `Intel RealSense D435i` `infra1` for monocular visual input

The scope includes:

- LiDAR-camera extrinsic calibration
- software-only image timing alignment
- project-owned launch, config, RViz, and script integration
- preserving the existing pure-`LIO` baseline

## Current State

The repository already has a working pure-`LIO` bring-up in
`overlay_ws/src/fast_livo2_mid360_bringup/`:

- `mid360_lio.launch.py`
- `fast_livo2_mid360_lio.yaml`
- `run_mid360_lio.sh`

This path must remain intact as a stable fallback.

`FAST-LIVO2` consumes:

- one Livox point cloud topic
- one IMU topic
- one image topic

It does not subscribe to `camera_info` or RealSense depth topics. Camera model
parameters are loaded from YAML, and image timestamps are adjusted with
`time_offset.img_time_offset`.

## Constraints

1. `MID360` and `D435i` have no hardware sync.
2. `FAST-Calib-ROS2` is available locally at
   `slam_ws/src/FAST-Calib-ROS2`.
3. This `FAST-Calib-ROS2` variant uses offline inputs:
   - `ros2 bag` containing `sensor_msgs/PointCloud2`
   - one corresponding image file
4. Current runtime `FAST-LIVO2` uses Livox `CustomMsg`, so calibration and
   runtime data paths must be separated.

## Chosen Approach

Use two independent project-owned flows:

### 1. Calibration Flow

Purpose: generate reliable `MID360 -> D435i` extrinsics.

- Run `MID360` in `PointCloud2` mode during calibration capture.
- Run `D435i` with `infra1/image_rect_raw`.
- Handhold the calibration target and collect a short bag dedicated to
  calibration.
- Save one matching `infra1` image.
- Feed the bag and image into `FAST-Calib-ROS2`.
- Convert the resulting `T_cam_lidar` into `FAST-LIVO2`
  `extrin_calib.Rcl` and `extrin_calib.Pcl`.

This flow is offline and does not share launch parameters with runtime except
for topic names and camera intrinsics.

### 2. Runtime LIVO Flow

Purpose: run `FAST-LIVO2` on the real sensor suite.

- Keep Livox runtime on `CustomMsg`.
- Keep `MID360` IMU as the only IMU source.
- Use `D435i infra1/image_rect_raw` as the visual topic.
- Enable `common.img_en: 1`.
- Load calibrated `Rcl/Pcl`.
- Tune `time_offset.img_time_offset` on hardware.

## Why `infra1`

Use `D435i infra1` before RGB because:

- `FAST-LIVO2` converts incoming images to grayscale anyway.
- `infra1` is the most direct fit for a stable monocular tracking input.
- this avoids mixing geometric validation with color-map concerns.

RGB support can be added later if needed for colored visualization, but it is
not part of the first usable `LIVO` milestone.

## Files To Add

Under `overlay_ws/src/fast_livo2_mid360_bringup/` add:

- `config/fast_livo2_mid360_livo_d435i.yaml`
- `config/d435i_infra1_pinhole.yaml`
- `config/fast_calib_mid360_d435i.yaml`
- `launch/mid360_livo_d435i.launch.py`
- `launch/mid360_d435i_calib_capture.launch.py`
- `rviz/fast_livo2_livo_d435i.rviz`
- `test/test_mid360_livo_d435i_config_contract.py`
- `test/test_mid360_livo_d435i_launch_contract.py`

At repo root add:

- `scripts/capture_fast_calib_sample.sh`
- `scripts/run_mid360_livo_d435i.sh`
- a runbook documenting calibration, parameter handoff, timing tune, and
  flight-readiness checks

## Parameter Ownership

### Runtime FAST-LIVO2 config

`fast_livo2_mid360_livo_d435i.yaml` owns:

- `common.img_topic`
- `common.img_en`
- `extrin_calib.Rcl`
- `extrin_calib.Pcl`
- `time_offset.img_time_offset`
- any visual frontend tuning kept different from pure `LIO`

It should not overwrite the pure-`LIO` calibration defaults unless necessary.

### Camera model config

`d435i_infra1_pinhole.yaml` owns the exact `infra1`:

- width and height
- focal lengths
- principal point
- distortion coefficients
- image scale used by `FAST-LIVO2`

### FAST-Calib config

`fast_calib_mid360_d435i.yaml` owns:

- `lidar_topic`
- `bag_path`
- `image_path`
- target geometry
- LiDAR spatial filtering window

## Calibration Procedure

1. Verify `D435i infra1/image_rect_raw` is stable at the chosen resolution and
   frame rate.
2. Launch calibration capture with `MID360` in `PointCloud2` mode.
3. Record a short bag containing the calibration target from multiple angles
   but limited motion blur.
4. Save one corresponding `infra1` image used for `FAST-Calib`.
5. Run `FAST-Calib-ROS2`.
6. Reject calibration if projected centers or RMSE are clearly poor.
7. Repeat once and compare results. Large differences indicate unstable
   capture or bad filtering bounds.

## Timing Alignment Procedure

Timing is not solved by `FAST-Calib`. It is handled separately in
`FAST-LIVO2` using `time_offset.img_time_offset`.

Tune on hardware in this order:

1. Start from `0.0`.
2. Perform hand-carried translational and yaw motions.
3. Observe map tearing, visual instability, and pose consistency.
4. Adjust only `img_time_offset`, one direction at a time.
5. Lock the first clearly stable value before any flight test.

This is a practical software-sync procedure, not a claim of true sensor
synchronization.

## Validation Gates

The implementation is accepted only if all gates pass in order:

1. Pure `LIO` still runs unchanged.
2. Calibration capture launch works.
3. `FAST-Calib` produces repeatable extrinsics.
4. `FAST-LIVO2` starts in `LIVO` mode with `infra1`.
5. Visual topics appear in RViz, including image and visual-map outputs.
6. Ground hand-carried `LIVO` is at least as stable as pure `LIO`.
7. Only then proceed to static-on-airframe and low-risk flight testing.

## Risks

Primary risks are:

- weak software-only synchronization
- poor `infra1` exposure under motion or lighting changes
- bad LiDAR-camera capture pairing during calibration
- confusing runtime and calibration data paths

The design addresses this by isolating calibration from runtime, preserving the
pure-`LIO` fallback, and treating timing offset as a dedicated tuning step
instead of guessing it from geometry calibration.
