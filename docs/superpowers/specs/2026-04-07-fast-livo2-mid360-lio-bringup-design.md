# FAST-LIVO2 MID360 LIO Bring-Up Design

**Date:** 2026-04-07

## Goal

Bring up `FAST-LIVO2` on the real UAV in pure LiDAR-inertial mode using `MID360` only.

This stage explicitly excludes `D435i` visual fusion. The immediate objective is to verify that `FAST-LIVO2` can subscribe to `MID360` point cloud and IMU data, initialize successfully, and publish stable odometry and registered point clouds on hardware.

## Scope

This design covers:

- `livox_ros_driver2` as the `MID360` source
- `FAST-LIVO2` in `ONLY_LIO` mode
- a repository-local parameter set for first-flight smoke testing
- the initial validation and triage sequence for real hardware

This design does not cover:

- `D435i` topic integration
- LiDAR-camera extrinsic calibration
- visual front-end tuning
- PX4 or MAVROS downstream integration beyond observing published pose topics

## Constraints

- The hardware stack is `MID360 + D435i`, but this phase must ignore camera input.
- `MID360` SDK and driver setup are already complete enough to launch and inspect topics.
- `FAST-LIVO2` already builds in the current workspace.
- `FAST-LIVO2` was written around Livox `CustomMsg` and hard-synchronized visual examples, so the safest first step is to minimize variables.
- The current `FAST-LIVO2` code loads camera parameters unconditionally during startup, even when image input is disabled. A valid camera parameter file is still required for pure `LIO`.

## Approaches Considered

### 1. Full `MID360 + D435i` `LIVO` bring-up immediately

This is the fastest path to “use both sensors,” but it combines too many unknowns at once: image topic choice, camera intrinsics, LiDAR-camera extrinsics, and visual timing. Without calibrated `MID360 -> D435i` extrinsics, failures would be ambiguous and expensive to debug.

### 2. Validate `MID360` with another LIO package first, then return to `FAST-LIVO2`

This would isolate whether hardware and timestamps are healthy, but it does not directly prove that the intended `FAST-LIVO2` runtime path is healthy. It also adds a redundant integration step.

### 3. Recommended: run `FAST-LIVO2` as pure `LIO` first

Disable image input, preserve the `MID360` LiDAR and IMU topics, and verify initialization, undistortion, mapping, and odometry publication. This keeps the test inside the target package while deferring visual complexity until the LiDAR-inertial baseline is trustworthy.

## Recommended Design

### Input Graph

- `livox_ros_driver2` publishes:
  - `/livox/lidar` as `livox_ros_driver2/msg/CustomMsg`
  - `/livox/imu` as `sensor_msgs/msg/Imu`
- `FAST-LIVO2` subscribes only to those two topics during this phase.
- `FAST-LIVO2` must run with `common.img_en: 0`, which causes the code to select `ONLY_LIO` mode.

### Parameter Strategy

Create a repository-local `MID360` pure-LIO parameter file instead of modifying the upstream sample configs in place.

Required settings for the first smoke test:

- `common.img_en: 0`
- `common.lidar_en: 1`
- `common.lid_topic: "/livox/lidar"`
- `common.imu_topic: "/livox/imu"`
- `preprocess.lidar_type: 1`
- `preprocess.feature_extract_enabled: false`
- `preprocess.scan_line: 6` for the first smoke test only
- `imu.imu_en: true`
- `extrin_calib.extrinsic_T: [-0.011, -0.02329, 0.04412]`
- `extrin_calib.extrinsic_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]`

The `scan_line` value remains `6` initially because the first goal is a minimal smoke test, not a final tuned configuration. If the system initializes but shows unstable motion or degraded map quality, `scan_line: 4` becomes an early follow-up check because the current Livox driver identifies `MID360` as a 4-line device.

The supplied extrinsic translation is treated as the first candidate `LiDAR -> IMU` transform because `FAST-LIVO2` applies `extrinsic_R/T` in that direction during point transformation and IMU-based undistortion. If the stack produces motion but degrades badly under rotation or acceleration, verify the translation sign before changing secondary parameters.

### Launch Strategy

Do not use the upstream `mapping_avia.launch.py` for the first real-hardware smoke test. That launch file still includes image transport republish logic and assumes `/left_camera/image`, which is unnecessary in pure `LIO`.

Use a two-process bring-up:

1. Launch `livox_ros_driver2` for `MID360`.
2. Run `fast_livo` directly with:
   - the new repository-local `MID360` LIO parameter file
   - a valid camera parameter file

The camera parameter file is still required because `FAST-LIVO2` loads camera model parameters during startup even when image processing is disabled. This is a startup requirement, not an indication that images are in use.

### Expected Outputs

Successful first-stage bring-up means:

- `FAST-LIVO2` logs `FIRST LIDAR FRAME!`
- IMU initialization progresses and eventually prints the `IMU Initials` summary
- the following topics are published:
  - `/cloud_registered`
  - `/aft_mapped_to_init`
  - `/path`

RViz quality is secondary. The first success criterion is correct runtime progression through initialization and steady publication of LiDAR-inertial outputs.

## Failure Triage Order

When the first smoke test fails, debug in this order:

1. Confirm `/livox/lidar` and `/livox/imu` both exist and have advancing timestamps.
2. Confirm `FAST-LIVO2` enters `ONLY_LIO` mode by verifying `img_en: 0` in the active params.
3. Confirm startup is not blocked by missing camera parameters.
4. If the node starts but never leaves initialization, prioritize IMU availability and timestamp health.
5. If odometry publishes but is clearly unstable, prioritize:
   - `extrinsic_T` direction
   - LiDAR/IMU timing quality
   - `scan_line` adjustment from `6` to `4`

Do not involve `D435i`, RGB topics, image republish, or LiDAR-camera calibration until the pure `LIO` baseline is stable.

## Follow-On Work After Approval

Once pure `MID360` `LIO` is stable:

1. Move the launch logic into a project-owned overlay package or wrapper launch file.
2. Freeze the proven `MID360` LiDAR-IMU parameter set.
3. Start a separate subproject for `D435i` visual integration, beginning with image topic selection, intrinsics, and LiDAR-camera extrinsic calibration.
