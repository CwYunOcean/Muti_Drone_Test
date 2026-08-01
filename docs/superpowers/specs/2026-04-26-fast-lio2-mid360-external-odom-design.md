# FAST-LIO2 MID360 External Odom Design

**Date:** 2026-04-26

## Goal

Add a project-owned FAST-LIO2 bring-up path that:

- starts `MID360` and `FAST-LIO2` together from this repo
- preserves the current downstream odometry contract by exposing `/aft_mapped_to_init`
- keeps `/cloud_registered` on the FAST-LIO2 native topic
- applies a fixed `-30 deg` pitch compensation in the FAST-LIO2 LiDAR-to-IMU extrinsic config

## Problem Statement

The current single-drone real-hardware stack expects LiDAR odometry on `/aft_mapped_to_init`.
Upstream `FAST-LIO2` publishes odometry on `/Odometry` instead, so the existing planner and PX4 bridge do not consume it without additional wiring.

There is also a source-side tilt issue: when RViz uses `camera_init` or `body` as the fixed frame, the registered ground plane still appears pitched. That means the problem is not only downstream body-frame interpretation. The world-frame point cloud itself is being reconstructed with a fixed rotation error.

## Root-Cause Hypothesis

For this hardware setup, the current FAST-LIO2 `mid360.yaml` LiDAR-to-IMU rotation is not matching the effective sensor orientation seen by the algorithm. Because the world-frame cloud remains tilted even in `camera_init`, the correction belongs in FAST-LIO2's `mapping.extrinsic_R`, not in a downstream `base_link` static transform.

## Scope

This change should:

- create a new local package under `overlay_ws/src/`
- keep upstream `slam_ws/src/FAST_LIO` untouched
- use a project-owned FAST-LIO2 config file
- disable FAST-LIO2 online extrinsic estimation for this path
- set the default pitch compensation to `-30 deg`
- remap FAST-LIO2 odometry from `/Odometry` to `/aft_mapped_to_init`
- add a new helper script under `scripts/`

This change should not:

- modify the existing `FAST-LIVO2` bring-up package
- change planner or PX4 bridge packages
- redesign downstream TF trees

## Design

### 1. New local bring-up package

Create `overlay_ws/src/fast_lio2_mid360_bringup`.

The package owns:

- one launch file for `MID360 + FAST-LIO2`
- one FAST-LIO2 config override file
- package-local contract tests

### 2. FAST-LIO2 launch behavior

The new launch file starts:

- `livox_ros_driver2_node` for `MID360`
- `fast_lio/fastlio_mapping`

The launch remaps:

- `/Odometry` -> `/aft_mapped_to_init`

The launch leaves these native outputs unchanged:

- `/cloud_registered`
- `/cloud_registered_body`
- `/path`

### 3. Project-owned FAST-LIO2 config

The new config is based on upstream `mid360.yaml` but changes only the source-side rotation assumptions required by this hardware path:

- `mapping.extrinsic_est_en: false`
- `mapping.extrinsic_R`: fixed pitch compensation of `-30 deg`

Initial rotation matrix:

```text
[ 0.8660254, 0.0, -0.5,
  0.0,       1.0,  0.0,
  0.5,       0.0,  0.8660254 ]
```

Translation remains aligned with the existing MID360 values.

## Expected Runtime Outcome

After launch:

- `/livox/lidar` and `/livox/imu` should stream normally
- FAST-LIO2 odometry should be available on `/aft_mapped_to_init`
- `/cloud_registered` should remain available
- the ground plane in RViz should be closer to level when viewed in `camera_init`

## Verification

The implementation is acceptable when:

1. the new package builds in `overlay_ws`
2. package-local launch/config/script contract tests pass
3. `run_mid360_fastlio2.sh` starts the new bring-up entrypoint
4. the user can test the new source-side pitch compensation without touching upstream FAST-LIO2 files
