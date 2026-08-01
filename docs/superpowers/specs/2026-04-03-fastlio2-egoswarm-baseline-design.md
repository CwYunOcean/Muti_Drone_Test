# FAST-LIO2 + EGO-Swarm Baseline Bring-Up Design

**Date:** 2026-04-03

## Goal

Create a minimal-impact integration repository for bringing up MID360, FAST-LIO2, D435i, and EGO-Swarm on a Jetson-class Ubuntu 22.04 / ROS2 Humble machine.

The immediate goal is not full autonomous flight. The immediate goal is a clean baseline where the upstream sensor, SLAM, and planner stacks can each build and start with reproducible commands and pinned revisions.

## Constraints

- The machine is Jetson-class ARM64.
- ROS2 Humble is already installed.
- Upstream source changes should be minimized.
- Large clone operations should remain manual.
- MID360 uses `livox_ros_driver2`, whose build script wipes workspace build products.
- D435i should use released ROS2 packages first, not custom librealsense source builds.

## Approaches Considered

### 1. One monolithic workspace with direct upstream edits

This is the fastest way to start hacking, but it is the least stable for this stack. `livox_ros_driver2/build.sh` removes workspace build artifacts before rebuilding, which makes a shared workspace fragile. Direct edits also make it hard to distinguish project code from upstream patches.

### 2. Track every upstream repository directly inside this git repo

This would make the whole stack visible in one repository, but it bloats the integration repo, creates noisy history, and increases the chance of accidental upstream edits. It also makes branch hygiene worse on a Jetson where build products are large.

### 3. Recommended: meta-repo + isolated workspaces + local overlay packages

Keep `Drone_SLAM` as a thin integration repo that stores manifests, docs, local packages, and patch files. Clone upstream repos into ignored workspace directories, and build `livox_ros_driver2` in its own dedicated workspace. Keep all project-specific logic in `overlay_ws/src`.

This satisfies the user's minimal-impact requirement and keeps the migration path open if individual upstream packages later need to be replaced or patched.

## Recommended Design

### Repository Roles

- `livox_ws/` is a dedicated workspace for `Livox-SDK2` and `livox_ros_driver2`.
- `slam_ws/` holds `FAST_LIO` on the official `ROS2` branch and `ego-planner-swarm` on `ros2_version`.
- `overlay_ws/` is reserved for local launch wrappers, adapters, and deployment code.
- `patches/` stores reproducible patch files if upstream edits become unavoidable.

### Why Separate `livox_ws`

`livox_ros_driver2/build.sh` clears the parent workspace's `build/`, `install/`, and `log/` directories. If it lives beside FAST-LIO or EGO-Swarm, every Livox rebuild risks invalidating the rest of the workspace. The clean solution is a dedicated Livox workspace that downstream workspaces source as an underlay.

### Sensor Strategy

- MID360 is the primary localization sensor for the first milestone.
- D435i is brought up independently with released ROS2 Humble packages.
- D435i is not fused into FAST-LIO in the first milestone.
- D435i is not yet the planner's primary obstacle source in the first milestone.

This keeps the first milestone bounded and avoids mixing SLAM bring-up with obstacle fusion.

### SLAM Strategy

Use `hku-mars/FAST_LIO` on its `ROS2` branch, not a forked pseudo-ROS2 port. The official `ROS2` branch uses `ament_cmake`, depends on `livox_ros_driver2`, ships `config/mid360.yaml`, and provides `mapping.launch.py`.

### Planning Strategy

Use `ZJU-FAST-Lab/ego-planner-swarm` on `ros2_version`, but start in simulator mode only. The baseline milestone verifies that the planner stack builds and launches with its own simulator before any attempt is made to feed real FAST-LIO odometry or real sensor obstacles into it.

### Follow-On Work After Baseline

After the baseline bring-up succeeds, the next subproject should add a local overlay package that:

- adapts FAST-LIO outputs to the topics and frames EGO-Swarm expects
- selects obstacle inputs from MID360, D435i, or both
- packages real-hardware launch files without modifying upstream repos

## Success Criteria

- `livox_ros_driver2` builds and launches against MID360 in `livox_ws`
- `fast_lio` builds in `slam_ws` and launches with `config_file:=mid360.yaml`
- `ego_planner` builds in `slam_ws` and launches `single_run_in_sim.launch.py`
- D435i ROS2 packages install cleanly from apt
- all upstream revisions are pinned in tracked manifest files

## Out of Scope

- PX4 bridge and closed-loop flight
- LiDAR-camera extrinsic calibration
- FAST-LIO to EGO-Swarm topic bridge implementation
- D435i obstacle fusion into planner inputs
- upstream source patching beyond emergency fixes
