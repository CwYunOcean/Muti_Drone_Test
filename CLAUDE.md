# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Integration repo for a Jetson-based UAV running ROS2 Humble on Ubuntu 22.04 ARM64. The stack centers on a Livox MID360 LiDAR, FAST-LIO2 SLAM, Intel D435i camera, EGO-Swarm planner, GVF/ISMC path following controller, and PX4 autopilot — all communicating via Fast-DDS with XRCE-DDS bridging to the flight controller.

## Workspace Layout

| Workspace | Purpose | Edit Policy |
|-----------|---------|-------------|
| `overlay_ws/src/` | Local packages, adapters, bringup wrappers, configs | **Edit freely** — this is project-owned code |
| `slam_ws/src/` | Upstream FAST-LIO2, EGO-Swarm, Livox driver | Read-only; patches go in `patches/` |
| `nav_ws/src/ego-swarm-ros2/` | Vendored EGO-Swarm navigation baseline | Shared source; changes require review and testing |
| `livox_ws/src/` | Livox SDK2 + livox_ros_driver2 | Read-only (uses its own `build.sh`) |
| `isacc_ws/src/` | Isaac ROS packages | Read-only |
| `GVF_ws/` | MATLAB DDSMC reference scripts | Reference only |

`manifests/` holds pinned external upstream `.repos` files (bootstrap with `vcs import slam_ws/src < manifests/slam_ws.repos`). `nav_ws/src/ego-swarm-ros2` is already present after a root clone; its provenance and GPLv3 license obligations are recorded in `nav_ws/UPSTREAM.md`. Do not commit build artifacts, rosbags, map dumps, or sensor recordings. Keep machine-specific edits (e.g., lidar IPs) local unless they belong in a tracked runbook or config template.

## Docs & Skills

- `docs/runbooks/` — hardware bring-up procedures and flight checklists (e.g., `gvf-ismc-single-real.md`, `manual-bootstrap.md`)
- `docs/superpowers/specs/` and `docs/superpowers/plans/` — design docs and executable implementation plans
- `.claude/skills/` — project skills: `bringup-flight-stack`, `building-testing-overlay`, `recording-replaying-bags`; invoke the matching skill before launching stacks, building/testing with colcon, or handling experiment bags

## Git Workflow

- `main` holds docs, manifests, vendored navigation source, and reviewed overlay code; use project-local `.worktrees/<branch>` for feature branches
- Keep external upstream workspaces pristine — prefer overlays/adapters; unavoidable upstream edits go in `patches/` with the upstream commit SHA they apply to
- Changes below `nav_ws/src/ego-swarm-ros2` are shared source changes. Keep the upstream `LICENSE`, state material deviations in the commit, and rebuild navigation-dependent packages before merge.
- PRs should state which workspace/package changed, list the exact colcon or smoke-test commands run, and note hardware assumptions

## Build Commands

All builds use `colcon` with `--symlink-install`. Source ROS and workspace setups before building:

```bash
source /opt/ros/humble/setup.bash
source <workspace>/install/setup.bash
```

Build a single overlay package:
```bash
cd overlay_ws
colcon build --symlink-install --packages-select <package_name>
```

Build with upstream dependencies (e.g., FAST-LIO2 needs slam_ws sourced first):
```bash
cd slam_ws
source /opt/ros/humble/setup.bash
source ~/Drone_SLAM/livox_ws/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to fast_lio
```

Install deps for any workspace:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Test Commands

Run all tests for a package:
```bash
cd overlay_ws
colcon test --packages-select <package_name>
colcon test-result --verbose
```

Run a single gtest binary directly:
```bash
cd overlay_ws
./build/<package_name>/test_<test_name>
```

Run a single pytest test:
```bash
cd overlay_ws
python3 -m pytest src/<package_name>/test/test_<name>.py
```

**Test conventions**: C++ logic uses `ament_add_gtest()`, Python launch/config contract checks use `ament_add_pytest_test()`. Tests live in `<package>/test/` and are named by behavior (e.g., `test_bridge_state_machine.cpp`, `test_config_contract.py`). Every new node, launch file, or config contract should ship with a focused test.

## Data Flow Architecture

Two parallel flight stacks share the same SLAM front-end and PX4 bridge:

**EGO-Swarm stack:**
```
MID360 → livox_ros_driver2 → FAST-LIO2 (/aft_mapped_to_init)
  → fastlio2_to_ego_swarm_leveling (/aft_mapped_to_init_level)
  → ego_planner (/drone_0_planning/pos_cmd)
  → position_cmd_to_px4_bridge → /fmu/in/*
```

**GVF+ISMC stack:**
```
MID360 → livox_ros_driver2 → FAST-LIO2 (/aft_mapped_to_init)
  → fastlio2_to_ego_swarm_leveling (/aft_mapped_to_init_level)
  → gvf_reference_node (/gvf/reference)
  → ismc_velocity_tracker_node (/drone_0_planning/pos_cmd)
  → position_cmd_to_px4_bridge → /fmu/in/*
```

Both stacks converge on `/drone_0_planning/pos_cmd` which feeds into the PX4 bridge. The leveling node (`fastlio2_to_ego_swarm_leveling`) applies `level_rpy_rad` to align the SLAM frame to a horizontal plane — adjust there first if the plane definition is wrong, not in downstream nodes.

## Overlay Package Summary

| Package | Type | Key Nodes / Libraries | Purpose |
|---------|------|-----------------------|---------|
| `position_cmd_to_px4_bridge` | C++ lib+node | `bridge_state_machine`, `trajectory_setpoint_conversion`, `offboard_control_mode_builder` | Converts PositionCommand → PX4 offboard setpoints; state machine manages IDLE→WAIT→STREAM→OFFBOARD→ARM→ACTIVE→FAILSAFE |
| `gvf_ismc_path_following` | C++ lib+node | `three_leaf_gvf`, `ismc_outer_loop`, `gvf_reference_node`, `ismc_velocity_tracker_node` | Three-leaf guidance vector field + integral sliding mode controller for path following |
| `gvf_ismc_real_bringup` | Launch-only | `single_real.launch.py`, `replay_visualization.launch.py` | Launch composition for real-hardware GVF/ISMC |
| `gvf_path_following_msgs` | Msg-only | GVFReference, PositionCommand, SO3Command, etc. | Custom message interfaces shared between GVF nodes and bridge |
| `fastlio2_to_ego_swarm_leveling` | C++ lib+node | `leveled_frame_transform` | Rotates FAST-LIO2 odometry into a level world frame |
| `fastlio2_to_px4_odometry` | C++ lib+node | `odometry_conversion` | Feeds FAST-LIO2 odometry directly to PX4 |
| `fast_lio2_mid360_bringup` | Launch-only | `mid360_fastlio2.launch.py` | Launch wrapper for MID360 + FAST-LIO2 |
| `ego_swarm_real_bringup` | Launch-only | `single_real.launch.py` | Launch composition for single-drone EGO-Swarm |
| `fast_livo2_mid360_bringup` | Launch-only | — | LIVO variant with D435i |
| `fastlivo_to_px4_odometry` | C++ lib+node | — | LIVO odometry → PX4 |
| `px4_msgs` | Msg-only | — | PX4 micro-RTPS message definitions |
| `quadrotor_msgs` | Msg-only | — | Quadrotor command/status messages |

Common pattern: each C++ package splits logic into a `_core` or `_transform` shared library (testable without ROS) plus a thin ROS node executable that links it.

## Hardware Launch Scripts

Scripts in `scripts/` handle multi-terminal bringup with proper workspace sourcing and DDS config:

- `run_mid360_fastlio2.sh` — starts FAST-LIO2 stack (sources slam_ws + overlay_ws)
- `run_ego_single_real.sh` — starts EGO-Swarm bringup (sources nav_ws + overlay_ws)
- `run_gvf_ismc_single_real.sh` — starts GVF+ISMC bringup (overlay_ws only)
- `run_px4_position_mode.sh` / `start_position_mode.sh` — PX4 position-mode VIO flight
- `micro_dds.sh` — XRCE-DDS agent over serial (`/dev/ttyUSB0`)
- `connecttoqgc.sh` — MAVProxy bridge to QGroundControl
- `record_gvf_ismc_experiment.sh` / `replay_gvf_ismc_bag.sh` — bag record/replay

DDS environment must be consistent across all terminals:
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## Coding Conventions

- **Indent**: 4 spaces for C++, Python, CMake, and launch files
- **Files/params/tests**: `snake_case`; **C++ types**: `PascalCase`; **namespaces**: lowercase
- **Compiler flags**: all overlay packages build with `-Wall -Wextra -Wpedantic` — new code must stay warning-clean
- **No repo-wide formatter** — match the existing file style
- **Conventional commits**: `feat:`, `fix:`, `test:`, `docs:`, `chore:` prefixes with imperative subjects

## PX4 Bridge State Machine

The `position_cmd_to_px4_bridge` manages the offboard flight lifecycle:
`IDLE → WAIT_FASTLIVO → WAIT_PX4 → STREAM_SETPOINT → ENTER_OFFBOARD → ARM → ACTIVE`

Key config parameters in `position_cmd_to_px4_bridge.yaml`:
- `auto_request_offboard_and_arm` — automatic mode transition vs manual
- `command_timeout_ms` / `status_timeout_ms` / `odom_timeout_ms` — health timeouts
- `world_axis` / `world_sign` — coordinate frame mapping between SLAM and PX4 (NED)
- `use_acceleration_feedforward` — toggle velocity vs acceleration control
