# Drone_SLAM

Integration repository for a Jetson-based UAV stack centered on MID360,
FAST-LIO2, D435i, EGO-Swarm, and PX4 multi-vehicle formation control.

## Scope

This repository tracks:

- design docs, plans, runbooks, and upstream source pins
- local overlay code under `overlay_ws/src`
- local formation controller code under `uav_formation_ws/src/distribute_control`
- the vendored EGO-Swarm navigation source under `nav_ws/src/ego-swarm-ros2`
- the vendored FAST-LIO, FAST-LIVO, Livox, and calibration source baseline under `slam_ws/src`
- reproducible patches when upstream edits are unavoidable

This repository does not track:

- external upstream clones other than the vendored navigation and SLAM baselines
- ROS build artifacts
- rosbags, map dumps, or sensor recordings

## Workspace Layout

- `livox_ws/src/` - dedicated workspace for `livox_ros_driver2`
- `slam_ws/src/` - vendored SLAM, driver, and calibration source baseline
- `nav_ws/src/ego-swarm-ros2/` - vendored EGO-Swarm navigation source
- `overlay_ws/src/` - local packages, launch wrappers, adapters, and configs
- `uav_formation_ws/src/` - local formation controller and its tests
- `manifests/` - pinned external upstream source manifests
- `config/*.example` - templates copied to ignored, per-robot configuration files
- `patches/` - local patch files if upstream modification becomes unavoidable
- `docs/superpowers/specs/` - design docs
- `docs/superpowers/plans/` - executable implementation plans
- `docs/runbooks/` - machine bring-up notes and commands

## Git Workflow

- `main` holds docs, manifests, vendored navigation and SLAM baselines, and reviewed overlay code.
- Use project-local `.worktrees/<branch>` for feature branches.
- Keep external upstream sources pristine; prefer overlays or patch files over direct edits.
- Treat `nav_ws/src/ego-swarm-ros2` as shared flight source: changes require
  review and testing, never per-aircraft edits. See `nav_ws/UPSTREAM.md`.
- Treat `slam_ws/src` as shared flight source: changes require review and
  testing, never per-aircraft edits. See `slam_ws/UPSTREAM.md`.
- If an upstream edit is unavoidable, store the exact patch under `patches/` and record the upstream commit SHA it applies to.
- Do not change tracked launch/config files on an aircraft. Put robot identity,
  MID360 IPs, and world-frame calibration in ignored files below `config/`.

## Multi-Robot Deployment

The full GitHub initialization, clone/bootstrap procedure, local configuration
contract, PX4 board settings, and update workflow are documented in
[`docs/runbooks/github-multi-drone-repository.md`](docs/runbooks/github-multi-drone-repository.md).

## Bootstrap

Manual source import and baseline commands are documented in `docs/runbooks/manual-bootstrap.md`.
