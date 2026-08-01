# MID360 LIO Launch Script Design

**Date:** 2026-04-08

## Goal

Add a simple project-local shell script that starts the current `MID360 + FAST-LIVO2` pure `LIO` stack and opens RViz with the upstream `FAST-LIVO2` configuration.

## Scope

This change adds one operator entrypoint only:

- `scripts/run_mid360_lio.sh`

The script should:

1. source ROS2 Humble
2. source `slam_ws/install/setup.bash`
3. build `overlay_ws` for `fast_livo2_mid360_bringup`
4. source `overlay_ws/install/setup.bash`
5. launch `fast_livo2_mid360_bringup mid360_lio.launch.py`
6. open RViz with the official `FAST-LIVO2` `fast_livo2.rviz`
7. stop the background launch when the script exits

## Constraints

- Keep the script short and readable.
- Do not add `tmux`, option parsing, or multiple helper scripts.
- Prefer a few top-level variables over hardcoded repeated paths.
- Use the official upstream RViz config instead of a project-owned copy.
- Do not modify upstream `FAST-LIVO2` or `livox_ros_driver2`.

## Recommended Design

Create `scripts/run_mid360_lio.sh` as a single Bash entrypoint with:

- `set -euo pipefail`
- a small path block:
  - `BASE_REPO_ROOT`
  - `OVERLAY_WS`
  - `LIVOX_CONFIG_FILE`
  - `RVIZ_CONFIG_FILE`
- one background `ros2 launch ...` process
- one `trap` that kills the background launch on exit

The script should keep RViz in the foreground so the operator sees logs in one terminal and can stop everything with `Ctrl+C`.

## Expected Outcome

The operator should be able to start the validated pure-LIO stack with one command:

```bash
bash scripts/run_mid360_lio.sh
```

The expected visible result is:

- the Livox driver and `fastlivo_mapping` start
- RViz opens with the official `fast_livo2.rviz`
- closing the script also stops the background launch process
