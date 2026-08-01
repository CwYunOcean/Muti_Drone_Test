# Repository Guidelines

## Project Structure & Module Organization
Treat this repository as a ROS 2 flight-stack repository. Put project-owned code in `overlay_ws/src/`; local packages there include bringup wrappers and PX4 bridge adapters, each with package-local `test/` directories. `nav_ws/src/ego-swarm-ros2` and `slam_ws/src/` are vendored, upstream-heavy shared baselines: prefer configuration, wrappers, or adapters over direct edits, and never put per-aircraft settings there. Treat `isacc_ws/` and `livox_ws/` as external workspaces. Keep external source pins in `manifests/`, operational notes in `docs/runbooks/`, design docs in `docs/superpowers/`, and reusable launch helpers in `scripts/`. If an external upstream patch is unavoidable, store it under `patches/` with the target upstream SHA.

## Build, Test, and Development Commands
After cloning, `nav_ws/src` and `slam_ws/src` already contain the shared source baseline. Bootstrap external Livox sources with `vcs import livox_ws/src < manifests/livox_ws.repos`. For workspace dependencies, run `rosdep install --from-paths src --ignore-src -r -y` inside the workspace you are building. Typical local build:

```bash
cd overlay_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select position_cmd_to_px4_bridge
```

Run targeted tests with:

```bash
colcon test --packages-select position_cmd_to_px4_bridge
colcon test-result --verbose
```

Use repo scripts for smoke tests, for example `./scripts/run_mid360_lio.sh` or `./scripts/run_ego_single_real.sh`.

## Coding Style & Naming Conventions
Follow the surrounding ROS 2 package style: 4-space indentation for C++, Python, CMake, and launch files. Use `snake_case` for files, ROS parameters, and test names; use `PascalCase` for C++ types and keep namespaces lowercase. Local CMake packages already build with `-Wall -Wextra -Wpedantic`; new code should stay warning-clean. There is no repo-wide formatter config, so match the existing file style instead of reformatting unrelated code.

## Testing Guidelines
Overlay packages use `ament_add_gtest()` for C++ logic and `ament_add_pytest_test()` for launch, config, and contract checks. Add tests under `<package>/test/` and name them by behavior, for example `test_bridge_state_machine.cpp` or `test_mid360_lio_launch_contract.py`. No fixed coverage percentage is tracked, but every new node, launch file, or config contract should ship with a focused test.

## Commit & Pull Request Guidelines
Recent history follows Conventional Commit prefixes: `feat:`, `fix:`, `test:`, `docs:`, and `chore:`. Keep subjects imperative and specific, for example `fix: align PX4 bridge topics and status qos`. PRs should state which workspace or package changed, list the exact `colcon` or smoke-test commands you ran, and note any hardware assumptions. Include an RViz screenshot or launch log excerpt when changing bringup or topic wiring.

## Configuration & Data Hygiene
Do not commit build artifacts, rosbags, map dumps, or sensor recordings. Keep machine-specific edits such as lidar IPs local unless the change belongs in a tracked runbook or config template.
