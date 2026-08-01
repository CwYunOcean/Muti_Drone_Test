# FAST-LIO2 + EGO-Swarm Baseline Bring-Up Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a reproducible baseline on Jetson/ROS2 Humble where MID360, FAST-LIO2, D435i, and EGO-Swarm each build and launch with minimal upstream modification.

**Architecture:** Keep Livox in a dedicated workspace because its build script wipes parent build products. Build FAST-LIO and EGO-Swarm in a second workspace that sources the Livox workspace as an underlay. Reserve project-specific code for a future overlay workspace instead of editing upstream repositories.

**Tech Stack:** Ubuntu 22.04, ROS2 Humble, Livox-SDK2, livox_ros_driver2, FAST_LIO `ROS2` branch, ego-planner-swarm `ros2_version`, vcstool, colcon

---

### Task 1: Verify and Import Locked Upstream Sources

**Files:**
- Modify: `manifests/livox_ws.repos`
- Modify: `manifests/slam_ws.repos`
- Reference: `docs/runbooks/manual-bootstrap.md`

- [ ] **Step 1: Verify the pinned revisions still exist upstream**

Run:

```bash
cd ~/Drone_SLAM
git ls-remote --heads https://github.com/Livox-SDK/Livox-SDK2.git
git ls-remote --heads https://github.com/Livox-SDK/livox_ros_driver2.git
git ls-remote --heads https://github.com/hku-mars/FAST_LIO.git
git ls-remote --heads https://github.com/ZJU-FAST-Lab/ego-planner-swarm.git
```

Expected:

- `FAST_LIO` still exposes `refs/heads/ROS2`
- `ego-planner-swarm` still exposes `refs/heads/ros2_version`

- [ ] **Step 2: Import the Livox workspace sources**

Run:

```bash
cd ~/Drone_SLAM
mkdir -p livox_ws/src
vcs import livox_ws/src < manifests/livox_ws.repos
```

Expected:

- `livox_ws/src/Livox-SDK2` exists
- `livox_ws/src/livox_ros_driver2` exists

- [ ] **Step 3: Import the SLAM workspace sources**

Run:

```bash
cd ~/Drone_SLAM
mkdir -p slam_ws/src
vcs import slam_ws/src < manifests/slam_ws.repos
git -C slam_ws/src/FAST_LIO submodule update --init --recursive
```

Expected:

- `slam_ws/src/FAST_LIO` exists
- `slam_ws/src/ego-planner-swarm` exists
- FAST-LIO submodules are initialized

- [ ] **Step 4: Commit manifest updates if any revision changed**

Run:

```bash
cd ~/Drone_SLAM
git add manifests/livox_ws.repos manifests/slam_ws.repos docs/runbooks/manual-bootstrap.md
git commit -m "docs: refresh upstream source locks"
```

Expected:

- Either a new commit exists, or there is nothing to commit because pins did not change

### Task 2: Bring Up the Dedicated Livox Workspace

**Files:**
- Reference: `docs/runbooks/manual-bootstrap.md`
- Runtime edit: `livox_ws/src/livox_ros_driver2/config/MID360_config.json`

- [ ] **Step 1: Build and install Livox-SDK2**

Run:

```bash
cd ~/Drone_SLAM/livox_ws/src/Livox-SDK2
mkdir -p build
cd build
cmake ..
make -j"$(nproc)"
sudo make install
```

Expected:

- `make` completes without compile errors
- `sudo make install` succeeds

- [ ] **Step 2: Build livox_ros_driver2 in the isolated workspace**

Run:

```bash
cd ~/Drone_SLAM/livox_ws/src/livox_ros_driver2
source /opt/ros/humble/setup.bash
./build.sh humble
```

Expected:

- `~/Drone_SLAM/livox_ws/install/setup.bash` exists
- no unrelated workspace packages are wiped because the workspace is isolated

- [ ] **Step 3: Set the MID360 host and lidar IPs**

Edit:

```json
{
  "MID360": {
    "host_net_info": {
      "cmd_data_ip": "<host_nic_ip>",
      "push_msg_ip": "<host_nic_ip>",
      "point_data_ip": "<host_nic_ip>",
      "imu_data_ip": "<host_nic_ip>"
    }
  },
  "lidar_configs": [
    {
      "ip": "<mid360_ip>"
    }
  ]
}
```

File:

```text
~/Drone_SLAM/livox_ws/src/livox_ros_driver2/config/MID360_config.json
```

- [ ] **Step 4: Smoke-test the MID360 driver**

Run:

```bash
source /opt/ros/humble/setup.bash
source ~/Drone_SLAM/livox_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

Expected:

- the launch starts without package lookup errors
- `ros2 topic list` in a second shell shows Livox driver topics

- [ ] **Step 5: Commit only tracked docs if you updated instructions**

Run:

```bash
cd ~/Drone_SLAM
git add docs/runbooks/manual-bootstrap.md docs/superpowers/specs/2026-04-03-fastlio2-egoswarm-baseline-design.md
git commit -m "docs: record livox workspace bring-up"
```

Expected:

- Only tracked repo files are committed
- No upstream clone content is staged

### Task 3: Build and Smoke-Test FAST-LIO2

**Files:**
- Reference: `docs/runbooks/manual-bootstrap.md`
- Upstream runtime config: `slam_ws/src/FAST_LIO/config/mid360.yaml`

- [ ] **Step 1: Install missing ROS dependencies**

Run:

```bash
cd ~/Drone_SLAM/slam_ws
source /opt/ros/humble/setup.bash
source ~/Drone_SLAM/livox_ws/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Expected:

- rosdep resolves system dependencies for both FAST-LIO and EGO-Swarm packages

- [ ] **Step 2: Build only the FAST-LIO package first**

Run:

```bash
cd ~/Drone_SLAM/slam_ws
source /opt/ros/humble/setup.bash
source ~/Drone_SLAM/livox_ws/install/setup.bash
colcon build --symlink-install --packages-up-to fast_lio
```

Expected:

- `slam_ws/install/setup.bash` exists
- package `fast_lio` is discoverable by ROS2

- [ ] **Step 3: Launch FAST-LIO with the MID360 config**

Run:

```bash
source /opt/ros/humble/setup.bash
source ~/Drone_SLAM/livox_ws/install/setup.bash
source ~/Drone_SLAM/slam_ws/install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

Expected:

- `fastlio_mapping` starts
- RViz starts unless disabled
- the node resolves `livox_ros_driver2` message types successfully

- [ ] **Step 4: Record any upstream config mismatches without patching yet**

Run:

```bash
cd ~/Drone_SLAM
mkdir -p docs/run-results
printf '%s\n' \
  '- FAST-LIO build result:' \
  '- MID360 topic names observed:' \
  '- Frame IDs observed:' \
  '- Any launch/runtime errors:' \
  > docs/run-results/fastlio-smoke-test.md
git add docs/run-results/fastlio-smoke-test.md
git commit -m "docs: record fast-lio smoke test findings"
```

Expected:

- runtime mismatches are captured as notes instead of ad-hoc upstream edits

### Task 4: Build and Smoke-Test EGO-Swarm in Simulation

**Files:**
- Reference: `docs/runbooks/manual-bootstrap.md`
- Upstream launch path: `slam_ws/src/ego-planner-swarm/src/planner/plan_manage/launch/single_run_in_sim.launch.py`

- [ ] **Step 1: Build the remaining packages in the SLAM workspace**

Run:

```bash
cd ~/Drone_SLAM/slam_ws
source /opt/ros/humble/setup.bash
source ~/Drone_SLAM/livox_ws/install/setup.bash
colcon build --symlink-install
```

Expected:

- package `ego_planner` is built
- simulator packages are built

- [ ] **Step 2: Smoke-test EGO-Swarm in simulator mode**

Run:

```bash
source /opt/ros/humble/setup.bash
source ~/Drone_SLAM/slam_ws/install/setup.bash
ros2 launch ego_planner single_run_in_sim.launch.py
```

Expected:

- simulator, planner, and RViz nodes start
- no package resolution errors for `ego_planner`

- [ ] **Step 3: Capture planner-side interface expectations**

Run:

```bash
cd ~/Drone_SLAM
printf '%s\n' \
  '- Planner odometry topics:' \
  '- Planner obstacle/map topics:' \
  '- Planner frame assumptions:' \
  '- Launch arguments worth overriding later:' \
  > docs/run-results/ego-planner-smoke-test.md
git add docs/run-results/ego-planner-smoke-test.md
git commit -m "docs: record ego planner smoke test findings"
```

Expected:

- the next integration step has explicit planner I/O notes

### Task 5: Finish the Baseline and Prepare the Next Subproject

**Files:**
- Create: `docs/superpowers/specs/2026-04-03-fastlio-to-egoplanner-bridge-design.md`
- Test: `docs/run-results/fastlio-smoke-test.md`
- Test: `docs/run-results/ego-planner-smoke-test.md`

- [ ] **Step 1: Verify the D435i binary package path still works**

Run:

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true enable_gyro:=true enable_accel:=true
```

Expected:

- the RealSense node starts
- `ros2 topic list | grep camera` shows camera topics

- [ ] **Step 2: Write the next bridge subproject spec**

Write:

```markdown
# FAST-LIO to EGO-Swarm Bridge Design

Goal: add a local overlay package in `overlay_ws/src` that maps FAST-LIO odometry and obstacle data to the topics, frames, and launch contracts that `ego_planner` expects.

Inputs:
- `docs/run-results/fastlio-smoke-test.md`
- `docs/run-results/ego-planner-smoke-test.md`

Outputs:
- one overlay package
- zero direct upstream edits unless a patch file is unavoidable
```

File:

```text
docs/superpowers/specs/2026-04-03-fastlio-to-egoplanner-bridge-design.md
```

- [ ] **Step 3: Commit the baseline handoff**

Run:

```bash
cd ~/Drone_SLAM
git add docs/superpowers/specs/2026-04-03-fastlio-to-egoplanner-bridge-design.md
git commit -m "docs: hand off fast-lio to ego planner bridge scope"
```

Expected:

- baseline bring-up is closed out cleanly
- the next subproject starts from measured interfaces, not guesses
