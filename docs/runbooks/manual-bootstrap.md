# Manual Bootstrap

This runbook keeps upstream source changes to a minimum. Large network-heavy clone operations are listed for manual execution.

## Assumptions

- Host OS: Ubuntu 22.04
- ROS: Humble
- Hardware: Jetson-class ARM64 machine, MID360, D435i
- Current repo root: `~/Drone_SLAM`

## 1. Base Packages

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git python3-vcstool python3-colcon-common-extensions \
  python3-rosdep libeigen3-dev libpcl-dev libyaml-cpp-dev
```

If `rosdep` has not been initialized on this machine:

```bash
sudo rosdep init
rosdep update
```

## 2. D435i Packages

Use released ROS2 Humble packages first. This keeps the RealSense stack out of the source tree and avoids the broken legacy `apt-key` instructions.

```bash
sudo apt install -y \
  ros-humble-librealsense2 \
  ros-humble-realsense2-camera \
  ros-humble-realsense2-camera-msgs \
  ros-humble-realsense2-description
```

## 3. Import External Sources

```bash
mkdir -p ~/Drone_SLAM/livox_ws/src
cd ~/Drone_SLAM
vcs import livox_ws/src < manifests/livox_ws.repos
```

`slam_ws/src` and `nav_ws/src/ego-swarm-ros2` are shared source baselines
already present after cloning `Drone_SLAM`. Do not re-import or initialize Git
submodules inside those directories; use the versions tracked by `main`.

## 4. Build Livox-SDK2

```bash
cd ~/Drone_SLAM/livox_ws/src/Livox-SDK2
mkdir -p build
cd build
cmake ..
make -j"$(nproc)"
sudo make install
```

## 5. Build livox_ros_driver2 in Its Own Workspace

`livox_ros_driver2/build.sh` cleans the workspace before building, so keep it isolated in `livox_ws`.

```bash
cd ~/Drone_SLAM/livox_ws/src/livox_ros_driver2
source /opt/ros/humble/setup.bash
./build.sh humble
```

Before launching MID360, edit the host IP and lidar IP in:

- `~/Drone_SLAM/livox_ws/src/livox_ros_driver2/config/MID360_config.json`

Smoke test:

```bash
source /opt/ros/humble/setup.bash
source ~/Drone_SLAM/livox_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

Expected result:

- the node starts without config parsing errors
- `ros2 topic list` includes Livox custom message topics

## 6. Build FAST_LIO on the Official ROS2 Branch

The official `ROS2` branch already declares `ament_cmake`, depends on `livox_ros_driver2`, and ships `config/mid360.yaml`.

```bash
cd ~/Drone_SLAM/slam_ws
source /opt/ros/humble/setup.bash
source ~/Drone_SLAM/livox_ws/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to fast_lio
```

Smoke test:

```bash
source /opt/ros/humble/setup.bash
source ~/Drone_SLAM/livox_ws/install/setup.bash
source ~/Drone_SLAM/slam_ws/install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

Expected result:

- RViz and `fastlio_mapping` start
- the node subscribes to MID360 topics without package resolution errors

## 7. Build ego-planner-swarm ROS2 Branch

`ego-planner-swarm` ships ROS2 Python launch files under `src/planner/plan_manage/launch/`.

```bash
cd ~/Drone_SLAM/slam_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Smoke test in simulator mode:

```bash
source /opt/ros/humble/setup.bash
source ~/Drone_SLAM/slam_ws/install/setup.bash
ros2 launch ego_planner single_run_in_sim.launch.py
```

Expected result:

- simulator, planner, and RViz nodes start
- no package lookup error for `ego_planner`

## 8. Follow-On Work

Do not edit upstream repos yet.

The next local package should live in `overlay_ws/src/` and handle:

- FAST-LIO odometry to EGO-Swarm input topic adaptation
- local map / obstacle topic selection between MID360 and D435i
- deployment launch files for real hardware
