#!/usr/bin/env bash
# Create a self-contained GVF+ISMC flight stack archive
# Includes: source code, scripts, experiment data, build instructions
# Excludes: build artifacts, .git, unused packages

set -euo pipefail

REPO_ROOT="/home/morphing01/Drone_SLAM"
OUTPUT_DIR="/tmp"
ARCHIVE_NAME="gvf_ismc_flight_stack_0509"

WORK_DIR="$OUTPUT_DIR/$ARCHIVE_NAME"
rm -rf "$WORK_DIR"
mkdir -p "$WORK_DIR"

echo "=== Copying overlay packages ==="
# Only the 9 packages used by the GVF flight stack
OVERLAY_PKGS=(
  gvf_ismc_path_following
  gvf_ismc_real_bringup
  gvf_path_following_msgs
  position_cmd_to_px4_bridge
  fastlio2_to_ego_swarm_leveling
  fastlio2_to_px4_odometry
  fast_lio2_mid360_bringup
  px4_msgs
  quadrotor_msgs
)

mkdir -p "$WORK_DIR/overlay_ws/src"
for pkg in "${OVERLAY_PKGS[@]}"; do
  src="$REPO_ROOT/overlay_ws/src/$pkg"
  if [ -d "$src" ]; then
    echo "  $pkg"
    cp -r "$src" "$WORK_DIR/overlay_ws/src/"
    # Remove build artifacts from copied dirs
    find "$WORK_DIR/overlay_ws/src/$pkg" -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true
    find "$WORK_DIR/overlay_ws/src/$pkg" -name "*.pyc" -delete 2>/dev/null || true
  fi
done

echo "=== Copying upstream SLAM dependencies ==="
mkdir -p "$WORK_DIR/slam_ws/src"
for pkg in FAST_LIO livox_ros_driver2 Livox-SDK2; do
  src="$REPO_ROOT/slam_ws/src/$pkg"
  if [ -d "$src" ]; then
    echo "  $pkg"
    # Use rsync to exclude .git and build artifacts
    rsync -a --exclude='.git' --exclude='build/' --exclude='install/' --exclude='log/' "$src/" "$WORK_DIR/slam_ws/src/$pkg/"
  fi
done

# Add slam_ws README
if [ -f "$REPO_ROOT/slam_ws/src/README.md" ]; then
  cp "$REPO_ROOT/slam_ws/src/README.md" "$WORK_DIR/slam_ws/src/"
fi

echo "=== Copying scripts ==="
mkdir -p "$WORK_DIR/scripts"
for f in run_mid360_fastlio2.sh run_gvf_ismc_single_real.sh micro_dds.sh connecttoqgc.sh record_gvf_ismc_experiment.sh replay_gvf_ismc_bag.sh gvf_ismc_reference.rviz gvf_ismc_bag_replay.rviz; do
  src="$REPO_ROOT/scripts/$f"
  if [ -f "$src" ]; then
    cp "$src" "$WORK_DIR/scripts/"
  fi
done
cp "$REPO_ROOT/scripts/parse_0509_bags.py" "$WORK_DIR/scripts/"

echo "=== Copying MATLAB reference ==="
mkdir -p "$WORK_DIR/GVF_ws"
cp "$REPO_ROOT/GVF_ws/DDSMC_three_leaf.m" "$WORK_DIR/GVF_ws/"

echo "=== Copying 0509 experiment data ==="
mkdir -p "$WORK_DIR/experiment_data"
for bag in "$REPO_ROOT/bags"/gvf_ismc_20260509_*/; do
  bag_name=$(basename "$bag")
  mkdir -p "$WORK_DIR/experiment_data/$bag_name"
  # Copy CSV files only (not db3 rosbags)
  for csv in "$bag"/*.csv; do
    if [ -f "$csv" ]; then
      cp "$csv" "$WORK_DIR/experiment_data/$bag_name/"
    fi
  done
  # Copy metadata for reference
  if [ -f "$bag/metadata.yaml" ]; then
    cp "$bag/metadata.yaml" "$WORK_DIR/experiment_data/$bag_name/"
  fi
done

echo "=== Copying top-level files ==="
cp "$REPO_ROOT/CLAUDE.md" "$WORK_DIR/"
cp "$REPO_ROOT/.gitignore" "$WORK_DIR/"

echo "=== Writing startup guide ==="
cat > "$WORK_DIR/README_STARTUP.md" << 'GUIDE_EOF'
# GVF + ISMC 飞行栈启动指南

## 1. 环境要求

- **硬件**: Jetson ARM64 + Livox MID360 LiDAR + PX4 飞控（串口 /dev/ttyUSB0）
- **系统**: Ubuntu 24.04 + ROS2 Jazzy
- **依赖**: `build-essential cmake git python3-vcstool python3-colcon-common-extensions python3-rosdep libeigen3-dev libpcl-dev libyaml-cpp-dev`

```bash
sudo apt update
sudo apt install -y build-essential cmake git python3-vcstool python3-colcon-common-extensions python3-rosdep libeigen3-dev libpcl-dev libyaml-cpp-dev
sudo rosdep init 2>/dev/null; rosdep update
```

## 2. 编译

### 2.1 编译 Livox SDK2

```bash
cd ~/Drone_SLAM/slam_ws/src/Livox-SDK2
mkdir -p build && cd build
cmake .. && make -j$(nproc)
sudo make install
```

### 2.2 编译 livox_ros_driver2

```bash
cd ~/Drone_SLAM/slam_ws/src/livox_ros_driver2
source /opt/ros/jazzy/setup.bash
./build.sh jazzy
```

### 2.3 编译 FAST-LIO2

```bash
cd ~/Drone_SLAM/slam_ws
source /opt/ros/jazzy/setup.bash
source ~/Drone_SLAM/slam_ws/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to fast_lio
```

### 2.4 编译 Overlay 包（GVF + ISMC + 桥接）

```bash
cd ~/Drone_SLAM/overlay_ws
source /opt/ros/jazzy/setup.bash
source ~/Drone_SLAM/slam_ws/install/setup.bash
colcon build --symlink-install
```

## 3. 启动（3 个终端）

所有终端统一设置：
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### 终端 A — XRCE Agent（飞控串口桥接）

```bash
sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 2000000
```

### 终端 B — FAST-LIO2（SLAM）

```bash
source /opt/ros/jazzy/setup.bash
source ~/Drone_SLAM/slam_ws/install/setup.bash
source ~/Drone_SLAM/overlay_ws/install/setup.bash

ros2 launch fast_lio2_mid360_bringup mid360_fastlio2.launch.py \
  livox_config_file:=~/Drone_SLAM/overlay_ws/src/fast_lio2_mid360_bringup/config/MID360_config_drone_1.json \
  drone_id:=1
```

### 终端 C — GVF + ISMC + PX4 Bridge

```bash
source /opt/ros/jazzy/setup.bash
source ~/Drone_SLAM/overlay_ws/install/setup.bash

ros2 launch gvf_ismc_real_bringup single_real.launch.py drone_id:=1 target_system:=2
```

## 4. 参数调整

配置文件: `overlay_ws/src/gvf_ismc_path_following/config/gvf_ismc_path_following.yaml`

| 参数 | 作用 | 默认值 |
|------|------|--------|
| `trajectory_type` | 轨迹类型: `"circular"` 或 `"three_leaf"` | `"circular"` |
| `radius_m` | 圆形半径 (m) | 3.5 |
| `target_height_m` | 目标高度 (m) | 1.2 |
| `base_radius_m` | 三叶草基础半径 (m) | 2.0 |
| `lobe_amplitude_m` | 三叶草叶片振幅 (m) | 0.6 |
| `lateral_gain` | 横向修正增益 | 1.0 |
| `vertical_gain` | 垂直修正增益 | 0.4 |
| `max_speed_mps` | 最大速度 (m/s) | 0.55 |
| `lambda` | 滑模面斜率 (ISMC) | 1.6 |
| `k` | 滑模面增益 (ISMC) | 0.8 |
| `c1` | 误差导数阻尼 (ISMC) | 0.05 |
| `epsilon` | tanh 趋近律厚度 (ISMC) | 0.01 |

Launch 时临时覆盖参数：
```bash
ros2 launch gvf_ismc_real_bringup single_real.launch.py trajectory_type:=three_leaf radius_m:=4.0
```

## 5. 关键话题监控

```bash
ros2 topic hz /drone_1/aft_mapped_to_init_level   # SLAM 里程计 (~10 Hz)
ros2 topic hz /gvf/reference               # GVF 参考速度
ros2 topic hz /drone_1_planning/pos_cmd    # ISMC 输出 (50 Hz)
ros2 topic hz /drone_1/fmu/in/trajectory_setpoint   # PX4 指令
```

## 6. 录包与回放

```bash
# 录包
./scripts/record_gvf_ismc_experiment.sh

# 回放
./scripts/replay_gvf_ismc_bag.sh <bag_path>

# 解析 CSV
python3 scripts/parse_0509_bags.py
```

## 7. 数据说明

`experiment_data/` 目录包含 2026-05-09 实验的 CSV 数据：

| 文件 | 内容 |
|------|------|
| `*_odom_level.csv` | SLAM 里程计 (位置/速度/姿态) |
| `*_gvf_reference.csv` | GVF 期望速度, phi1(高度误差), phi2(径向误差) |
| `*_pos_cmd.csv` | ISMC 输出指令 (速度/加速度/yaw) |
| `*_px4_setpoint.csv` | PX4 轨迹设定值 |
| `*_px4_odometry.csv` | PX4 飞控里程计 |

跟踪误差直接用 `phi1`(高度误差) 和 `phi2`(径向误差)。

## 8. 数据流架构

```
MID360 → livox_ros_driver2 → FAST-LIO2 (/drone_1/aft_mapped_to_init)
  → fastlio2_to_ego_swarm_leveling (/drone_1/aft_mapped_to_init_level)
  → gvf_reference_node (/gvf/reference)
  → ismc_velocity_tracker_node (/drone_1_planning/pos_cmd)
  → position_cmd_to_px4_bridge → /drone_1/fmu/in/*
```

- XY 轴: ISMC 外环跟踪 GVF 期望速度（tanh 趋近律 + C1 阻尼）
- Z 轴: GVF 比例修正直接发速度，PX4 内部速度控制器跟踪
GUIDE_EOF

echo "=== Creating archive ==="
cd "$OUTPUT_DIR"
tar czf "$ARCHIVE_NAME.tar.gz" "$ARCHIVE_NAME"
SIZE=$(du -sh "$ARCHIVE_NAME.tar.gz" | cut -f1)
echo ""
echo "Done: $OUTPUT_DIR/$ARCHIVE_NAME.tar.gz ($SIZE)"
