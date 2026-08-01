# EGO-Swarm 双机飞行准备记录

记录时间: 2026-07-28 17:13:14 CST +0800  
记录对象: 当前 `/home/aim05/Drone_SLAM` 发布目录, 本机按 drone1 使用  
本机身份: `drone_id=1`, PX4 `MAV_SYS_ID=2`, XRCE namespace `drone_1`

## 结论

当前 drone1 侧的 EGO/SLAM/PX4 bridge 话题命名、RViz 目标点、leveling、PX4 namespace 适配代码没有发现新的必须修改项。双机放飞前的主要工作不是继续改代码, 而是逐机确认硬件参数、DDS 互通、时间同步、共享世界系和地面拆桨联调结果。

需要保留的风险是: 这份当前目录默认值已经偏向 drone1。drone0 不能直接原样使用 drone1 的默认配置, 尤其不能沿用 `target_system: 2`、drone1 的 Livox IP 和 drone1 的 XRCE namespace。

## 当前配置基线

### ROS 和系统

- OS: Ubuntu 24.04.4 LTS
- ROS: Jazzy
- `ROS_DISTRO`: `jazzy`
- RMW: `rmw_fastrtps_cpp`
- `ros-jazzy-rmw-fastrtps-cpp`: `8.4.4-1noble.20260612.092346`

所有实机终端建议统一:

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
```

### 工作空间状态

本目录内检查结果:

```text
livox_ws   src=no install_setup=no
slam_ws    src=yes install_setup=yes
nav_ws     src=yes install_setup=yes
overlay_ws src=yes install_setup=yes
isacc_ws   src=no install_setup=no
```

Livox SDK2 与 livox_ros_driver2 当前在 `slam_ws` 内:

```text
slam_ws/src/Livox-SDK2
slam_ws/src/livox_ros_driver2
slam_ws/local/livox_sdk2/include/livox_lidar_api.h
slam_ws/local/livox_sdk2/lib/liblivox_lidar_sdk_shared.so
```

### drone1 Livox 配置

文件: `overlay_ws/src/fast_lio2_mid360_bringup/config/MID360_config_drone_1.json`

```text
host_ip: 192.168.1.50
lidar_ip: 192.168.1.108
cmd_data_port: 56100 -> 56101
point_data_port: 56300 -> 56301
imu_data_port: 56400 -> 56401
```

双机时两台 MID360 的 IP 必须不同, 两台机载电脑的 Livox 网卡 IP 也不能冲突。

### drone1 leveling 配置

文件: `overlay_ws/src/fastlio2_to_ego_swarm_leveling/config/fastlio2_to_ego_swarm_leveling.yaml`

```yaml
input_odom_topic: "/drone_1/aft_mapped_to_init"
input_cloud_topic: "/drone_1/cloud_registered"
output_odom_topic: "/drone_1/aft_mapped_to_init_level"
output_cloud_topic: "/drone_1/cloud_registered_level"
world_frame_id: "world"
output_frame_id: "camera_init_level"
level_rpy_rad: [0.0, 0.5235987755982988, 0.0]
world_origin_xyz: [0.0, 0.0, 0.0]
world_origin_yaw_rad: 0.0
```

`level_rpy_rad` 当前按 MID360 前倾 30 度处理。若某台机的雷达安装角不同, 只改该机 leveling 的 `level_rpy_rad`。

双机飞行前, drone1 的 `world_origin_xyz` 不能继续盲用 `[0.0, 0.0, 0.0]`, 必须按 drone1 起飞点相对 drone0 起飞点的实际位置填写。坐标约定是 drone0 共享世界系: x 前, y 左, z 上。例如 drone1 在 drone0 右侧 3 m:

```yaml
world_origin_xyz: [0.0, -3.0, 0.0]
world_origin_yaw_rad: 0.0
```

### drone1 EGO 配置

文件: `overlay_ws/src/ego_swarm_real_bringup/config/ego_planner_real.yaml`

关键值:

```yaml
fsm/flight_type: 1
fsm/waypoint_num: 1
fsm/waypoint0_z: 1.0
manager/max_vel: 1.0
manager/max_acc: 1.0
optimization/dist0: 0.6
optimization/swarm_clearance: 1.2
bspline/limit_vel: 1.0
bspline/limit_acc: 1.0
manager/drone_id: 1
```

代码中 RViz `/move_base_simple/goal` 回调使用点击的 `x/y`, 高度固定为 `z=1.0`。因此 RViz 目标点当前应按 1 m 高度理解。

### drone1 PX4 bridge 配置

文件: `overlay_ws/src/position_cmd_to_px4_bridge/config/position_cmd_to_px4_bridge.yaml`

```yaml
command_topic: "/drone_1_planning/pos_cmd"
offboard_mode_topic: "/drone_1/fmu/in/offboard_control_mode"
trajectory_setpoint_topic: "/drone_1/fmu/in/trajectory_setpoint"
vehicle_command_topic: "/drone_1/fmu/in/vehicle_command"
vehicle_odometry_topic: "/drone_1/fmu/out/vehicle_odometry"
vehicle_status_topic: ""
fastlivo_odom_topic: "/drone_1/aft_mapped_to_init_level"
auto_request_offboard_and_arm: false
target_system: 2
```

`auto_request_offboard_and_arm: false` 表示 bridge 不会主动发送 arm 或切 Offboard 命令。但 EGO 产生 `/drone_1_planning/pos_cmd` 后, bridge 仍会发布 offboard control mode 和 trajectory setpoint。是否进入 Offboard/arm 仍应由遥控器或 QGC 人工控制。

## 双机飞前必须确认

### 1. 机号和 PX4 命名空间

drone0:

```text
drone_id=0
MAV_SYS_ID=1
XRCE namespace=drone_0
bridge target_system=1
```

drone1:

```text
drone_id=1
MAV_SYS_ID=2
XRCE namespace=drone_1
bridge target_system=2
```

PX4 侧 `uxrce_dds_client` 必须分别带 `-n drone_0` 和 `-n drone_1`。本机 agent 不应修改 PX4 参数或 SD 卡启动脚本, 除非飞控负责人明确批准。

### 2. 网络和 DDS

两台机载电脑和笔记本必须在同一 WiFi/LAN, 且 ROS 2 domain 一致。检查:

```bash
ros2 topic list | grep -E 'drone_0|drone_1|broadcast_bspline'
```

如果看不到对方 topic, 优先检查:

- `ROS_DOMAIN_ID` 是否一致
- `RMW_IMPLEMENTATION` 是否都是 `rmw_fastrtps_cpp`
- `ROS_LOCALHOST_ONLY` 是否为 `0`
- VPN、防火墙、路由器 AP 隔离是否阻断 multicast

### 3. 时间同步

EGO-Swarm 对 `/broadcast_bspline` 的时间差有硬门限, 时差过大时会丢弃对方轨迹。目标偏差 <10 ms。

```bash
# drone0
sudo ./scripts/setup_chrony_time_sync.sh server

# drone1
sudo ./scripts/setup_chrony_time_sync.sh client <drone0_wifi_ip>

# 两台都检查
chronyc tracking
chronyc sources -v
```

单机测试时可以暂缓 chrony, 双机飞行前不能跳过。

### 4. 共享世界系

起飞前摆好两台飞机, 机头尽量同向, 量出 drone1 相对 drone0 起飞点的位置, 写入 drone1 的 `world_origin_xyz`。然后在 RViz 中确认:

```bash
ros2 topic hz /drone_0/aft_mapped_to_init_level
ros2 topic hz /drone_1/aft_mapped_to_init_level
```

RViz Fixed Frame 使用 `camera_init_level`, 视觉上两台飞机相对位置必须和实际摆放一致。若不一致, 不进入 Offboard。

### 5. PX4 和 SLAM 话题

两台地面通电、不解锁时检查:

```bash
ros2 topic hz /drone_0/fmu/out/vehicle_odometry
ros2 topic hz /drone_1/fmu/out/vehicle_odometry
ros2 topic hz /drone_0/aft_mapped_to_init
ros2 topic hz /drone_1/aft_mapped_to_init
ros2 topic hz /drone_0/aft_mapped_to_init_level
ros2 topic hz /drone_1/aft_mapped_to_init_level
```

期望:

- `/drone_N/fmu/out/vehicle_odometry` 有稳定输出, 表示 XRCE namespace 正确
- `/drone_N/aft_mapped_to_init` 有稳定输出, 表示 FAST-LIO2 正常
- `/drone_N/aft_mapped_to_init_level` 有稳定输出, 表示 leveling 正常

### 6. EGO 多机轨迹广播

两台启动 EGO 后, 检查共享广播:

```bash
ros2 topic info -v /broadcast_bspline
ros2 topic echo /broadcast_bspline --once
```

两台应都能看到 `/broadcast_bspline`, 且消息里的 `drone_id` 应能区分 0 和 1。若日志出现 `Time difference is too large`, 先修 chrony。

### 7. RViz 目标点

笔记本同 WiFi, 同 ROS domain:

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

source /opt/ros/jazzy/setup.bash
source overlay_ws/install/setup.bash
rviz2 -d overlay_ws/src/ego_swarm_real_bringup/rviz/ego_swarm_ground_station.rviz
```

双机 RViz 有两个 `2D Goal Pose` 工具:

```text
drone0 goal: /drone_0/move_base_simple/goal
drone1 goal: /drone_1/move_base_simple/goal
```

发目标前必须在 Tool Properties 面板核对 topic。点目标就是给对应飞机下达 EGO 目标, 不应在未准备好时随意点击。

## 建议测试流程

### 阶段 A: 双机地面拆桨联调

目标: 证明两台 ROS/PX4/SLAM/EGO 能互通, 不进入真实飞行控制。

每台机分别启动:

```bash
sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 2000000
OPEN_RVIZ=false DRONE_ID=<0或1> ./scripts/run_mid360_fastlio2.sh
DRONE_ID=<0或1> ./scripts/run_ego_single_real.sh
```

注意: 这台 drone1 可用默认 `DRONE_ID=1`; drone0 必须显式传 `DRONE_ID=0`, 并确保自己的 bridge `target_system=1`。

验证:

```bash
ros2 topic list | grep -E 'drone_0|drone_1|broadcast_bspline'
ros2 topic hz /broadcast_bspline
ros2 topic hz /drone_0_planning/pos_cmd
ros2 topic hz /drone_1_planning/pos_cmd
ros2 topic hz /drone_0/fmu/in/trajectory_setpoint
ros2 topic hz /drone_1/fmu/in/trajectory_setpoint
```

### 阶段 B: 单机飞、另一台地面广播

目标: 先证明一台飞行时能看到另一台的轨迹/占位信息。

建议顺序:

1. drone0 单飞小目标, drone1 地面通电并运行 SLAM/EGO 广播
2. drone1 单飞小目标, drone0 地面通电并运行 SLAM/EGO 广播
3. 每次只移动一台, 另一台不进入 Offboard

检查:

```bash
ros2 topic echo /broadcast_bspline --once
ros2 topic echo /drone_0_planning/pos_cmd --once
ros2 topic echo /drone_1_planning/pos_cmd --once
```

### 阶段 C: 双机同飞

首次双机同飞只做大间距、低速、小目标:

- 起飞间距建议先 >= 3 m
- 航线间距应明显大于 `optimization/swarm_clearance: 1.2`
- 一开始不要交叉航线
- 每台单次 RViz goal 移动距离建议 0.5 到 1.0 m
- 每次只点一个目标, 等轨迹稳定后再点下一台
- 两名飞手分别持遥控器, 任一异常立即退出 Offboard 或切回人工可控模式

推荐录包:

```bash
DRONE_IDS="0 1" ./scripts/record_ego_swarm_experiment.sh
```

默认记录:

```text
/broadcast_bspline
/tf
/tf_static
/drone_N/aft_mapped_to_init
/drone_N/aft_mapped_to_init_level
/drone_N/cloud_registered_level
/drone_N_planning/pos_cmd
/drone_N_planning/bspline
/drone_N_planning/swarm_trajs
/drone_N_grid/grid_map/occupancy_inflate
/drone_N/fmu/out/vehicle_odometry
/drone_N/fmu/in/trajectory_setpoint
```

## 当前离线测试数据

### 构建

命令:

```bash
source /opt/ros/jazzy/setup.bash
source slam_ws/install/setup.bash
source nav_ws/install/setup.bash
source overlay_ws/install/setup.bash
cd overlay_ws
colcon build --symlink-install --packages-select \
  fast_lio2_mid360_bringup \
  fast_livo2_mid360_bringup \
  fastlio2_to_ego_swarm_leveling \
  fastlio2_to_px4_odometry \
  position_cmd_to_px4_bridge \
  ego_swarm_real_bringup \
  gvf_path_following_msgs \
  gvf_ismc_path_following \
  gvf_ismc_real_bringup \
  --event-handlers console_direct+
```

结果:

```text
Summary: 9 packages finished [17.6s]
```

### 测试

命令:

```bash
source /opt/ros/jazzy/setup.bash
source slam_ws/install/setup.bash
source nav_ws/install/setup.bash
source overlay_ws/install/setup.bash
cd overlay_ws
colcon test --packages-select \
  fast_lio2_mid360_bringup \
  fast_livo2_mid360_bringup \
  fastlio2_to_ego_swarm_leveling \
  fastlio2_to_px4_odometry \
  position_cmd_to_px4_bridge \
  ego_swarm_real_bringup \
  gvf_path_following_msgs \
  gvf_ismc_path_following \
  gvf_ismc_real_bringup \
  --event-handlers console_direct+
colcon test-result --verbose
```

结果:

```text
Summary: 9 packages finished [17.1s]
Summary: 219 tests, 0 errors, 0 failures, 0 skipped
```

### launch 参数检查

`fast_lio2_mid360_bringup mid360_fastlio2.launch.py --show-args`:

```text
drone_id default: 1
livox_config_file default: .../fast_lio2_mid360_bringup/config/MID360_config_drone_1.json
publish_freq default: 10.0
xfer_format default: 1
multi_topic default: 0
odom_topic default: /drone_<drone_id>/aft_mapped_to_init
```

`ego_swarm_real_bringup single_real.launch.py --show-args`:

```text
drone_id default: 1
odom_topic default: /drone_<drone_id>/aft_mapped_to_init
cloud_topic default: /drone_<drone_id>/cloud_registered
odom_bridge_type default: fastlio2
planner_input_mode default: fastlio2_leveled
leveled_odom_topic default: /drone_<drone_id>/aft_mapped_to_init_level
leveled_cloud_topic default: /drone_<drone_id>/cloud_registered_level
leveled_frame_id default: camera_init_level
```

### 已确认 ROS 包可见

在 Jazzy + `slam_ws` + `nav_ws` + `overlay_ws` 环境下可见:

```text
ego_planner
ego_swarm_real_bringup
fast_lio2_mid360_bringup
fastlio2_to_ego_swarm_leveling
fastlio2_to_px4_odometry
position_cmd_to_px4_bridge
px4_msgs
quadrotor_msgs
traj_utils
```

## 不应跳过的现场数据记录

每次双机测试前记录到日志或实验记录里:

```text
date/time:
drone0 WiFi IP:
drone1 WiFi IP:
ground station IP:
drone0 MID360 IP:
drone1 MID360 IP:
drone0 MAV_SYS_ID:
drone1 MAV_SYS_ID:
drone0 XRCE namespace:
drone1 XRCE namespace:
drone0 world_origin_xyz:
drone1 world_origin_xyz:
drone0 level_rpy_rad:
drone1 level_rpy_rad:
chronyc tracking drone0 System time:
chronyc tracking drone1 System time:
ros2 topic hz /broadcast_bspline:
ros2 topic hz /drone_0/fmu/out/vehicle_odometry:
ros2 topic hz /drone_1/fmu/out/vehicle_odometry:
ros2 topic hz /drone_0/aft_mapped_to_init_level:
ros2 topic hz /drone_1/aft_mapped_to_init_level:
RViz relative pose matches measured spacing: yes/no
propellers removed during ground test: yes/no
pilot takeover tested: yes/no
```

## 是否还需要改代码

以当前离线验证结果看, drone1 侧继续做双机地面联调前没有新的代码修改阻塞项。应优先做现场确认和拆桨测试。

仍可能需要后续改动的情况:

- drone0 使用同一套 drone1 默认配置, 导致 `target_system`、Livox IP 或默认话题不匹配
- 双机 RViz 操作时容易点错 drone0/drone1 goal 工具
- 现场发现 `world_origin_xyz` 或 `level_rpy_rad` 与实际摆放/安装角不符
- DDS 在实际 WiFi 下发现不稳定, 需要固定网段、关闭 AP 隔离或加 Fast DDS discovery 配置
- 希望 bridge 对 drone0/drone1 自动推导 `target_system = drone_id + 1`, 减少人工配置错误

其中最后一项是工程质量改进, 不是当前 drone1 地面联调的硬阻塞。正式双机放飞前, drone0 侧必须单独复核。
