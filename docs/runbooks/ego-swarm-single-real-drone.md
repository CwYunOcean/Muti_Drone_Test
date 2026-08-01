# EGO-Swarm 单机实机测试手册

## 1. 目的

这份手册用于验证当前单机实机复刻链路是否连通：

1. `FAST-LIVO -> ego_planner`
2. `ego_planner -> traj_server -> PositionCommand`
3. `FAST-LIVO -> PX4 external vision odometry`
4. `PositionCommand -> PX4 offboard inputs`
5. `RViz` 中的路径规划显示
6. 手拿无人机时，定位和规划轨迹是否随运动更新

本文档同时记录了我在 `2026-04-23` 的实测结果。

## 2. 当前结论

截至 `2026-04-23`，这套链路的实测状态如下：

- 已通过：
  - `overlay_ws` 中新增包的构建与单元测试
  - `FAST-LIVO` 实机启动与 `/aft_mapped_to_init`、`/cloud_registered` 发布
  - `MicroXRCEAgent` 通过 `/dev/ttyUSB0` 建立 PX4 XRCE session
  - `fastlivo_to_px4_odometry -> /fmu/in/vehicle_visual_odometry`
  - `goal -> /drone_0_planning/bspline -> /drone_0_planning/pos_cmd`
- 当前阻塞：
  - `position_cmd_to_px4_bridge` 订阅的是 `/fmu/out/vehicle_status_v1`
  - 本次实测时该 topic `Publisher count = 0`
  - 因此 bridge 状态机不会继续流式发布 `/fmu/in/offboard_control_mode` 和 `/fmu/in/trajectory_setpoint`
- 结论：
  - 规划链已经能工作
  - PX4 外部里程计注入已经工作
  - 但 PX4 offboard 接管这一步还没有最终跑通

## 3. 前提条件

### 3.1 硬件

- MID360 已接通并能被 `livox_ros_driver2` 识别
- 飞控已上电
- PX4 的 XRCE 串口链路接在 `/dev/ttyUSB0`
- 当前主机还能看到 `/dev/ttyACM0`

### 3.2 软件工作空间

- `/home/morphing01/Drone_SLAM/slam_ws`
- `/home/morphing01/Drone_SLAM/nav_ws`
- `/home/morphing01/Drone_SLAM/overlay_ws`

### 3.3 环境变量

PX4 ROS2 话题发现使用 `Fast DDS`，测试时统一使用：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## 4. 离线验证

先确认本地新增包仍然能构建和测试通过。

### 4.1 构建

```bash
source /opt/ros/humble/setup.bash
cd /home/morphing01/Drone_SLAM/overlay_ws
colcon build --symlink-install --packages-select \
  px4_msgs \
  quadrotor_msgs \
  fastlivo_to_px4_odometry \
  position_cmd_to_px4_bridge \
  ego_swarm_real_bringup
```

预期：

- `5 packages finished`
- 无 `FAILED` 包

### 4.2 测试

```bash
source /opt/ros/humble/setup.bash
cd /home/morphing01/Drone_SLAM/overlay_ws
colcon test --packages-select \
  fastlivo_to_px4_odometry \
  position_cmd_to_px4_bridge \
  ego_swarm_real_bringup
```

查看结果：

```bash
cd /home/morphing01/Drone_SLAM/overlay_ws
colcon test-result --all --verbose --test-result-base build/fastlivo_to_px4_odometry/test_results
colcon test-result --all --verbose --test-result-base build/position_cmd_to_px4_bridge/test_results
colcon test-result --all --verbose --test-result-base build/ego_swarm_real_bringup/test_results
```

我本次实测结果：

- `fastlivo_to_px4_odometry`: `3 tests, 0 failures`
- `position_cmd_to_px4_bridge`: `5 tests, 0 failures`
- `ego_swarm_real_bringup`: `4 tests, 0 failures`

## 5. 实机链路验证

建议开 3 个终端。

### 5.1 终端 A：启动 PX4 XRCE Agent

```bash
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 2000000
```

预期日志中出现：

- `session established`
- `participant created`

我本次实测：

- 已看到 `client_key: 0x00000001`
- 已看到 `session established`

### 5.2 终端 B：启动 FAST-LIVO

推荐直接启动 launch，不走脚本中的 `rviz2` 部分：

```bash
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/slam_ws/install/setup.bash
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
ros2 launch fast_livo2_mid360_bringup mid360_lio.launch.py \
  livox_config_file:=/home/morphing01/Drone_SLAM/slam_ws/src/livox_ros_driver2/config/MID360_config.json
```

预期：

- `livox_ros_driver2_node` 正常启动
- `fastlivo_mapping` 正常启动
- 有 `Gravity Alignment Finished`
- 进入正常 LIO update

检查关键话题：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/nav_ws/install/setup.bash
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
ros2 topic list -t | rg 'aft_mapped_to_init|cloud_registered'
ros2 topic echo /aft_mapped_to_init --once
ros2 topic hz /aft_mapped_to_init
ros2 topic hz /cloud_registered
```

我本次实测：

- `/aft_mapped_to_init` 在线
- `/cloud_registered` 在线
- `/aft_mapped_to_init` 约 `10 Hz`
- `/cloud_registered` 约 `10 Hz`

### 5.3 终端 C：启动 planner 和两个 PX4 bridge

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/nav_ws/install/setup.bash
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
ros2 launch ego_swarm_real_bringup single_real.launch.py
```

预期：

- `ego_planner_node`
- `traj_server`
- `fastlivo_to_px4_odometry_node`
- `position_cmd_to_px4_bridge_node`

检查节点：

```bash
ros2 node list
ros2 node info /drone_0_ego_planner_node
ros2 node info /drone_0_traj_server
```

我本次实测：

- `/drone_0_ego_planner_node` 已订阅 `/aft_mapped_to_init`
- `/drone_0_ego_planner_node` 已订阅 `/cloud_registered`
- `/drone_0_traj_server` 已订阅 `/drone_0_planning/bspline`

### 5.4 验证 FAST-LIVO 到 PX4 外部里程计

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/nav_ws/install/setup.bash
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
ros2 topic info /fmu/in/vehicle_visual_odometry -v
ros2 topic echo /fmu/in/vehicle_visual_odometry --once
ros2 topic hz /fmu/in/vehicle_visual_odometry
```

预期：

- `Publisher count: 1`
- 发布者是 `fastlivo_to_px4_odometry`
- 频率约 `10 Hz`

我本次实测：

- 已确认发布者是 `fastlivo_to_px4_odometry`
- 已确认 PX4 侧有订阅者
- `vehicle_visual_odometry` 约 `10 Hz`

### 5.5 发送目标点并验证规划输出

发目标点：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/nav_ws/install/setup.bash
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: camera_init}, pose: {position: {x: 1.5, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}"
```
[ego_planner_node-1] [drone 0 replan 5]==============================================
[ego_planner_node-1] iter(+1)=27,time(ms)=0.111,total_t(ms)=0.112,cost=0.017
[ego_planner_node-1] plan_success=1
[ego_planner_node-1] total time:0.00022,optimize:0.00022,refine:6.6e-06,avg_time=0.00074
[ego_planner_node-1] refine_success=1
[ego_planner_node-1] [FSM]: from REPLAN_TRAJ to EXEC_TRAJ
[ego_planner_node-1] [FSM]: state: EXEC_TRAJ
[ego_planner_node-1] [FSM]: from EXEC_TRAJ to REPLAN_TRAJ
[ego_planner_node-1]
[ego_planner_node-1] [drone 0 replan 6]==============================================
[ego_planner_node-1] iter(+1)=15,time(ms)=0.071,total_t(ms)=0.071,cost=0.014
[ego_planner_node-1] plan_success=1
[ego_planner_node-1] total time:0.00017,optimize:0.00016,refine:5.4e-06,avg_time=0.00066
[ego_planner_node-1] refine_success=1
[ego_planner_node-1] [FSM]: from REPLAN_TRAJ to EXEC_TRAJ
[ego_planner_node-1] [FSM]: state: EXEC_TRAJ
[ego_planner_node-1] [FSM]: from EXEC_TRAJ to REPLAN_TRAJ
[ego_planner_node-1]
[ego_planner_node-1] [drone 0 replan 7]==============================================
[ego_planner_node-1] iter(+1)=11,time(ms)=0.059,total_t(ms)=0.060,cost=0.016
[ego_planner_node-1] plan_success=1
[ego_planner_node-1] total time:0.00015,optimize:0.00015,refine:5.2e-06,avg_time=0.00059
[ego_planner_node-1] refine_success=1
[ego_planner_node-1] [FSM]: from REPLAN_TRAJ to EXEC_TRAJ
[ego_planner_node-1] [FSM]: state: EXEC_TRAJ
[ego_planner_node-1] [FSM]: from EXEC_TRAJ to REPLAN_TRAJ
[ego_planner_node-1]
[ego_planner_node-1] [drone 0 replan 8]==============================================
[ego_planner_node-1] iter(+1)=11,time(ms)=0.060,total_t(ms)=0.061,cost=0.010
[ego_planner_node-1] plan_success=1
[ego_planner_node-1] total time:0.00015,optimize:0.00015,refine:4.8e-06,avg_time=0.00054
[ego_planner_node-1] refine_success=1
[ego_planner_node-1] [FSM]: from REPLAN_TRAJ to EXEC_TRAJ
[ego_planner_node-1] [FSM]: state: EXEC_TRAJ
[ego_planner_node-1] [FSM]: state: EXEC_TRAJ
[ego_planner_node-1] [FSM]: state: EXEC_TRAJ
[ego_planner_node-1] [FSM]: from EXEC_TRAJ to WAIT_TARGET
[ego_planner_node-1] [FSM]: state: WAIT_TARGET
[ego_planner_node-1] wait for goal or trigger.
[ego_planner_node-1] [FSM]: state: WAIT_TARGET
验证输出：

```bash
ros2 topic echo /drone_0_planning/bspline --once
ros2 topic hz /drone_0_planning/pos_cmd
ros2 topic echo /drone_0_planning/pos_cmd --once
```

预期：

- `ego_planner_node` 从 `WAIT_TARGET` 进入 `GEN_NEW_TRAJ`
- 出现一条新的 `/drone_0_planning/bspline`
- `/drone_0_planning/pos_cmd` 持续发布

我本次实测：

- planner 已打印 `Triggered!`
- planner 已打印 `plan_success=1`
- 已抓到有效的 `/drone_0_planning/bspline`
- `/drone_0_planning/pos_cmd` 约 `100 Hz`
- 已抓到有效的 `PositionCommand`

### 5.6 验证 bridge 到 PX4 offboard 输入

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/nav_ws/install/setup.bash
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
ros2 topic info /fmu/in/offboard_control_mode -v
ros2 topic info /fmu/in/trajectory_setpoint -v
ros2 topic info /fmu/in/vehicle_command -v
ros2 topic echo /fmu/in/offboard_control_mode --once
ros2 topic echo /fmu/in/trajectory_setpoint --once
```

理想预期：

- 发布者是 `position_cmd_to_px4_bridge`
- bridge 能持续推送 `OffboardControlMode`
- bridge 能持续推送 `TrajectorySetpoint`

我本次实测：

- 这 3 个 topic 上已经能看到 `position_cmd_to_px4_bridge` 的 publisher
- 但 `echo /fmu/in/offboard_control_mode --once` 和 `echo /fmu/in/trajectory_setpoint --once` 在本轮都没有抓到消息

### 5.7 当前阻塞点确认

继续检查 PX4 状态回读：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/nav_ws/install/setup.bash
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
ros2 topic info /fmu/out/vehicle_status_v1 -v
```

我本次实测结果：

- `/fmu/out/vehicle_status_v1`
  - `Publisher count: 0`
  - `Subscription count: 1`
  - 订阅者是 `position_cmd_to_px4_bridge`

这意味着：

- bridge 当前拿不到 PX4 状态
- 状态机不会进入真正的 offboard setpoint 流
- 所以目前还不能证明 PX4 已接受 offboard 指令

## 6. RViz 路径规划观察

### 6.1 启动方式

当前终端环境没有图形桌面时，`rviz2` 不能直接弹出窗口。桌面环境下请另开终端：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/nav_ws/install/setup.bash
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
rviz2
```

### 6.2 推荐手动添加的显示项

`Fixed Frame` 设为：

- `camera_init`

添加这些 Display：

- `PointCloud2`
  - Topic: `/cloud_registered`
- `PointCloud2`
  - Topic: `/drone_0_grid/grid_map/occupancy_inflate`
- `Marker`
  - Topic: `/drone_0_plan_vis/goal_point`
- `Marker`
  - Topic: `/drone_0_plan_vis/global_list`
- `Marker`
  - Topic: `/drone_0_plan_vis/init_list`
- `Marker`
  - Topic: `/drone_0_plan_vis/optimal_list`
- `Marker`
  - Topic: `/drone_0_plan_vis/a_star_list`
- `TF`

### 6.3 如何观察规划路径

1. 先确认 `/cloud_registered` 已显示
2. 再发一个 `/move_base_simple/goal`
3. 观察：
   - `goal_point`
   - `global_list`
   - `init_list`
   - `optimal_list`
   - `a_star_list`
4. 如果 planner 正常，会看到从当前位置到目标点的规划结果刷新

## 7. 手拿无人机验证路径是否更新

这一阶段不要直接带桨测试。

### 7.1 安全要求

- 建议先拆桨
- 或者保持飞控不上锁、不进 offboard
- 只验证“定位和规划是否更新”，不要先验证实飞跟踪

### 7.2 验证方法

1. 先让 FAST-LIVO 和 planner 都已经正常运行
2. 在 RViz 中已经能看到点云和规划结果
3. 发一个固定目标点
4. 手动移动无人机机体
5. 观察：
   - `/aft_mapped_to_init` 的位姿是否变化
   - `/drone_0_planning/bspline` 是否重新规划
   - `/drone_0_planning/pos_cmd` 的目标位置/航向是否变化
   - RViz 中的轨迹和 marker 是否随当前位置变化而刷新

### 7.3 终端观察命令

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/nav_ws/install/setup.bash
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
ros2 topic echo /aft_mapped_to_init --once
ros2 topic hz /drone_0_planning/bspline
ros2 topic hz /drone_0_planning/pos_cmd
```

判据：

- 如果你手动移动机体后，`/aft_mapped_to_init` 明显变化，且 planner 再次输出新的 `bspline`
- 就说明“手拿无人机时路径会随当前位置更新”这一步已经成立

## 8. 下一步建议

当前推荐按下面顺序继续：

1. 先在桌面环境完成 RViz 可视化观察
2. 再做“拆桨、手拿机体、只看路径更新”的静态安全验证
3. 最后再解决 PX4 `vehicle_status` 回读缺失的问题

在进入真实 offboard 飞行前，必须先解决：

- `position_cmd_to_px4_bridge` 订阅的状态 topic 没有有效 publisher
- 因而当前还不能证明 PX4 已进入 offboard/armed 状态
