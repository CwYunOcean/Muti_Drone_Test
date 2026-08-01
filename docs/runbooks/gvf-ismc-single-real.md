# GVF + ISMC 单机实机测试手册

更新时间：`2026-05-08`

## 1. 目的

这份手册用于验证 `GVF + ISMC` 单机实机链路是否完整打通，重点确认以下数据流：

1. `FAST-LIO2 -> /aft_mapped_to_init`
2. `/aft_mapped_to_init -> fastlio2_to_ego_swarm_leveling -> /aft_mapped_to_init_level`
3. `/aft_mapped_to_init_level -> gvf_reference_node -> /gvf/reference`
4. `/gvf/reference + /aft_mapped_to_init_level -> ismc_velocity_tracker_node -> /drone_0_planning/pos_cmd`
5. `/drone_0_planning/pos_cmd -> position_cmd_to_px4_bridge -> /fmu/in/*`

本手册只覆盖台架联调、首次低风险接管和论文数据采集，不包含激进飞行科目。

## 1.1 `/aft_mapped_to_init_level` 在哪里维护

`/aft_mapped_to_init_level` 不是 `FAST-LIO2` 原生输出，而是本仓库本地 leveling 节点维护的二次里程计。

数据流是：

1. `FAST-LIO2` 发布 `/aft_mapped_to_init`
2. `fastlio2_to_ego_swarm_leveling_node` 订阅 `/aft_mapped_to_init`
3. 节点按 `level_rpy_rad` 对世界系做 leveling
4. 发布 `/aft_mapped_to_init_level`

当前维护位置：

- 配置文件：[fastlio2_to_ego_swarm_leveling.yaml](/home/morphing01/Drone_SLAM/overlay_ws/src/fastlio2_to_ego_swarm_leveling/config/fastlio2_to_ego_swarm_leveling.yaml:1)
- 节点实现：[fastlio2_to_ego_swarm_leveling_node.cpp](/home/morphing01/Drone_SLAM/overlay_ws/src/fastlio2_to_ego_swarm_leveling/src/fastlio2_to_ego_swarm_leveling_node.cpp:1)

关键参数：

- `input_odom_topic: /aft_mapped_to_init`
- `output_odom_topic: /aft_mapped_to_init_level`
- `level_rpy_rad`

如果你后面觉得水平面定义不对，优先改这里的 `level_rpy_rad`，不要先改 `GVF` 节点。

## 2. 启动前准备

- 确认飞控、遥控器、机体供电正常。
- 确认 `MID360` 已连接，串口设备为 `/dev/ttyUSB0`。
- 确认当前终端使用相同 DDS 配置：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

- 如 `FASTDDS_DEFAULT_PROFILES_FILE` 已在本机流程中使用，保持与既有实机流程一致后再启动。

## 3. 建议启动顺序

建议固定使用 3 个终端。

### 3.1 终端 A：启动 XRCE Agent

```bash
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 2000000
```

### 3.2 终端 B：启动 FAST-LIO2

```bash
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/slam_ws/install/setup.bash
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 launch fast_lio2_mid360_bringup mid360_fastlio2.launch.py \
  livox_config_file:=/home/morphing01/Drone_SLAM/slam_ws/src/livox_ros_driver2/config/MID360_config.json
```

### 3.3 终端 C：启动 GVF + ISMC + PX4 bridge

```bash
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 launch gvf_ismc_real_bringup single_real.launch.py \
  use_acceleration_feedforward:=true
```

注意：

- 当前 `gvf_ismc_real_bringup` 路径下，bridge 默认 `不会` 自动请求 `Offboard` 和 `Arm`
- 它只会持续发送 setpoint
- 是否真正切入 `Offboard`，由操作者通过遥控器或 QGC 手动完成

## 4. 启动后先看什么

先确认定位、参考轨迹、速度跟踪和 bridge 状态都正常。

### 4.1 Level 后里程计频率

```bash
ros2 topic hz /aft_mapped_to_init_level
```

预期：

- 频率稳定，持续发布。
- 没有明显断流或长时间卡顿。

### 4.2 GVF 参考输出

```bash
ros2 topic echo /gvf/reference --once
```

预期：

- 能收到一帧有效参考。
- 参考方向和当前测试轨迹配置一致。

### 4.3 位置指令频率

```bash
ros2 topic hz /drone_0_planning/pos_cmd
```

预期：

- 持续发布。
- 频率稳定，足够支撑 Offboard 连续输入。

### 4.4 bridge 状态

重点确认 bridge 已经开始稳定 stream setpoint；`ACTIVE` 是否出现，取决于你是否已经手动切入 `Offboard` 且机体处于 armed。

预期顺序：

- 已收到 PX4 心跳
- 已收到 `/drone_0_planning/pos_cmd`
- 状态进入 `ACTIVE`

如果还没手动切 `Offboard`，停在非 `ACTIVE` 是正常的。
如果已经手动切 `Offboard` 且已 armed，但仍始终停在非 `ACTIVE`，不要进入正式实飞。

### 4.5 检查加速度前馈是否真的送进 PX4

当前默认实验链路应当开启 `use_acceleration_feedforward:=true`，否则 `ISMC` 外环算出的
`/drone_0_planning/pos_cmd.acceleration` 不会真正进入 `PX4`。

起飞前至少检查一次：

```bash
ros2 topic echo /drone_0_planning/pos_cmd --once
ros2 topic echo /fmu/in/trajectory_setpoint --once
```

预期：

- `/drone_0_planning/pos_cmd.acceleration.{x,y,z}` 不是长期全零
- `/fmu/in/trajectory_setpoint.acceleration` 也不是长期 `NaN` 或全零

如果前者有值、后者没有值，说明当前并没有把 `ISMC` 外环真正送进飞控，不要继续用这组结果判断控制器优劣。

## 4.6 建议的快速可视化

最少建议开 1 个 `RViz` 和 1 个 `rqt_plot`。

### RViz

没有单独的 `gvf_ismc` 配套 rviz 配置文件时，先复用现有 FAST-LIO2 视图：

```bash
rviz2 -d /home/morphing01/Drone_SLAM/overlay_ws/src/fast_livo2_mid360_bringup/rviz/fast_livo2_with_base_link.rviz
```

在 `RViz` 里重点加这些显示项：

- `TF`
- `Odometry`
  - topic: `/aft_mapped_to_init`
- `Odometry`
  - topic: `/aft_mapped_to_init_level`
- `Path`
  - topic 可后续由 bag 或辅助节点生成

重点观察：

- level 前后轨迹是否只发生坐标系“扶正”，而不是形状畸变
- `body_level` 是否在 `camera_init_level` 下连续变化
- 高度平面是否符合预期

### rqt_plot

建议至少画这些量：

```bash
rqt_plot \
  /gvf/reference/phi1 \
  /gvf/reference/phi2 \
  /gvf/reference/desired_velocity/x \
  /gvf/reference/desired_velocity/y \
  /gvf/reference/desired_velocity/z \
  /drone_0_planning/pos_cmd/velocity/x \
  /drone_0_planning/pos_cmd/velocity/y \
  /drone_0_planning/pos_cmd/velocity/z
```

如果要和实际速度对比，再额外打开：

```bash
rqt_plot \
  /aft_mapped_to_init_level/twist/twist/linear/x \
  /aft_mapped_to_init_level/twist/twist/linear/y \
  /aft_mapped_to_init_level/twist/twist/linear/z
```

## 5. 台架联调建议

第一次接电联调时，先做不飞行的链路检查：

1. 终端 A/B/C 全部启动成功。
2. `FAST-LIO2` 正常发布 `/aft_mapped_to_init`。
3. leveling 正常发布 `/aft_mapped_to_init_level`。
4. `gvf_reference_node` 正常发布 `/gvf/reference`。
5. `ismc_velocity_tracker_node` 正常发布 `/drone_0_planning/pos_cmd`。
6. bridge 持续发送 `/fmu/in/*`。
7. 在真正需要接管时，再由操作者手动切 `Offboard`。

如果这一步没有稳定通过，不要直接上桨试飞。

## 6. 首次接管顺序

第一次实飞只做低风险动作，顺序固定如下：

1. 先用遥控器切到 `Position` 模式，手动起飞到 `1.2m`。
2. 稳住悬停后，确认 bridge 正在持续发送 setpoint。
3. 由操作者手动切入 `Offboard`。
4. 先验证一个很小的速度参考，观察机体响应方向是否正确。
5. 小参考确认正常后，再做大半径圆轨迹。
6. 圆轨迹正常后，再做低速三叶草曲线。

执行原则：

- 每一步只在上一步稳定后才继续。
- 任意时刻出现漂移、方向异常、掉高或姿态突变，立即切回人工模式。
- 首飞不追求速度，只追求链路稳定和接管平顺。

## 7. 论文数据采集

每次正式实验至少同步保留以下数据：

### 7.1 rosbag

建议录制整条控制链路，至少包含：

- `/aft_mapped_to_init`
- `/aft_mapped_to_init_level`
- `/gvf/reference`
- `/drone_0_planning/pos_cmd`
- bridge 相关状态话题

建议直接用脚本：

```bash
./scripts/record_gvf_ismc_experiment.sh
```

默认会录：

- `/aft_mapped_to_init`
- `/aft_mapped_to_init_level`
- `/gvf/reference`
- `/drone_0_planning/pos_cmd`
- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `/fmu/in/vehicle_command`
- `/fmu/out/vehicle_status_v1`
- `/fmu/out/vehicle_odometry`

示例：

```bash
ros2 bag record \
  /aft_mapped_to_init \
  /aft_mapped_to_init_level \
  /gvf/reference \
  /drone_0_planning/pos_cmd
```

### 7.2 PX4 ULog

- 每次飞行都保存对应 `ULog`。
- 记录飞行日期、场地、机体编号、参数版本。

### 7.3 视频

- 至少保留地面观察视频。
- 如条件允许，同时保留机载或屏幕录像。

### 7.4 重点指标

复盘时重点整理：

- `|phi2|`
- 高度误差
- 速度误差
- Offboard 连续性

## 7.5 和 MATLAB 作图怎么对齐

`/home/morphing01/Drone_SLAM/GVF_ws/DDSMC_three_leaf.m` 当前主要看 3 类结果：

1. 三叶草轨迹
2. `|phi2|` 误差曲线
3. `v_d` 和 `v_actual` 的分轴对比

ROS 侧可直接对齐成下面这些量：

### 轨迹图

- MATLAB:
  - `x_hist`
  - `y_hist`
  - `z_hist`
- ROS:
  - `/aft_mapped_to_init_level.pose.pose.position.{x,y,z}`

### 误差曲线

- MATLAB:
  - `error_hist(i) = abs(phi2)`
- ROS:
  - `/gvf/reference.phi2`
  - 后处理时取 `abs(phi2)`

### 期望速度

- MATLAB:
  - `v_d_hist(1:3,:)`
- ROS:
  - `/gvf/reference.desired_velocity.{x,y,z}`

### 实际速度

- MATLAB:
  - `v_actual_hist(1:3,:)`
- ROS:
  - `/aft_mapped_to_init_level.twist.twist.linear.{x,y,z}`

### 高度误差

- MATLAB:
  - 原脚本里 `phi1 = z`
- 实飞当前实现:
  - `phi1 = z - h`
- ROS:
  - `/gvf/reference.phi1`

### 控制器输出

- MATLAB:
  - `a_cmd`
  - `eta_d`
- ROS 当前可直接取：
  - `/drone_0_planning/pos_cmd.acceleration.{x,y,z}`
  - `/drone_0_planning/pos_cmd.yaw`
  - `/drone_0_planning/pos_cmd.yaw_dot`

注意：

- 当前实飞实现不是 MATLAB 完整 `DDSMC` 的 1:1 复现
- 但对“轨迹、`phi2`、期望/实际速度对比”这 3 类图，已经能做一一对应

## 7.6 推荐后处理顺序

1. 从 bag 导出 `/aft_mapped_to_init_level`
2. 从 bag 导出 `/gvf/reference`
3. 按时间戳对齐
4. 先画 `x-y` 轨迹图
5. 再画 `abs(phi2)` 曲线
6. 最后画 `v_{d,x/y/z}` 和 `v_{x/y/z}` 对比

如果只想先复现 MATLAB 当前的 3 张核心图，这已经够了。

建议每次飞完立刻记下：

- 是否成功手动切入 `Offboard`
- 切入 `Offboard` 后 bridge 是否进入 `ACTIVE`
- 是否出现 Offboard 中断
- 接管时是否有明显突变
- 大半径圆与低速三叶草是否完整执行

## 8. 最低放飞门槛

只有同时满足以下条件，才建议进入首次接管：

- `/aft_mapped_to_init_level` 稳定发布
- `/gvf/reference` 可正常输出
- `/drone_0_planning/pos_cmd` 频率稳定
- bridge 已进入 `ACTIVE`
- 人工模式可随时抢回

任一条件不满足，都先停在台架或空转联调阶段。


[rviz2-3]          at line 294 in ./src/buffer_core.cpp
[rviz2-3] Warning: TF_OLD_DATA ignoring data from the past for frame body_level at time 1778237311.488737 according to authority Authority undetectable
[rviz2-3] Possible reasons are listed at http://wiki.ros.org/tf/Errors%20explained
[rviz2-3]          at line 294 in ./src/buffer_core.cpp
[rviz2-3] Warning: TF_OLD_DATA ignoring data from the past for frame body_level at time 1778237311.488737 according to authority Authority undetectable
[rviz2-3] Possible reasons are listed at http://wiki.ros.org/tf/Errors%20explained
[rviz2-3]          at line 294 in ./src/buffer_core.cpp
[INFO] [1778242926.716842848] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[fastlio2_to_ego_swarm_leveling_node-1] [INFO] [1778242926.716848480] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[gvf_reference_node-2] [INFO] [1778242926.716861824] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[rviz2-3] [INF
