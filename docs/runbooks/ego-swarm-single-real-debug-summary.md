# EGO-Swarm 单机实机桥接问题总结

更新时间：`2026-04-24`

## 1. 正常情况下应该是什么路径

当前这套单机实机复刻，正常的数据路径应该是：

1. `FAST-LIVO` 发布真实定位和点云
   - `/aft_mapped_to_init`
   - `/cloud_registered`
2. `ego_planner_node` 使用真实定位和障碍信息进行规划
   - 输入：`/aft_mapped_to_init`、`/cloud_registered`
   - 输出：`/drone_0_planning/bspline`
3. `traj_server` 将 B-spline 轨迹展开成时序参考命令
   - 输入：`/drone_0_planning/bspline`
   - 输出：`/drone_0_planning/pos_cmd`
4. `fastlivo_to_px4_odometry` 将 FAST-LIVO 的 ROS odom 转为 PX4 外部视觉里程计
   - 输入：`/aft_mapped_to_init`
   - 输出：`/fmu/in/vehicle_visual_odometry`
5. `position_cmd_to_px4_bridge` 将 `PositionCommand` 转为 PX4 offboard 输入
   - 输入：
     - `/drone_0_planning/pos_cmd`
     - `/aft_mapped_to_init`
     - `/fmu/out/vehicle_local_position`
     - `/fmu/out/vehicle_status`
   - 输出：
     - `/fmu/in/offboard_control_mode`
     - `/fmu/in/trajectory_setpoint`
     - `/fmu/in/vehicle_command`
6. `PX4` 内部根据外部里程计和轨迹指令做闭环控制
   - 外部定位进入 EKF
   - Offboard 位置控制接管飞行

简化成一句话：

`FAST-LIVO -> ego_planner -> traj_server -> position_cmd_to_px4_bridge -> PX4`

同时：

`FAST-LIVO -> fastlivo_to_px4_odometry -> PX4 EKF`

## 2. 目前已经确认正常的部分

下面这些链路已经实测通过：

- `FAST-LIVO` 能正常启动
- `/aft_mapped_to_init` 正常发布
- `/cloud_registered` 正常发布
- 发送目标点后，`ego_planner` 能生成规划结果
- `/drone_0_planning/bspline` 正常生成
- `/drone_0_planning/pos_cmd` 能被 `traj_server` 发布
- `MicroXRCEAgent` 能通过 `/dev/ttyUSB0` 建立 session
- `fastlivo_to_px4_odometry -> /fmu/in/vehicle_visual_odometry` 这条链是通的
- PX4 内部 `listener vehicle_local_position` 有数据

这说明：

- 规划链本身不是主要问题
- `FAST-LIVO -> PX4` 的外部视觉里程计入口不是主要问题

## 3. 当前真正遇到的问题

当前阻塞点在：

- `position_cmd_to_px4_bridge` 一订阅
  - `/fmu/out/vehicle_local_position`
  - `/fmu/out/vehicle_status`
- 就持续报错：
  - `Fast CDR exception deserializing message of type px4_msgs::msg::dds_::VehicleLocalPosition_`
  - `Fast CDR exception deserializing message of type px4_msgs::msg::dds_::VehicleStatus_`

这说明：

- PX4 确实在 DDS 图里发布了这两个 topic
- 但是 companion 侧的 ROS2 节点在反序列化这两个消息时失败
- 错误发生在 callback 之前，不是 bridge 自己的业务逻辑写坏了

直接后果是：

- `position_cmd_to_px4_bridge` 拿不到有效的 PX4 心跳/状态
- bridge 状态机无法进入正常的 `STREAM_SETPOINT`
- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `/fmu/in/vehicle_command`
  无法稳定对 PX4 输出

所以现在没打通的不是 planner，而是：

- `PX4 -> ROS` 方向上的这两个 DDS 出站消息

## 4. 为什么 `vrpn_to_px4_node` 能用，不代表 bridge 这里没版本问题

`vrpn_to_px4_node` 的路径是：

- 订阅动捕 `PoseStamped`
- 发布 `px4_msgs::msg::VehicleOdometry`
- 输出到 `/fmu/in/vehicle_visual_odometry`

也就是说它只走了：

`ROS -> PX4`

并且只用到了一个消息类型：

- `px4_msgs::msg::VehicleOdometry`

而当前 `position_cmd_to_px4_bridge` 出问题的路径是：

- 订阅 `/fmu/out/vehicle_local_position`
- 订阅 `/fmu/out/vehicle_status`

这走的是：

`PX4 -> ROS`

并且用到的是另外两个消息类型：

- `px4_msgs::msg::VehicleLocalPosition`
- `px4_msgs::msg::VehicleStatus`

因此：

- `vrpn_to_px4_node` 能正常给 PX4 输入外部定位
- 只能证明 `VehicleOdometry` 这条输入路径当前可用
- 不能证明 `VehicleLocalPosition` 和 `VehicleStatus` 的 DDS 出站反序列化也一定可用

## 5. 已经排除掉的干扰项

下面这些问题已经查过，不是当前主因：

- 不是 `ego_planner` 没出轨迹
- 不是 `traj_server` 没出 `PositionCommand`
- 不是 `/fmu/out/vehicle_status_v1` 话题名写错导致的唯一问题
- 不是 bridge 没有订阅到 `/drone_0_planning/pos_cmd`
- 不是 `/aft_mapped_to_init` 的 QoS 不兼容
- 不是 `/fmu/in/*` publisher 还停留在默认 `Reliable`
  - 已经改成对齐 AIM 的 `BestEffort + Volatile`
- 不是重复 bridge 进程导致的全部问题
  - 虽然之前存在重复实例，但清理后 `Fast CDR exception` 仍然存在
- 不是单纯 build 脏了
  - 已经对 `px4_msgs` 和 `position_cmd_to_px4_bridge` 做过定向 clean rebuild
  - 问题仍然存在

## 6. 当前最可能的根因

当前最像的根因是：

1. 飞控当前固件是 `PX4 1.14.3dev`
2. companion 侧当前使用的 `px4_msgs` 虽然消息文本和 AIM 本地版本看起来一致
3. 但对 `VehicleLocalPosition` / `VehicleStatus` 这两个 topic，运行时实际的 DDS 序列化布局并没有正确匹配

换句话说：

- 不是“整个 `px4_msgs` 都不能用”
- 而是“至少这两个 `PX4 -> ROS` 的出站消息当前不能被正确解出来”

## 7. 接下来应该做什么

当前建议按这个顺序处理：

1. 优先拿到飞控当前 PX4 固件的精确 commit
   - 当前只知道版本是 `1.14.3dev`
   - 最好拿到具体源码 commit，而不是只拿版本号
2. 用该 commit 对应的 `px4_msgs` 重新替换当前工作空间中的
   - `overlay_ws/src/px4_msgs`
3. 清理并重编：
   - `px4_msgs`
   - `position_cmd_to_px4_bridge`
4. 重新启动 `position_cmd_to_px4_bridge`
5. 只验证一件事：
   - 订阅 `/fmu/out/vehicle_local_position`
   - `/fmu/out/vehicle_status`
   时是否还报 `Fast CDR exception`

如果这一步恢复正常，再继续验证：

- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `/fmu/in/vehicle_command`

## 8. 当前结论

当前这套单机实机复刻的状态是：

- `FAST-LIVO -> ego_planner -> traj_server`：基本通
- `FAST-LIVO -> PX4 external vision odometry`：通
- `PositionCommand -> PX4 Offboard`：卡在 `PX4 -> ROS` 状态话题反序列化失败

因此现在不适合直接实飞。

在 `VehicleLocalPosition` / `VehicleStatus` 的 DDS 反序列化问题解决之前，不能认为 offboard 桥接已经打通。
