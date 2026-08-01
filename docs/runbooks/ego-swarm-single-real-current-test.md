# EGO-Swarm 单机实机当前测试手册

更新时间：`2026-04-24`

## 1. 目的

这份手册只覆盖**当前已经打通到的链路**，目标是验证：

1. `FAST-LIVO -> ego_planner -> traj_server`
2. `FAST-LIVO -> PX4 external vision odometry`
3. `PositionCommand -> /fmu/in/offboard_control_mode`
4. `PositionCommand -> /fmu/in/trajectory_setpoint`
[fastlivo_mapping-2] [INFO] [1777036495.356301267] [laserMapping]: get imu at
  time: 1777036492.913977
  [fastlivo_mapping-2] [WARN] [1777036495.356330484] [laserMapping]: IMU and
  LiDAR not synced! delta time: 2.305465 .
  [fastlivo_mapping-2]
  [fastlivo_mapping-2] [INFO] [1777036495.356462680] [laserMapping]: get imu at
  time: 1777036492.918205
  [fastlivo_mapping-2] [WARN] [1777036495.356486425] [laserMapping]: IMU and
  LiDAR not synced! delta time: 2.301237 .
  [fastlivo_mapping-2]
  [fastlivo_mapping-2] [INFO] [1777036495.356616509] [laserMapping]: get imu at
  time: 1777036492.924077
  [fastlivo_mapping-2] [WARN] [1777036495.356640446] [laserMapping]: IMU and
  LiDAR not synced! delta time: 2.295365 .
  [fastlivo_mapping-2]
  [fastlivo_mapping-2] [ LIO ] Raw feature num: 13558, downsampled feature
  num:11880 effective feature num: 11579 average residual: 0.0218056
  [fastlivo_mapping-2] [ LIO ] Raw feature num: 13558, downsampled feature
  num:11880 effective feature num: 11582 average residual: 0.0207805

当前这份手册不把“PX4 已经成功切进 `OFFBOARD` 并成功 `ARM`”作为默认已完成项。

## 2. 当前正常链路

当前单机实机链路是：

- `FAST-LIVO` 发布
  - `/aft_mapped_to_init`
  - `/cloud_registered`
- `ego_planner_node` 订阅真实定位和点云，输出
  - `/drone_0_planning/bspline`
- `traj_server` 输出
  - `/drone_0_planning/pos_cmd`
- `fastlivo_to_px4_odometry` 输出
  - `/fmu/in/vehicle_visual_odometry`
- `position_cmd_to_px4_bridge` 订阅
  - `/aft_mapped_to_init`
  - `/fmu/out/vehicle_odometry`
  - `/drone_0_planning/pos_cmd`
- `position_cmd_to_px4_bridge` 发布
  - `/fmu/in/offboard_control_mode`
  - `/fmu/in/trajectory_setpoint`
  - `/fmu/in/vehicle_command`

## 3. 测试前说明

当前 bridge 的关键行为是：

- 不再依赖 `/fmu/out/vehicle_local_position`
- 不再默认依赖 `/fmu/out/vehicle_status`
- heartbeat 改用 `/fmu/out/vehicle_odometry`
- planner 没收到目标点之前，会一直停在 `WAIT_TARGET`
- bridge 没收到 `PositionCommand` 之前，会一直停在 `WAIT_PX4`

所以当前测试一定要包含“发目标点”这一步。

## 4. 建议启动顺序

建议 3 个终端。

### 4.1 终端 A：启动 XRCE Agent

```bash
sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 2000000
```

### 4.2 终端 B：启动 FAST-LIVO

```bash
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/slam_ws/install/setup.bash
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 launch fast_livo2_mid360_bringup mid360_lio.launch.py \
  livox_config_file:=/home/morphing01/Drone_SLAM/slam_ws/src/livox_ros_driver2/config/MID360_config.json
```

### 4.3 终端 C：启动 planner + 两个 bridge

```bash
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/nav_ws/install/setup.bash
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 launch ego_swarm_real_bringup single_real.launch.py
```

## 5. 启动后先看什么

### 5.1 FAST-LIVO 关键话题

```bash
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/nav_ws/install/setup.bash
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic hz /aft_mapped_to_init
ros2 topic echo /aft_mapped_to_init --once
```

预期：

- `/aft_mapped_to_init` 正常发布
- 频率大约 `10 Hz`

### 5.2 PX4 heartbeat 关键话题

```bash
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/nav_ws/install/setup.bash
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic hz /fmu/out/vehicle_odometry
ros2 topic echo /fmu/out/vehicle_odometry --qos-reliability best_effort --qos-durability transient_local --once
```

预期：

- `/fmu/out/vehicle_odometry` 正常发布
- 不再出现 `Fast CDR exception`

### 5.3 启动后 bridge 的正常日志

如果 bridge 工作正常，启动后你应该先看到：

- `received first FAST-LIVO odometry`
- `received first VehicleOdometry heartbeat`

然后状态会停在：

- `state=WAIT_PX4 ... command=0`

这时是正常的，因为还没发目标点。

## 6. 发送目标点

### 6.1 直接用终端发一个目标点

```bash
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/nav_ws/install/setup.bash
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: world}, pose: {position: {x: 2.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}"
```

也可以直接在 RViz 里发 `2D Goal`。

### 6.2 目标点发出后的正常日志

这时应当看到：

- planner 从 `WAIT_TARGET` 进入 `GEN_NEW_TRAJ`
- 再进入 `EXEC_TRAJ`
- bridge 打印
  - `received first PositionCommand ...`
- bridge 状态从
  - `WAIT_PX4`
  进入
  - `STREAM_SETPOINT`
  再进入
  - `ACTIVE`

## 7. 验证 `/fmu/in/*` 是否真的在发

### 7.1 推荐验证方式

`ros2 topic echo` 对某些 PX4 输入话题不总是稳定，当前更可靠的方式是用一个最小 Python 订阅器计数。

```bash
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/nav_ws/install/setup.bash
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

python3 - <<'PY'
import rclpy, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint

class Probe(Node):
    def __init__(self):
        super().__init__('px4_in_probe')
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.offboard = 0
        self.traj = 0
        self.create_subscription(OffboardControlMode, '/fmu/in/offboard_control_mode', self.cb_off, qos)
        self.create_subscription(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.cb_traj, qos)

    def cb_off(self, msg):
        self.offboard += 1

    def cb_traj(self, msg):
        self.traj += 1

rclpy.init()
node = Probe()
start = time.time()
while time.time() - start < 4.0:
    rclpy.spin_once(node, timeout_sec=0.1)
print(f'offboard_count={node.offboard}')
print(f'trajectory_count={node.traj}')
node.destroy_node()
rclpy.shutdown()
PY
```

当前实测参考值：

- `offboard_count=81`
- `trajectory_count=80`

这说明当前 `/fmu/in/offboard_control_mode` 和 `/fmu/in/trajectory_setpoint` 已经在真实发样本。

### 7.2 辅助查看

```bash
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/nav_ws/install/setup.bash
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic info /fmu/in/offboard_control_mode -v
ros2 topic info /fmu/in/trajectory_setpoint -v
```

预期：

- `position_cmd_to_px4_bridge` 是 publisher
- `_CREATED_BY_BARE_DDS_APP_` 是 PX4 侧 subscriber

## 8. 当前可以认定已经打通的部分

当前可以认为以下链路已经打通：

- `FAST-LIVO -> ego_planner`
- `ego_planner -> traj_server -> PositionCommand`
- `FAST-LIVO -> /fmu/in/vehicle_visual_odometry`
- `PositionCommand -> /fmu/in/offboard_control_mode`
- `PositionCommand -> /fmu/in/trajectory_setpoint`

## 9. 当前还没有最终确认的部分

下面这些还需要单独验证：

- PX4 是否真正接受了 `VehicleCommand`
- PX4 是否成功切换到 `OFFBOARD`
- PX4 是否真正 `ARM`
- 实机是否会跟踪轨迹飞行

下一步建议直接检查：

```bash
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/nav_ws/install/setup.bash
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic list -t | rg '/fmu/out/vehicle_command_ack|/fmu/out/vehicle_status'
```

然后再看：

```bash
ros2 topic echo /fmu/out/vehicle_command_ack --qos-reliability best_effort --qos-durability transient_local --once
ros2 topic echo /fmu/out/vehicle_status --qos-reliability best_effort --qos-durability transient_local --once
```

## 10. 常见现象和解释

### 10.1 bridge 一直打印 `command=0`

原因通常是：

- planner 还在 `WAIT_TARGET`
- 还没发目标点

### 10.2 bridge 卡在 `WAIT_FASTLIVO`

原因通常是：

- `/aft_mapped_to_init` 没进来
- FAST-LIVO 没起好

### 10.3 bridge 卡在 `WAIT_PX4`

原因通常是：

- `/fmu/out/vehicle_odometry` 没进来
- XRCE agent 或飞控 DDS 没正常起来
- 或者还没收到 `PositionCommand`

### 10.4 以前刷 `Fast CDR exception`

之前报错的根因是：

- bridge 订阅了
  - `/fmu/out/vehicle_local_position`
  - `/fmu/out/vehicle_status`
- 这两个 topic 在当前环境下反序列化不稳定

当前已经绕开：

- heartbeat 改成 `/fmu/out/vehicle_odometry`
- 默认不再依赖 `/fmu/out/vehicle_status`

## 11. 一句话总结

当前这版单机实机链路，已经可以稳定验证到：

`发目标点 -> planner 出轨迹 -> bridge 收到 PositionCommand -> /fmu/in/offboard_control_mode 和 /fmu/in/trajectory_setpoint 连续发样本`

下一步重点不再是 planner，而是：

- `VehicleCommand` 是否被 PX4 接受
- `OFFBOARD`/`ARM` 是否真正成功
- 最终实机是否会跟踪轨迹运动
