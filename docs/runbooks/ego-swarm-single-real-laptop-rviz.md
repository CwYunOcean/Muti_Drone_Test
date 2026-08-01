# EGO-Swarm 单机实机上位机 RViz 接入指南

更新时间：`2026-04-24`

## 1. 目标

这份指南的目标是让笔记本作为上位机：

- 通过同一局域网直接加入当前 ROS2 图
- 在笔记本本地运行 `RViz`
- 直接可视化这台机器上的 `FAST-LIVO + ego_planner` 话题
- 在笔记本 `RViz` 里直接发 `/move_base_simple/goal`

这不是 `SSH X11` 转发方案。
这里推荐的是：

- 传感器、定位、规划、桥接节点全部继续跑在机载这台机器上
- 笔记本只加入同一个 DDS 域，本地开 `RViz`

## 2. 当前推荐架构

当前推荐的连接方式是：

- 机载机器运行：
  - `MicroXRCEAgent`
  - `FAST-LIVO`
  - `ego_swarm_real_bringup`
- 笔记本运行：
  - `rviz2`
  - 可选的 `ros2 topic` 调试命令

这种方式比 `SSH + X11` 更合适，原因是：

- 画面更流畅
- `2D Goal` 工具响应更自然
- 笔记本本地窗口不依赖远端 OpenGL/X11
- ROS2 图本来就是分布式的，RViz 本地订阅是自然用法

## 3. 笔记本前置条件

笔记本至少需要：

- Ubuntu + `ROS2 Humble`
- `rviz2`
- 与机载机器在同一局域网
- 没有被路由器的 AP/Client Isolation 隔离

建议在笔记本上确认：

```bash
source /opt/ros/humble/setup.bash
rviz2 --help >/dev/null
```

如果这条命令能正常执行，说明本地 `RViz` 基础环境是好的。

## 4. 这个方案是否需要完整工作空间

只为了在笔记本上运行 `RViz` 看当前这套配置，通常**不需要**完整拷贝 `nav_ws` 和 `overlay_ws`。

原因是当前 [ego_single_real.rviz](/home/morphing01/Drone_SLAM/overlay_ws/src/ego_swarm_real_bringup/rviz/ego_single_real.rviz) 里用到的显示类型几乎都是标准消息：

- `sensor_msgs/PointCloud2`
- `nav_msgs/Odometry`
- `visualization_msgs/Marker`
- `geometry_msgs/PoseStamped`
- `tf2_msgs/TFMessage`

所以笔记本只需要：

- `ROS2 Humble`
- `rviz2`
- 一份本地可读的 `ego_single_real.rviz`

只有在你还想在笔记本上直接：

- `ros2 topic echo /drone_0_planning/pos_cmd`

这种非标准自定义消息时，笔记本才需要再安装对应消息包，例如 `quadrotor_msgs`。

## 5. 两台机器的环境变量要对齐

### 5.1 必须对齐的项

两台机器都建议显式设置：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
```

### 5.2 `ROS_DOMAIN_ID`

当前这套 `ego + PX4` 实机联调，推荐两台机器都显式设成：

```bash
export ROS_DOMAIN_ID=0
```

原因是当前 PX4 的 `uXRCE-DDS` 话题实际就在 `domain 0`。
如果笔记本或机载机器改成别的域，例如 `30`，就会出现：

- 笔记本能看到 `FAST-LIVO` 和 `ego` 的话题
- 但 `position_cmd_to_px4_bridge` 看不到 `/fmu/out/vehicle_odometry`
- 最终卡在 `heartbeat=0`

如果你后面给机载机器设置了：

```bash
export ROS_DOMAIN_ID=<某个数字>
```

那笔记本必须设成同一个值。

### 5.3 防火墙和网络

如果笔记本能 `SSH` 到机载机器，但 `ROS2` 看不到话题，优先排查：

- 两台机器是否真的在同一子网
- Wi-Fi 路由器是否启用了 AP Isolation
- 本机防火墙是否拦截 DDS 多播/UDP

## 6. 机载机器怎么启动

机载机器继续按当前实机链路启动即可。

### 6.1 终端 A：XRCE Agent

```bash
sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 2000000
```

### 6.2 终端 B：FAST-LIVO

```bash
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/slam_ws/install/setup.bash
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

ros2 launch fast_livo2_mid360_bringup mid360_lio.launch.py \
  livox_config_file:=/home/morphing01/Drone_SLAM/slam_ws/src/livox_ros_driver2/config/MID360_config.json
```

### 6.3 终端 C：ego + 两个 bridge

```bash
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/nav_ws/install/setup.bash
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

ros2 launch ego_swarm_real_bringup single_real.launch.py
```

## 7. 把 RViz 配置带到笔记本

笔记本只需要能访问到一份本地配置文件。最简单是从机载机器拷过去：

```bash
scp morphing01@<机载IP>:/home/morphing01/Drone_SLAM/overlay_ws/src/ego_swarm_real_bringup/rviz/ego_single_real.rviz .
```

如果你笔记本上也同步了这个仓库，直接用仓库里的同路径文件也可以。

## 8. 笔记本如何确认已经看到远端 ROS2 图

在笔记本本地执行：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

ros2 topic list | rg 'cloud_registered|aft_mapped_to_init|drone_0_plan_vis|occupancy_inflate'
```

预期至少能看到这些话题：

- `/cloud_registered`
- `/aft_mapped_to_init`
- `/drone_0_grid/grid_map/occupancy_inflate`
- `/drone_0_plan_vis/goal_point`
- `/drone_0_plan_vis/global_list`
- `/drone_0_plan_vis/init_list`
- `/drone_0_plan_vis/optimal_list`
- `/drone_0_plan_vis/a_star_list`

如果这一步看不到，先不要开 `RViz`，先解决 DDS 发现问题。

## 9. 笔记本本地开 RViz

在笔记本本地执行：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

rviz2 -d /path/to/ego_single_real.rviz
```

当前这份配置已经包含：

- `Fixed Frame: camera_init`
- `/cloud_registered`
- `/drone_0_grid/grid_map/occupancy_inflate`
- `/drone_0_plan_vis/goal_point`
- `/drone_0_plan_vis/global_list`
- `/drone_0_plan_vis/init_list`
- `/drone_0_plan_vis/optimal_list`
- `/drone_0_plan_vis/a_star_list`
- `/move_base_simple/goal`

另外，`Inflated Map` 现在已经按 `Z` 高度着色，不再是单一纯色。

## 10. 怎么从笔记本发目标点

最直接的方法是在笔记本本地 `RViz` 里用 `2D Goal` 工具。

当前配置已经把工具话题指向：

- `/move_base_simple/goal`

所以你在笔记本本地点下去，目标点会直接进机载机器上的 `ego_planner`。

如果想用命令行，也可以直接在笔记本本地发：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: world}, pose: {position: {x: 1.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}"
```

## 11. 笔记本上该怎么看效果

在笔记本本地 `RViz` 中，重点看：

- `Live Cloud`
  - 对应 `/cloud_registered`
  - 用来观察 FAST-LIVO 当前点云
- `Inflated Map`
  - 对应 `/drone_0_grid/grid_map/occupancy_inflate`
  - 用来观察规划器使用的膨胀障碍
- `Goal`
  - 对应 `/drone_0_plan_vis/goal_point`
- `Global Path`
  - 对应 `/drone_0_plan_vis/global_list`
- `Init Path`
  - 对应 `/drone_0_plan_vis/init_list`
- `Optimized Path`
  - 对应 `/drone_0_plan_vis/optimal_list`
- `Astar Path`
  - 对应 `/drone_0_plan_vis/a_star_list`

## 12. 常见问题

### 12.1 能 SSH，但笔记本上看不到 ROS2 话题

这通常不是 SSH 问题，而是 DDS 发现问题。优先检查：

- 两台机器是否同子网
- 是否有 AP Isolation
- 是否环境变量不一致
- 是否某一侧误设了 `ROS_LOCALHOST_ONLY=1`

### 12.2 笔记本能看到点云，但发目标点没反应

优先检查：

- `RViz` 中 `2D Goal` 是否确实发到 `/move_base_simple/goal`
- 机载侧 `ego_planner_node` 是否正在运行
- 机载侧是否已经有 `/aft_mapped_to_init`

### 12.3 笔记本想直接 `echo /drone_0_planning/pos_cmd`

这需要笔记本本地也具备 `quadrotor_msgs`。
如果只是看图，不需要先装这类自定义消息包。

## 13. 当前最推荐的使用方式

当前最推荐的工作流是：

1. 机载机器上运行全部传感器、定位、规划、PX4 bridge
2. 笔记本通过同一局域网直接加入 ROS2 图
3. 笔记本本地运行 `RViz`
4. 在笔记本上观察点云、膨胀障碍、规划轨迹
5. 在笔记本上直接发目标点

这条路比 `SSH X11` 更适合你现在这个实机调试阶段。
