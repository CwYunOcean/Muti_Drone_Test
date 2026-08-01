# EGO-Swarm 直接 ROS 指令发布目标点

适用场景: 不通过 RViz 点击, 直接用 `ros2 topic pub` 给 EGO-Swarm 发布手动目标点。

重要限制:

- 发布目标点就是给对应无人机下达 EGO 规划目标, 不是普通通信测试。
- 真机测试前必须确认拆桨、定位稳定、急停/遥控接管可用。
- 当前代码只使用目标点的 `x/y`; EGO 回调里将目标高度固定为 `z=1.0 m`。消息里的 `z` 只用于过滤, 保持 `z >= -0.1` 即可。
- `orientation` 当前基本不参与目标位置规划, 填 `{w: 1.0}` 即可。

## 1. 终端环境

在机载电脑或笔记本上先 source ROS 和工作空间:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Drone_SLAM/nav_ws/install/setup.bash
source ~/Drone_SLAM/overlay_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

如果是在笔记本上发送, 还要保证笔记本和机载电脑 DDS 能互相发现, 并且 topic list 里能看到目标机的 EGO 话题。

## 2. 发布 drone_1 目标点

当前 drone_1 的目标点 topic:

```text
/drone_1/move_base_simple/goal
```

示例: 发送一个相对共享世界系 `camera_init_level` 下的 `(x=1.0, y=0.0)` 目标。

```bash
ros2 topic pub --times 3 --rate 1 /drone_1/move_base_simple/goal geometry_msgs/msg/PoseStamped "{header: {frame_id: 'camera_init_level'}, pose: {position: {x: 1.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}"
```

更小的首次测试目标可以改成:

```bash
ros2 topic pub --times 3 --rate 1 /drone_1/move_base_simple/goal geometry_msgs/msg/PoseStamped "{header: {frame_id: 'camera_init_level'}, pose: {position: {x: 0.5, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}"
```

## 3. 发布 drone_0 目标点

双机地面站要给 drone_0 发目标时, topic 换成:

```text
/drone_0/move_base_simple/goal
```

示例:

```bash
ros2 topic pub --times 3 --rate 1 /drone_0/move_base_simple/goal geometry_msgs/msg/PoseStamped "{header: {frame_id: 'camera_init_level'}, pose: {position: {x: 1.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}"
```

## 4. 发送前检查

确认 EGO 正在订阅目标点:

```bash
ros2 topic info -v /drone_1/move_base_simple/goal
```

正常应看到 `Subscription count` 大于 0, 并且订阅节点是 drone_1 的 `ego_planner_node`。

确认 EGO 已有 odom:

```bash
ros2 topic hz /drone_1/aft_mapped_to_init_level
```

确认规划输出 topic 存在:

```bash
ros2 topic list | grep drone_1_planning
```

## 5. 发送后检查

看目标 marker 是否刷新:

```bash
ros2 topic echo --once /drone_1_plan_vis/goal_point
```

看 EGO 是否生成 position command:

```bash
ros2 topic echo /drone_1_planning/pos_cmd
```

如果只做地面拆桨验证, 看到 `goal_point` 和 `/drone_1_planning/pos_cmd` 刷新即可, 不需要 arm, 不需要切 Offboard。

## 6. 常见问题

如果发了目标点没有反应:

- 检查 topic 是否发错, 当前 drone_1 应是 `/drone_1/move_base_simple/goal`。
- 检查 `ros2 topic info -v /drone_1/move_base_simple/goal` 是否有订阅者。
- 检查 EGO 是否还在等 odom, 即 `/drone_1/aft_mapped_to_init_level` 是否连续发布。
- 检查 DDS 环境变量是否一致: `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=0`, `ROS_LOCALHOST_ONLY=0`。
- 如果在笔记本发送, 先确认笔记本能看到机载电脑发布的 `/drone_1/...` topic。
