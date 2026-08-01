# EGO-Swarm 单机实机规划风险说明

更新时间：`2026-04-24`

## 1. 这份说明解决什么问题

这份说明专门回答一个现实问题：

- 现在 `ego_planner` 在 `RViz` 里画出来的红色轨迹会随时间向前“缩短”
- 实际无人机如果没有跟上这条轨迹，规划器是否会按真实机体位置及时重规划
- 在真实障碍环境里，这套链路现在到底能信到什么程度

这份说明不是参数手册，而是“当前阶段能飞什么、不能信什么”的边界说明。

## 2. 当前源码行为结论

当前这份 `ego_planner` 的核心执行逻辑是：

- 规划器生成一条**带时间参数**的局部轨迹
- `traj_server` 按当前时间对轨迹采样，持续输出 `PositionCommand`
- `position_cmd_to_px4_bridge` 把 `PositionCommand` 送给 PX4
- PX4 作为低层控制器去跟踪参考轨迹

关键点是：

- `traj_server` 是按轨迹时间采样，不是按真实机体位置“拖着参考点走”
- `ego_planner` 的 `EXEC_TRAJ` 收尾和阶段推进，也主要依赖轨迹时间
- 到轨迹末尾后，规划器会回到 `WAIT_TARGET`

本地源码证据：

- [traj_server.cpp](/home/morphing01/Drone_SLAM/nav_ws/src/ego-swarm-ros2/planner/ego_planner/src/traj_server.cpp)
- [ego_replan_fsm.cpp#L571](/home/morphing01/Drone_SLAM/nav_ws/src/ego-swarm-ros2/planner/ego_planner/src/ego_replan_fsm.cpp#L571)

所以你在 `RViz` 里看到“红线自己慢慢缩短”，这不是显示 bug，而是当前上游设计本身的表现。

## 3. 这意味着什么

这套系统默认有一个前提：

- 下游控制器必须基本能跟住规划轨迹

也就是说，上游 `ego_planner` 的思路更像：

- 先规划一条未来几秒的参考轨迹
- 然后相信控制器能够跟踪
- 如果周期性重规划、碰撞检查或局部目标推进触发，再更新轨迹

它**不是**一套“强 tracking-error 闭环”的规划器。
现在本地代码里，没有把“真实机体明显落后于参考轨迹”作为第一优先级的强制重规划触发条件。

## 4. 你现在最担心的风险是什么

你担心的是：

- 如果无人机没有跟上红线
- 但轨迹时间仍然在往前走
- 那规划器会不会误以为机体已经接近轨迹末端或已经完成这一段任务
- 最终导致没有按真实位置及时避障

这个担心是合理的。

当前阶段，不能把“红线在更新”直接等价成“真实机体一定安全”。

## 5. GitHub 上游有没有类似反馈

有，而且方向上是吻合的。

比较相关的 issue：

- `ego-planner #106`
  `Not able to fly real system with ego-planner`
  用户直接描述了真实系统里机体与轨迹/可视化表现不一致的问题。
  https://github.com/ZJU-FAST-Lab/ego-planner/issues/106

- `ego-planner #122`
  `规划轨迹重规划问题`
  日志里已经出现：
  - `traj failed`
  - `the drone is in obstacle`
  - `terminal point of the current trajectory is in obstacle`
  https://github.com/ZJU-FAST-Lab/ego-planner/issues/122

- `ego-planner #129`
  `避障功能似乎没有实现`
  同样出现了终点落进障碍和进度异常的现象。
  https://github.com/ZJU-FAST-Lab/ego-planner/issues/129

- `ego-planner-swarm #3`
  讨论的是执行上一条轨迹时，如何触发 replanning。
  https://github.com/ZJU-FAST-Lab/ego-planner-swarm/issues/3

这些 issue 不能直接证明你的系统就一定会撞，但至少说明：

- “实机/半实机下轨迹与真实机体不完全一致”
- “重规划和避障在现实系统中可能不如 RViz 看起来那么可靠”

这些担心不是孤例。

## 6. 当前阶段可以相信什么

你现在可以相对相信这些：

- `FAST-LIVO -> ego_planner` 数据链路已经打通
- 发布目标点后，`ego_planner` 能正常进入 `GEN_NEW_TRAJ / EXEC_TRAJ`
- `traj_server` 能稳定输出 `PositionCommand`
- `position_cmd_to_px4_bridge` 能进入 `ACTIVE`
- `/fmu/in/offboard_control_mode` 和 `/fmu/in/trajectory_setpoint` 在持续发

也就是说，**导航参考链路是通的**。

## 7. 当前阶段不能直接相信什么

你现在**不能直接把这些当成已证明成立**：

- “只要看见红线就说明真实机体已经按红线安全飞行”
- “只要轨迹在自动更新，就说明避障一定可靠”
- “只要规划成功，真实机体跟不上时也一定会自动停住或自动修正”

更直接一点说：

- 当前这套系统已经适合做**数据链路验证**
- 也适合做**低速、低高度、空旷环境**的目标点跟踪验证
- 但**还不适合直接把‘贴障碍安全飞行’当成已验证结论**

## 8. 当前最合适的测试边界

现阶段建议把测试范围压在这里：

- 空旷环境
- 低高度
- 低速度
- 低加速度
- 近距离目标点
- 先验证“能否稳定接管并跟踪”
- 不把“复杂避障”作为第一轮目标

### 当前不建议做的

- 首飞就贴近障碍物飞
- 首飞就让它穿障碍缝隙
- 首飞就连续发布远距离目标点
- 首飞就把 `RViz` 里的红线当作真实安全边界

## 9. 如果后面要把它变得更安全

工程上更稳的改法应该是：

- 增加真实 tracking error 监控
  例如比较 `||odom_pos - reference_pos||`
- 当跟踪误差过大时，强制：
  - `REPLAN`
  - 或 `hover`
  - 或直接切回人工接管
- 不再只靠轨迹时间作为任务完成的主要依据

这部分当前还没做，所以现在不能把“避障安全性”说得太满。

## 10. 一句话结论

当前这套 `ego + FAST-LIVO + PX4` 链路已经足够支持：

- 目标点导航链路验证
- 低风险实飞接管验证

但还**不能仅凭 RViz 红线**就认定：

- 跟踪落后时一定会自动纠正
- 复杂障碍环境里一定可靠避障

所以当前阶段最正确的心态是：

- 可以飞
- 但要低风险飞
- 先验证“跟踪”
- 暂时不要把“避障安全”当成已完成证明
