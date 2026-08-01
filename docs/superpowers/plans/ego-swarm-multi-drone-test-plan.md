# EGO-Swarm 多机(双机)飞行测试准备计划

状态:计划文档(2026-07-24),尚未实施代码改动。
前提:单机 EGO-Swarm(MID360 + FAST-LIO2 + ego_planner + PX4 offboard)航点飞行与避障已实测可用;GVF/ISMC 工作暂停。

本文回答四个核心问题:

1. 多机测试需要准备什么?
2. 第二台无人机直接跑一份和当前 ws 一样的代码可以吗?
3. 两机之间需要通信哪些数据?
4. 各机 SLAM 坐标系怎么统一?飞控(PX4)需不需要改?

---

## 1. 机间通信机制(代码分析结论)

EGO-Swarm 的多机协同**只需要互传轨迹,不需要互传点云、地图或里程计**。

| 话题 | 消息类型 | 方向 | 作用 |
|------|----------|------|------|
| `/broadcast_bspline` | `traj_utils/msg/Bspline` | 全体互发 | 每机每次重规划后广播自己的 B 样条轨迹(含 `drone_id`);其他机将其作为动态约束做互避障 |
| `/drone_{i-1}_planning/swarm_trajs` | `traj_utils/msg/MultiBsplines` | drone i 订阅 drone i-1 | 链式初始化:后启动的机一次性获取前面所有机的当前轨迹 |

代码出处(`nav_ws/src/ego-swarm-ros2/planner/ego_planner/src/ego_replan_fsm.cpp`):

- 76–112 行:`drone_id >= 1` 时订阅 `/drone_{id-1}_planning/swarm_trajs`;发布/订阅 `planning/broadcast_bspline_from_planner` / `to_planner`(官方 `swarm.launch.py` → `advanced_param.launch.py:99-100` 把两者都重映射到共享的 `/broadcast_bspline`)。
- 272–330 行 `BroadcastBsplineCallback`:
  - 按 `msg->drone_id` 过滤自己的消息;
  - **时间门限:两机系统时间差 > 0.25 s 直接丢弃**(`ego_replan_fsm.cpp:284`)——多机必须做时间同步;
  - 距离门限:对方轨迹起点距本机 > 4/3 × `planning_horizon` 则忽略。

互避障安全距离参数:`optimization/swarm_clearance`(`ego_planner_real.yaml:46`,当前 0.5 m)。

**关键隐含假设:广播的轨迹坐标是"共享世界系"下的坐标** —— 这就是坐标系统一问题的根源(见第 4 节)。

## 2. 第二台无人机的部署方式

**可以克隆同一个仓库、按同样流程编译**(livox_ws → slam_ws → nav_ws → overlay_ws),但**不能原样启动**,因为当前 bringup 是单机硬编码的。每机需要一份"机号差异配置":

| 差异项 | 位置 | drone 0 | drone 1 |
|--------|------|---------|---------|
| `manager/drone_id` | `ego_planner_real.yaml:40` | 0 | 1 |
| 节点名/话题前缀 | `single_real.launch.py`(硬编码 `drone_0_*`) | drone_0_* | drone_1_* |
| broadcast 重映射 | 目前**缺失** | → `/broadcast_bspline` | → `/broadcast_bspline` |
| Livox 雷达 IP | MID360 config | 各自雷达 IP | 各自雷达 IP |
| 世界原点偏移 | leveling 节点(**需新增参数**) | 0 | 实测偏移 |
| PX4 `MAV_SYS_ID` | 飞控参数 | 1 | 2 |

### 话题串台风险(必须处理)

两台 Jetson 在同一 WiFi、同一 `ROS_DOMAIN_ID` 下,以下无命名空间话题会互相串台:

- `/fmu/in/*`、`/fmu/out/*`(**最危险**:drone 1 的桥可能吃到 drone 0 的 vehicle_odometry,或给对方飞控发 setpoint);
- `/aft_mapped_to_init(_level)`、`/cloud_registered(_level)`、livox 原始话题;
- `/drone_0_planning/pos_cmd`(若两机都叫 drone_0)。

**方案:每机所有本机话题加命名空间或 drone_id 前缀**(`/drone_0/fmu/...`、`/drone_0/aft_mapped_to_init` 等),只有 `/broadcast_bspline` 和 `swarm_trajs` 链保持全局共享。备选:PX4 侧设 `UXRCE_DDS` namespace(agent/固件参数)让 fmu 话题变 `/px4_N/fmu/*` —— 若 ROS 侧已做命名空间则不必改 PX4。

## 3. 网络与时间同步准备

- 两机 + 地面站(RViz/QGC)同一 WiFi;统一 `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` 和相同 `ROS_DOMAIN_ID`。
- 带宽:`/broadcast_bspline` 每机每次重规划一条消息(几十个控制点),量很小;**不要**让点云话题跨机传输(命名空间隔离后 RViz 按需订阅时注意带宽)。
- **时间同步(硬性要求,门限 0.25 s)**:用 chrony,drone 0 或地面站做 NTP server,两机作 client;验证 `chronyc tracking` 偏差 < 10 ms。
- 地面验证:`ros2 topic echo /broadcast_bspline` 在两机上都能看到对方的 `drone_id`。

## 4. SLAM 坐标系统一(核心问题)

每机 FAST-LIO2 的原点 = 各自上电时刻的位姿,两机的 `camera_init_level` 系互不相同;但 `/broadcast_bspline` 的轨迹坐标被当作同一世界系使用。**必须把两机的 SLAM 输出对齐到一个共享 world 系。**

首飞推荐最简方案(静态外参标定,不做在线地图配准):

1. 两机摆放在**测量好的起飞位**,机头**同向**(yaw 一致,建议用同一参照物对齐);
2. 给 `fastlio2_to_ego_swarm_leveling` 新增参数 `world_origin_xyz`(平移)与 `world_origin_yaw`(偏航),在现有 `level_rpy_rad` 调平之后再叠加该变换,输出即共享 world 系;
3. drone 0 原点作为全局原点(偏移全 0);drone 1 填入实测的相对位置(如 [0, 3.0, 0])与 0 yaw;
4. 注意:该偏移同时作用于 odom 和点云输出,下游 planner、PX4 odom 桥、pos_cmd 全链路自动一致,PX4 桥的 `world_axis`/`world_sign` 不用改。

风险:SLAM 漂移会让共享系逐渐失准 —— `swarm_clearance` 要在标定误差 + 双机漂移之上留足余量;首飞控制在短时间、短航线。

进阶备选(不在本次范围):起飞前互看对方定位求解外参、或基于点云地图配准的在线对齐。

## 5. 飞控(PX4)检查单

固件与飞行逻辑**不需要修改**,仅参数/接线检查:

- [ ] 每机 `MAV_SYS_ID` 唯一(1 / 2),QGC 可同时区分两机;
- [ ] 每机 XRCE-DDS 仍走各自本机串口 agent(`scripts/micro_dds.sh`),agent 只在本机跑;
- [ ] fmu 话题隔离方案二选一:ROS 侧命名空间(推荐)或 PX4 `UXRCE_DDS` namespace;
- [ ] 单机已验证的 EKF2 外部里程计配置、offboard/failsafe 参数原样复制到第二台;
- [ ] 遥控器/安全开关:两机各自独立的手动接管手段。

## 6. 需要的代码/配置改动清单(后续实施)

1. `overlay_ws/src/ego_swarm_real_bringup`:launch 增加 `drone_id` 参数(或新增 `swarm_real.launch.py`),按 drone_id 生成节点名/话题前缀,并把 `planning/broadcast_bspline_from_planner`/`to_planner` 重映射到 `/broadcast_bspline`;drone_id ≥ 1 时接好 `swarm_trajs` 链。
2. `overlay_ws/src/fastlio2_to_ego_swarm_leveling`:新增 `world_origin_xyz` / `world_origin_yaw` 参数及变换实现 + gtest。
3. `position_cmd_to_px4_bridge` / `fastlio2_to_px4_odometry` 配置:话题名带 drone 前缀(随 launch 传参)。
4. `scripts/run_ego_single_real.sh`:接受 `DRONE_ID` 环境变量。
5. 更新相应契约测试(launch/config contract pytest)。

## 7. 参数调整

| 参数 | 当前 | 建议 | 说明 |
|------|------|------|------|
| `optimization/swarm_clearance` | 0.5 | ≥ 1.0–1.5 | 机间安全距离,含标定误差+漂移余量 |
| `grid_map/obstacles_inflation` | 0.35 | 按单机经验加大 | 用户反馈单机安全距离需加大 |
| `optimization/dist0` | 0.5 | 随 inflation 同步评估 | 障碍物斥力起始距离 |

## 8. 渐进测试流程

1. **地面联调**:两机通电不起飞 —— 验证 DDS 互通、chrony 偏差、`/broadcast_bspline` 互见且无 "Time difference is too large" 报错、RViz 中两机 odom 落在共享系的正确相对位置。
2. **单飞单广播**:drone 0 飞航点,drone 1 地面通电广播(其"轨迹"为静止点)——验证 drone 0 的规划绕开 drone 1 位置。
3. **双机同飞**:先大间距平行航线,再逐步收窄、最后交叉航线;全程 RViz 监控双机轨迹与 swarm_clearance。
4. 每次飞行用 `record_gvf_ismc_experiment.sh` 同类方式录 bag(需扩充话题集到双机)。

## 9. 风险与未决问题

- SLAM 漂移导致共享系失准(缓解:短航线、大 clearance);
- WiFi 丢包/延迟导致对方轨迹过期(代码自带 0.25 s 时间门限会丢弃过期轨迹,退化为"看不见对方"——clearance 余量兜底);
- `swarm_trajs` 链式初始化对启动顺序有要求(drone 1 需在 drone 0 之后启动规划);
- 两机 `level_rpy_rad`(雷达安装角)是否一致需各自标定确认。
