# 双机 EGO-Swarm 部署与交接说明(drone 1 机上 agent 请读)

本仓库快照已完成多机化改造(见 `docs/superpowers/plans/ego-swarm-multi-drone-test-plan.md` 的背景分析)。
本文说明:已改了什么、drone 1 这台机需要做什么、两机怎么联调。

## 已完成的改造(两台机通用,代码无需再改)

1. **launch 全部按 `drone_id` 参数化**
   - `fast_lio2_mid360_bringup/mid360_fastlio2.launch.py`:输出 `/drone_{id}/livox/lidar|imu`、`/drone_{id}/aft_mapped_to_init`、`/drone_{id}/cloud_registered(_body)`、`/drone_{id}/path`
   - `ego_swarm_real_bringup/single_real.launch.py`:节点名 `drone_{id}_*`,规划话题 `/drone_{id}_planning/*`,fmu 话题 `/drone_{id}/fmu/in|out/*`
   - 机间共享话题:`/broadcast_bspline`(双向重映射已接好);`/drone_{i-1}_planning/swarm_trajs` 链由 ego_planner 按 drone_id 自动订阅
2. **leveling 节点支持共享世界系**:新参数 `world_origin_xyz`(米)与 `world_origin_yaw_rad`,在 `level_rpy_rad` 调平后叠加"先转 yaw 再平移"到共享 world 系;静态 TF 已同步
3. **脚本支持 `DRONE_ID` 环境变量**:`run_mid360_fastlio2.sh`、`run_ego_single_real.sh`、`run_px4_position_mode.sh`、`record_ego_swarm_experiment.sh`
4. **安全参数已加大**:`obstacles_inflation` 0.35→0.5、`dist0` 0.5→0.6、`swarm_clearance` 0.5→1.2(在 `ego_swarm_real_bringup/config/ego_planner_real.yaml`)
5. **雷达 30° 倾斜角补偿收拢到唯一位置**:只在 `fastlio2_to_ego_swarm_leveling` 的 `level_rpy_rad` 一处(world 侧调平 + body 侧姿态校正由同一参数自动取逆)。`fastlio2_to_px4_odometry` 的 `body_extrinsic_rpy_rad` 已清零、退化为纯 ENU→NED 轴映射;FAST-LIO 外参保持单位阵。**若 drone 1 安装角不同,只改 `level_rpy_rad` 一个参数**

## drone 1 这台机需要改/配置的(按顺序)

### A. 编译(和 drone 0 相同流程)

```bash
# 顺序:livox_ws → slam_ws → nav_ws → overlay_ws,均 colcon build --symlink-install
# slam_ws 构建前 source livox_ws;overlay_ws 构建前 source nav_ws(详见 CLAUDE.md)
```

### B. 机号差异配置(必须)

1. **Livox 雷达 IP**:`slam_ws/src/livox_ros_driver2/config/MID360_config.json` 改成本机雷达 IP(每台雷达 IP 不同,机身贴纸/QGC 可查)
2. **世界原点标定**(关键):两机摆好起飞位、机头同向后,量出 drone 1 相对 drone 0 起飞点的位置(drone 0 机体系:x 前 y 左 z 上),写入
   `overlay_ws/src/fastlio2_to_ego_swarm_leveling/config/fastlio2_to_ego_swarm_leveling.yaml`:
   ```yaml
   world_origin_xyz: [<dx>, <dy>, 0.0]   # 例如右侧 3 米: [0.0, -3.0, 0.0]
   world_origin_yaw_rad: 0.0             # 机头同向则为 0
   ```
   drone 0 保持全零。
3. **`level_rpy_rad`**:若 drone 1 的雷达安装角与 drone 0 相同(前倾 30°),保持 `[0, 0.5236, 0]` 不动;安装不同则单独标定
4. **航点/目标点**:`ego_planner_real.yaml` 的 `fsm/waypoint0_*` 是共享 world 系坐标,两机要设不同航点、航线间距 ≥ swarm_clearance。RViz 手动点目标已按机隔离:drone N 的 planner 只听 `/drone_N/move_base_simple/goal`(本机 RViz 配置的 2D Nav Goal 已指向 `/drone_0/...`;drone 1 的 RViz 配置需把 `drone_0` 批量替换为 `drone_1`)

### C. 飞控(PX4)配置(必须)

1. `MAV_SYS_ID = 2`(drone 0 为 1)
2. **XRCE-DDS 命名空间(用 `-n` 启动参数,不是改参数)**:
   - `UXRCE_DDS_DOM_ID` 是 DDS domain(保持 0,两机必须一致,别动);PX4 没有设置任意命名空间的参数,只能给 `uxrce_dds_client` 加 `-n` 启动参数。
   - 当前飞控配置(见 `mav.parm`):`UXRCE_DDS_CFG=102`(TELEM2)、波特率 2000000。
   - **确定 TELEM2 的设备路径**(status 不显示 device,板子间映射不同,一次性试出来):
     ```
     uxrce_dds_client stop
     uxrce_dds_client start -t serial -d /dev/ttyS2 -b 2000000 -n drone_1
     uxrce_dds_client status     # "Running, connected" 即正确
     ```
     不对就依次试 /dev/ttyS1、S3、S4(需 Jetson 侧 XRCE agent 已运行,否则永远 disconnected)。
     **drone 0 实测:TELEM2 = `/dev/ttyS3`**(连上会立刻打印一串 `successfully created rt/drone_N/fmu/...`);drone 1 同型号飞控大概率相同,直接先试 S3。
   - **持久化(每台机只配一次,开机自动执行)**:SD 卡根目录建 `etc/extras.txt`:
     ```
     uxrce_dds_client stop
     uxrce_dds_client start -t serial -d <上面试出的device> -b 2000000 -n drone_1
     ```
     多机部署时每张 SD 卡只有 `-n drone_N` 的机号不同。
   - **drone 0 同样操作,用 `-n drone_0`**,否则本机 offboard 链路不通(桥收不到 `/drone_0/fmu/out/vehicle_odometry` 会停在 WAIT 状态)。
   - 备注:PX4 v1.17+ 有 `UXRCE_DDS_NS_IDX` 参数(纯参数配置命名空间、免 SD 卡),但前缀固定为 `/uav_N` 且当前固件无此参数;若日后升级固件想用它,需同步把 ROS 侧 `/drone_N` 前缀改为 `/uav_N`。
3. 其余 EKF2/offboard/failsafe 参数从 drone 0 导出后导入

### D. 系统配置(必须)

1. **时间同步**:两机时差必须 <0.25 s(代码硬门限),目标 <10 ms。用仓库脚本:
   ```bash
   # drone 0(server):
   sudo ./scripts/setup_chrony_time_sync.sh server
   # drone 1(client,填 drone 0 的 WiFi IP):
   sudo ./scripts/setup_chrony_time_sync.sh client <drone0_ip>
   # 两边验证:
   chronyc tracking     # System time 偏差应 <10ms
   ```
2. **DDS**:两机同一 WiFi、相同 `ROS_DOMAIN_ID`(脚本已统一 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`)

## 启动方式(两台机各自执行,替换机号)

```bash
# drone 1:
DRONE_ID=1 ./scripts/run_mid360_fastlio2.sh          # 终端1: SLAM
DRONE_ID=1 ./scripts/run_ego_single_real.sh          # 终端2: 规划+桥(默认 fastlio2_leveled)
sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 2000000   # 终端3
```

## 地面站:双机显示与航点下发

笔记本(同 WiFi、同 `ROS_DOMAIN_ID`、`export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`)上:

```bash
rviz2 -d overlay_ws/src/ego_swarm_real_bringup/rviz/ego_swarm_ground_station.rviz
```

- 双机同图:drone 0 蓝色、drone 1 橙色(各自点云、里程计箭头、目标点、优化轨迹),共享 `camera_init_level` 世界系;
- **航点下发**:工具栏有两个 "2D Goal Pose" 按钮,**第一个发给 drone 0、第二个发给 drone 1**(在 Tool Properties 面板可核对各自话题 `/drone_N/move_base_simple/goal`)。点击地图即让对应机飞到该点(固定 1 m 高);
- 单机调试仍可用 `ego_single_real.rviz`(仅 drone 0 视角);
- **注意**:点目标 = 下达飞行命令(桥自动 offboard+解锁),地面验证请不装桨。

## 地面联调检查单(起飞前,两机通电不解锁)

- [ ] `ros2 topic list | grep drone_1` 在 drone 0 上能看到(DDS 互通)
- [ ] `ros2 topic echo /broadcast_bspline` 两机各自能看到对方 drone_id 的消息
- [ ] planner 日志无 `Time difference is too large`(时间同步 OK)
- [ ] RViz(共享系):两机 `/drone_N/aft_mapped_to_init_level` 的相对位置与实际摆放一致(验证 world_origin 标定)
- [ ] `ros2 topic hz /drone_1/fmu/out/vehicle_odometry` 有数据(PX4 命名空间 OK)
- [ ] 桥状态机能走到 STREAM(各自遥控器可随时接管)

## 已知未改项(留意)

- GVF/ISMC 栈(`gvf_ismc_*`)仍引用旧话题名(无 `/drone_N` 前缀),当前不可用;GVF 工作恢复时需要同样参数化
- 旧 runbook(`ego-swarm-single-real-*.md` 等)中话题名是改造前的,以本文档和代码为准
- `record_gvf_ismc_experiment.sh` / `replay_gvf_ismc_bag.sh` 未更新(GVF 专用)
- RViz 配置 `ego_single_real.rviz` 已更新为 drone_0 视角;drone 1 上如需 RViz,把其中 `drone_0` 批量替换为 `drone_1`

## 渐进测试流程

1. 地面联调(上面检查单)
2. drone 0 单飞航点,drone 1 地面通电广播 —— 验证 drone 0 绕开 drone 1
3. 双机同飞:大间距平行航线 → 收窄 → 交叉;全程录包 `DRONE_IDS="0 1" ./scripts/record_ego_swarm_experiment.sh`
