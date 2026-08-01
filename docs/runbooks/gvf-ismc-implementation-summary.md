# GVF + ISMC 实现总结

更新时间：`2026-05-08`

## 1. 本次实现做了什么

本次工作在当前仓库内新增了一条完整的 `GVF + ISMC` 实机控制链，并复用了已有的：

- `FAST-LIO2`
- `fastlio2_to_ego_swarm_leveling`
- `fastlio2_to_px4_odometry`
- `position_cmd_to_px4_bridge`
- PX4 Offboard 输入链路

实现后的总体数据流为：

1. `FAST-LIO2 -> /aft_mapped_to_init`
2. `/aft_mapped_to_init -> fastlio2_to_ego_swarm_leveling -> /aft_mapped_to_init_level`
3. `/aft_mapped_to_init_level -> gvf_reference_node -> /gvf/reference`
4. `/gvf/reference + /aft_mapped_to_init_level -> ismc_velocity_tracker_node -> /drone_0_planning/pos_cmd`
5. `/drone_0_planning/pos_cmd -> position_cmd_to_px4_bridge -> /fmu/in/*`

## 2. 主要新增/修改内容

### 2.1 `position_cmd_to_px4_bridge`

为支持向量场实飞控制链，bridge 增加了显式 Offboard 控制模式配置能力：

- 新增 `position_control_enabled`
- 新增 `velocity_control_enabled`
- 保留 `use_acceleration_feedforward`

同时补充了：

- `OffboardControlMode` helper
- 配套 gtest
- 配置合同测试
- 非法配置兜底

### 2.2 `gvf_path_following_msgs`

新增接口包：

- `overlay_ws/src/gvf_path_following_msgs`

新增消息：

- `GVFReference.msg`

字段包括：

- `desired_velocity`
- `desired_yaw`
- `desired_yaw_rate`
- `phi1`
- `phi2`

### 2.3 `gvf_ismc_path_following`

新增核心控制包：

- `overlay_ws/src/gvf_ismc_path_following`

其中包含：

- 三叶草隐式曲线 GVF 核心
- ISMC 外环核心
- `gvf_reference_node`
- `ismc_velocity_tracker_node`
- 参数配置
- 单元测试和合同测试

已实现的关键功能：

- `phi1 = z - h`
- `phi2 = r - (r0 + a cos(3 theta))`
- 基于 `phi2` 梯度构造真实三叶曲线切向场
- 横向/纵向修正项
- 速度幅值限制
- 低平面速度时保持上一拍 yaw
- `yaw_alpha` 平滑
- ISMC 外环加速度输出
- 首拍导数尖峰抑制
- stale input freshness gating
- tracker 参数显式暴露到 YAML

### 2.4 `gvf_ismc_real_bringup`

新增 bringup 包：

- `overlay_ws/src/gvf_ismc_real_bringup`

新增：

- `launch/single_real.launch.py`
- launch 合同测试
- script 合同测试
- `scripts/run_gvf_ismc_single_real.sh`

关键点：

- 只启动 5 个核心节点
- 统一使用 `/aft_mapped_to_init_level`
- bridge 走 velocity-dominant 模式
- `use_acceleration_feedforward` 可通过 launch/script 开关控制

### 2.5 文档

新增文档：

- [gvf-ismc-single-real.md](/home/morphing01/Drone_SLAM/.worktrees/gvf-ismc-path-following/docs/runbooks/gvf-ismc-single-real.md)
- 本文档 [gvf-ismc-implementation-summary.md](/home/morphing01/Drone_SLAM/.worktrees/gvf-ismc-path-following/docs/runbooks/gvf-ismc-implementation-summary.md)

## 3. 主要功能总结

本次实现后，仓库具备以下能力：

1. 在 ROS 2 中生成三叶草向量场参考 `GVFReference`
2. 用 ISMC 外环跟踪 `desired_velocity`
3. 输出 `PositionCommand` 给 PX4 bridge
4. 通过单独 bringup 包启动整条 `GVF + ISMC` 控制链
5. 对控制链关键几何、参数、安装布局、launch wiring 做自动化合同测试

## 4. 建议检查顺序

### 4.1 代码与构建

先确认以下包都已构建：

- `position_cmd_to_px4_bridge`
- `gvf_path_following_msgs`
- `gvf_ismc_path_following`
- `gvf_ismc_real_bringup`

### 4.2 单元/合同测试

推荐检查顺序：

1. `position_cmd_to_px4_bridge`
2. `gvf_path_following_msgs`
3. `gvf_ismc_path_following`
4. `gvf_ismc_real_bringup`

### 4.3 launch smoke test

先跑不接实机数据的 smoke test，确认：

- `gvf_reference_node`
- `ismc_velocity_tracker_node`
- `position_cmd_to_px4_bridge_node`

都能被 `ros2 launch` 正确拉起。

### 4.4 实机联调顺序

建议顺序：

1. XRCE Agent
2. FAST-LIO2
3. GVF + ISMC bringup
4. 查看 `/aft_mapped_to_init_level`
5. 查看 `/gvf/reference`
6. 查看 `/drone_0_planning/pos_cmd`
7. 查看 bridge 是否进入 `ACTIVE`
8. 再进入人工起飞和首次接管

## 5. 当前验证结果

截至 `2026-05-08`，当前 worktree 内已完成的验证包括：

### 5.1 包测试

- `position_cmd_to_px4_bridge`: `20 tests, 0 failures`
- `gvf_path_following_msgs`: `1 test, 0 failures`
- `gvf_ismc_path_following`: `21 tests, 0 failures`
- `gvf_ismc_real_bringup`: `7 tests, 0 failures`

### 5.2 launch smoke test

使用：

```bash
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/.worktrees/gvf-ismc-path-following/overlay_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
timeout 10 ros2 launch gvf_ismc_real_bringup single_real.launch.py \
  use_acceleration_feedforward:=false
```

结果：

- 5 个核心进程均成功启动
- `timeout` 返回 `124`
- 这是预期结果，说明 launch 能正常起来并持续运行

## 6. 与 MATLAB 代码的一致性检查

MATLAB 参考文件：

- [DDSMC_three_leaf.m](/home/morphing01/Drone_SLAM/GVF_ws/DDSMC_three_leaf.m:1)

### 6.1 已保持一致的部分

当前实现与 MATLAB 在以下层面是一致的：

1. 都使用三叶草隐式曲线
   - MATLAB: `phi2 = r - (r0 + a*cos(3*theta))`
   - 当前实现同样使用这一形式

2. 都使用高度约束
   - MATLAB 原始写法是 `phi1 = z`
   - 当前实飞实现使用的是固定高度版本 `phi1 = z - h`
   - 这是我们设计阶段明确做过的实飞适配，不是误差

3. 都从 `v_d` 推导参考偏航
   - MATLAB: `psi_raw = atan2(v_d(2), v_d(1))`
   - 当前实现同样基于 `desired_velocity` 的平面方向算 yaw

4. 都有 yaw 平滑/保持逻辑
   - MATLAB 有 `alpha` 平滑
   - 当前实现有 `yaw_alpha`

### 6.2 不完全一致的部分

当前实现 `不是` 对 MATLAB 全控制律的逐行等价移植，主要差异如下：

1. GVF 切向项实现方式不同
   - MATLAB 使用 `cross(n1, n2)` 直接构造切向速度
   - 当前实现先求 `phi2` 平面梯度，再取其正交方向作为平面切向
   - 在几何意义上，这是与隐式曲线相容的等价实现思路，不是圆切线近似；但实现形式和 MATLAB 不完全逐行一致

2. 实机 ISMC 外环没有完整照搬 MATLAB 外环公式
   - MATLAB 外环核心是：
     - `e_v = v_d - v_actual`
     - `e_dot_v = (v_d - v_actual)/dt`
     - `s1 = e_dot_v + mu1*e_v`
     - `P1_hat` 自适应更新
     - `a_cmd = C1*e_dot_v + K_xi*mu1*e_v + k1*s1 + (lambda1+epsilon).*tanh(s1) + [0;0;m*g] + P1_hat`
   - 当前 ROS 实现的 `ismc_outer_loop` 是一个简化版外环：
     - 保留了速度误差、误差导数、滑模面、自适应 bias、加速度饱和
     - 但参数结构、项的组合形式、重力补偿、矩阵增益形式，并没有完整按 MATLAB 外环逐项重建

3. 当前实机没有实现 MATLAB 的内环姿态/力矩控制
   - MATLAB 中：
     - 外环给 `a_cmd`
     - 再解出 `phi_d / theta_d / psi_d`
     - 再通过内环 `tau` 控姿态
   - 当前工程里：
     - `ismc_velocity_tracker_node` 只输出 `PositionCommand`
     - PX4 仍然承担更底层姿态与推力闭环

4. 当前 ROS 实现是“论文方法的工程化外环版本”
   - 它保留了：
     - 三叶草 GVF
     - 速度跟踪外环
     - 偏航参考生成
   - 它没有完整复刻：
     - MATLAB 里的完整 `DDSMC/ISMC` 双环控制结构

### 6.3 最终结论

结论要说得严格一点：

- 当前仓库实现与 MATLAB 的 `GVF` 路径表达和上层参考生成逻辑 `基本一致`
- 与 MATLAB 的完整 `DDSMC_three_leaf.m` `不完全一致`
- 差异主要在：
  - 实机外环控制律做了工程化简化
  - MATLAB 的姿态内环没有搬到 ROS/PX4 中
  - PX4 继续承担了低层控制

所以如果你问“是否和 MATLAB 完全一致”，答案是：

- `不是完全一致`
- `是按我们前面确认过的论文实飞路线，保留 GVF + 外环跟踪思想，并对实机链路做了可飞化改造`

如果论文里要写严谨，建议表述成：

- 仿真验证：完整 `GVF + DDSMC/ISMC` 控制框架
- 实飞验证：`GVF` 引导下的外环跟踪策略在真实平台上的工程实现与可行性验证

而不要写成：

- “MATLAB 中完整内外环控制律已被 1:1 搬到实机”

## 7. 当前分支建议

当前 `gvf-ismc-path-following` 工作树已经具备：

- 代码实现
- 包级测试
- bringup 合同测试
- smoke test
- runbook
- 实现总结文档

已经适合合并回 `main`。
