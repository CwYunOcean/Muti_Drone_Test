# Manual Offboard Switching Design

更新时间：`2026-05-08`

## 1. 目标

把当前 `GVF + ISMC + PX4 bridge` 实机链路改成：

- bridge 持续发送 `OffboardControlMode` 和 `TrajectorySetpoint`
- bridge 不再默认自动请求 `Offboard` 和 `Arm`
- 是否切入 `Offboard` 由操作者通过遥控器或 QGC 手动控制

本次改动不改变：

- FAST-LIO2 定位链路
- GVF/ISMC 控制律
- PX4 里程计注入链路

## 2. 现状

当前 [bridge_state_machine.cpp](/home/morphing01/Drone_SLAM/overlay_ws/src/position_cmd_to_px4_bridge/src/bridge_state_machine.cpp:1) 在 warmup setpoint 后会自动进入 `ACTIVE`，并请求：

- `request_offboard`
- `request_arm`

随后 [position_cmd_to_px4_bridge_node.cpp](/home/morphing01/Drone_SLAM/overlay_ws/src/position_cmd_to_px4_bridge/src/position_cmd_to_px4_bridge_node.cpp:329) 会发：

- `VEHICLE_CMD_DO_SET_MODE`
- `VEHICLE_CMD_COMPONENT_ARM_DISARM`

这和实飞安全要求冲突，因为用户希望：

- 程序只负责准备好参考和 setpoint
- 是否接管由用户自己决定

## 3. 方案选型

### 方案 A：直接删除自动请求逻辑

优点：

- 实现最直接

缺点：

- 破坏兼容性
- 其他依赖自动接管的流程也一起被改死

### 方案 B：增加参数开关

例如：

- `auto_request_offboard`
- `auto_request_arm`

或单个总开关：

- `auto_request_offboard_and_arm`

优点：

- 保留兼容性
- `gvf_ismc_real_bringup` 可以默认关掉自动接管
- 其他流程以后如有需要可以显式打开

缺点：

- 比硬删多一点配置复杂度

推荐采用方案 B，并用单个总开关：

- `auto_request_offboard_and_arm`

## 4. 设计

### 4.1 bridge 参数

在 `position_cmd_to_px4_bridge` 中新增参数：

- `auto_request_offboard_and_arm`

默认值：

- 包默认配置先保持 `true`

这样可以最小化对其他潜在使用者的破坏。

### 4.2 状态机行为

状态机保留：

- `WAIT_FASTLIVO`
- `WAIT_PX4`
- `STREAM_SETPOINT`
- `ACTIVE`
- `FAILSAFE`

但在自动接管关闭时：

- 继续 `stream_setpoint`
- 不再输出 `request_offboard`
- 不再输出 `request_arm`

`ACTIVE` 的进入条件改成：

- 自动接管开启时：沿用当前行为
- 自动接管关闭时：仅当 PX4 实际已经处于 `Offboard` 且已 `ARM` 后进入 `ACTIVE`

这样状态机会正确反映：

- 用户未切 Offboard 前：只是准备中和持续送 setpoint
- 用户手动切入 Offboard 且机体已 armed 后：进入 `ACTIVE`

### 4.3 `gvf_ismc_real_bringup` 默认行为

在 [single_real.launch.py](/home/morphing01/Drone_SLAM/overlay_ws/src/gvf_ismc_real_bringup/launch/single_real.launch.py:1) 给 `position_cmd_to_px4_bridge` 增加 override：

- `auto_request_offboard_and_arm: False`

这样 `run_gvf_ismc_single_real.sh` 启动后：

- 仍然持续推 setpoint
- 但不会自己切模式或解锁

### 4.4 文档行为

runbook 要明确改成：

1. 启动 XRCE / FAST-LIO2 / GVF chain
2. 确认 `/aft_mapped_to_init_level`、`/gvf/reference`、`/drone_0_planning/pos_cmd` 正常
3. 人工 `Position` 模式起飞
4. 操作者通过遥控器或 QGC 手动切入 `Offboard`
5. 进入 `Offboard` 后开始由控制链接管

## 5. 影响范围

预计修改：

- [bridge_state_machine.hpp](/home/morphing01/Drone_SLAM/overlay_ws/src/position_cmd_to_px4_bridge/include/position_cmd_to_px4_bridge/bridge_state_machine.hpp:1)
- [bridge_state_machine.cpp](/home/morphing01/Drone_SLAM/overlay_ws/src/position_cmd_to_px4_bridge/src/bridge_state_machine.cpp:1)
- [position_cmd_to_px4_bridge_node.cpp](/home/morphing01/Drone_SLAM/overlay_ws/src/position_cmd_to_px4_bridge/src/position_cmd_to_px4_bridge_node.cpp:1)
- [position_cmd_to_px4_bridge.yaml](/home/morphing01/Drone_SLAM/overlay_ws/src/position_cmd_to_px4_bridge/config/position_cmd_to_px4_bridge.yaml:1)
- `position_cmd_to_px4_bridge` 相关测试
- [single_real.launch.py](/home/morphing01/Drone_SLAM/overlay_ws/src/gvf_ismc_real_bringup/launch/single_real.launch.py:1)
- `gvf_ismc_real_bringup` launch 合同测试
- [gvf-ismc-single-real.md](/home/morphing01/Drone_SLAM/docs/runbooks/gvf-ismc-single-real.md:1)

## 6. 测试

至少覆盖：

- 状态机在自动接管关闭时不再请求 `Offboard/Arm`
- 节点和 yaml 都声明 `auto_request_offboard_and_arm`
- `gvf_ismc_real_bringup` launch 显式传 `False`
- 构建与回归测试：
  - `position_cmd_to_px4_bridge`
  - `gvf_ismc_real_bringup`

## 7. 风险

主要风险：

- 状态机改动后迟迟进不了 `ACTIVE`
- 文档与代码再次不一致

规避方式：

- 用测试固定手动模式行为
- runbook 明确“ACTIVE 取决于你真的切进 Offboard 且已 armed”
