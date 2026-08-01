# GVF Reference RViz Visualization Design

更新时间：`2026-05-08`

## 1. 目标

为当前 `GVF + ISMC + PX4 bridge` 实机链路补充一个最小的参考轨迹可视化能力，满足以下需求：

- 在 `RViz` 中直接看到固定高度三叶草参考轨迹
- 不修改现有控制链和接管逻辑
- 不新增独立可视化包
- 单独提供一个专用 `RViz` 配置文件，放到 `scripts/` 目录

本次改动不处理控制器启动即追踪的问题，也不增加控制使能开关。

## 2. 现状

当前 [gvf_reference_node.cpp](/home/morphing01/Drone_SLAM/overlay_ws/src/gvf_ismc_path_following/src/gvf_reference_node.cpp:1) 只发布：

- `/gvf/reference`

消息类型为 [GVFReference.msg](/home/morphing01/Drone_SLAM/overlay_ws/src/gvf_path_following_msgs/msg/GVFReference.msg:1)，包含期望速度、偏航和 `phi1/phi2`，但不包含任何 `RViz` 可直接显示的路径或 marker。

当前仓库已有 `RViz` 使用先例：

- `rviz_default_plugins/Path`
- `rviz_default_plugins/Marker`

因此不需要引入新的显示方式。

## 3. 方案选型

### 方案 A：只发布 `nav_msgs/Path`

优点：

- 实现最简单
- 与现有 RViz `Path` 显示直接兼容

缺点：

- 视觉辨识度一般
- 线宽与颜色灵活性比 `Marker` 弱

### 方案 B：只发布 `visualization_msgs/Marker`

优点：

- 颜色、线宽、样式更灵活
- 更适合突出“参考曲线”

缺点：

- 不如 `Path` 语义直观

### 方案 C：同时发布 `Path + Marker`

优点：

- `Path` 语义清晰
- `Marker` 显示效果更好
- 调试时可保留其中一种作为备用

缺点：

- 比单一发布多一点点实现量

推荐采用方案 C。

## 4. 设计

### 4.1 节点边界

不新增节点，直接在 `gvf_reference_node` 中增加静态参考轨迹发布。

新增两个 publisher：

- `/gvf/reference_path`
- `/gvf/reference_marker`

它们只表达“理论参考曲线几何形状”，不表达实时跟踪历史。

### 4.2 轨迹生成

参考曲线与当前控制器参数保持一致：

- `target_height_m`
- `base_radius_m`
- `lobe_amplitude_m`

采样方式：

- 均匀采样 `theta in [0, 2pi]`
- 默认采样点数固定，例如 `200` 或 `300`
- 曲线闭合，首尾相接

生成公式：

- `r = base_radius_m + lobe_amplitude_m * cos(3 * theta)`
- `x = r * cos(theta)`
- `y = r * sin(theta)`
- `z = target_height_m`

注意这里是“固定高度三叶草”，与当前实飞版本 `phi1 = z - h` 保持一致。

### 4.3 发布时间机

静态参考轨迹不依赖实时 odom 才能定义，因此在节点启动后即可发布。

保守实现方式：

- 节点启动时生成一次参考曲线
- 周期性重发，避免 RViz 后启动看不到

不要求 latched 行为，周期性发布即可。

### 4.4 消息形式

`/gvf/reference_path`

- 类型：`nav_msgs/msg/Path`
- `header.frame_id` 使用与控制器一致的世界系
- 默认使用 `world`

`/gvf/reference_marker`

- 类型：`visualization_msgs/msg/Marker`
- 类型选 `LINE_STRIP`
- 颜色默认使用高可见度，例如青色或黄色
- 线宽明显高于默认 `Path`

### 4.5 RViz 配置文件

新增文件：

- `/home/morphing01/Drone_SLAM/scripts/gvf_ismc_reference.rviz`

固定显示至少包含：

- `TF`
- `Odometry`
  - `/aft_mapped_to_init_level`
- `Path`
  - `/gvf/reference_path`
- `Marker`
  - `/gvf/reference_marker`

如果显示过杂，不默认打开点云大图层。

### 4.6 使用方式

保持控制链脚本不变，单独开 RViz：

```bash
rviz2 -d /home/morphing01/Drone_SLAM/scripts/gvf_ismc_reference.rviz
```

## 5. 影响范围

预计修改范围：

- [gvf_reference_node.cpp](/home/morphing01/Drone_SLAM/overlay_ws/src/gvf_ismc_path_following/src/gvf_reference_node.cpp:1)
- [CMakeLists.txt](/home/morphing01/Drone_SLAM/overlay_ws/src/gvf_ismc_path_following/CMakeLists.txt:1)
- [package.xml](/home/morphing01/Drone_SLAM/overlay_ws/src/gvf_ismc_path_following/package.xml:1)
- 相关合同测试
- 新增 `scripts/gvf_ismc_reference.rviz`

不修改：

- `single_real.launch.py`
- `ismc_velocity_tracker_node`
- `position_cmd_to_px4_bridge`

## 6. 测试

至少覆盖：

- 配置/合同测试确认新增依赖声明存在
- 节点源码合同测试确认存在 `Path` 和 `Marker` publisher
- `bash -n` 不适用 `rviz` 文件，但需确认路径有效
- `colcon test --packages-select gvf_ismc_path_following`

## 7. 风险

主要风险很低，集中在：

- `frame_id` 选错导致 RViz 中曲线位置不对
- RViz 配置固定 frame 与话题 frame 不一致
- 参考曲线参数与控制器实际参数不同步

规避方式：

- 使用同一组参数源
- `Path/Marker` 统一使用世界系
- RViz 固定坐标系与 `Path` 保持一致
