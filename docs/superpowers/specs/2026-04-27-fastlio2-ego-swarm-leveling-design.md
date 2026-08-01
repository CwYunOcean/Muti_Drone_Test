# FAST-LIO2 EGO-Swarm Leveling Design

## Goal

Keep `FAST-LIO2` native odometry and PX4 external-vision integration unchanged while providing a separate, gravity-leveled odometry and point-cloud stream for `ego_swarm` perception and planning.

## Problem

`MID360` is mounted with a fixed pitch offset relative to the airframe for downward perception. With the built-in IMU, `FAST-LIO2` should keep the LiDAR-to-IMU extrinsic unchanged. However, the `camera_init` world used by upstream `FAST-LIO2` is not guaranteed to be horizontally aligned with the real ground plane for this installation. That is acceptable for localization, but `ego_swarm` directly consumes `/aft_mapped_to_init` and `/cloud_registered` as world-frame planning inputs, so the local occupancy map and planner run inside a tilted world.

## Constraints

- Do not modify `FAST-LIO2` LiDAR-to-IMU extrinsics to compensate installation pitch.
- Do not change the PX4 bridge input path by default.
- Preserve raw `FAST-LIO2` topics:
  - `/aft_mapped_to_init`
  - `/cloud_registered`
- Avoid direct upstream edits under `slam_ws/` or `nav_ws/` unless integration cannot be solved in `overlay_ws/`.

## Findings

- Upstream `FAST-LIO2` publishes world-frame odometry and point clouds in `camera_init`.
- `ego_swarm` independent cloud ingestion does not use TF to transform incoming points before occupancy integration.
- Therefore, adding a TF alone will not fix planning; the consumed odometry and cloud messages themselves must be rotated into a leveled planning frame.

## Chosen Approach

Add a new local ROS 2 package in `overlay_ws/src/` that:

- subscribes to raw `FAST-LIO2` odometry and cloud topics,
- applies one fixed rotation from the raw planning world into a leveled planning world,
- republishes:
  - leveled odometry topic,
  - leveled cloud topic,
  - leveled frame IDs for planner-facing data.

`ego_swarm_real_bringup` will gain a planner input mode switch:

- `raw`: existing behavior
- `fastlio2_leveled`: planner consumes the leveled adapter outputs

The PX4 bridge selection remains independent and continues to use the raw `FAST-LIO2` odometry unless explicitly changed later.

## Data Flow

Raw localization path:

`FAST-LIO2 -> /aft_mapped_to_init -> fastlio2_to_px4_odometry -> /fmu/in/vehicle_visual_odometry`

Leveled planning path:

`FAST-LIO2 -> (/aft_mapped_to_init, /cloud_registered) -> fastlio2_to_ego_swarm_leveling -> (/aft_mapped_to_init_level, /cloud_registered_level) -> ego_planner`

## Rotation Semantics

The new adapter exposes one fixed RPY rotation parameter, defined as:

- the rotation applied to incoming odometry poses, velocities, and point coordinates
- from the raw `FAST-LIO2` planning world into the leveled planning world

For the current installation, the default pitch compensation will be set to the inverse of the observed raw-world tilt so that the planner sees a horizontal ground plane. This value remains operator-tunable in config and launch.

## Interfaces

### New package

`fastlio2_to_ego_swarm_leveling`

Responsibilities:

- rotate `nav_msgs/msg/Odometry`
- rotate `sensor_msgs/msg/PointCloud2`
- republish with leveled frame/topic names

### Launch integration

`ego_swarm_real_bringup/launch/single_real.launch.py`

New launch argument:

- `planner_input_mode:=raw|fastlio2_leveled`

Behavior:

- `raw`: planner remaps stay on `/aft_mapped_to_init` and `/cloud_registered`, `grid_map/frame_id=camera_init`
- `fastlio2_leveled`: start leveling adapter, planner remaps switch to leveled topics, `grid_map/frame_id=camera_init_level`

### Script integration

`scripts/run_ego_single_real.sh`

New optional environment variable:

- `PLANNER_INPUT_MODE`

Default remains the existing raw behavior unless explicitly changed.

## Testing

- Unit test the fixed-rotation odometry transform.
- Unit test the fixed-rotation point-cloud transform.
- Add config contract test for the new leveling package.
- Extend `single_real.launch.py` contract tests to cover planner mode argument and leveling node presence.
- Keep existing FAST-LIO2 raw bringup tests unchanged.

## Risks

- The pitch compensation sign may need one real-hardware validation pass.
- If the cloud field layout differs from the expected FAST-LIO2 XYZI layout, generic point handling may need adjustment.

## Non-Goals

- Reworking upstream `FAST-LIO2` world-frame semantics
- Changing PX4 external vision to the leveled planner frame
- Generalizing this adapter for all odometry sources
