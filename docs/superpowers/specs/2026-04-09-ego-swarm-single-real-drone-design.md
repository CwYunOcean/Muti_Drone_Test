# EGO-Swarm ROS2 Single-Drone Real Hardware Replication Design

## Goal

Replicate the current `ego-swarm-ros2` planning stack for a single real drone while preserving the upstream swarm planner structure as much as possible. The real-hardware path will use FAST-LIVO as the ROS-side state-estimation and obstacle source, and PX4 as the flight controller receiving external odometry plus offboard trajectory commands.

## Scope

In scope:

- Single-drone deployment only
- Reuse upstream planner packages under `nav_ws/src/ego-swarm-ros2/planner`
- Replace the simulation loop with real-hardware bridge nodes
- Use FAST-LIVO topics currently available from `run_mid360_lio.sh`
- Use `px4_msgs` / `px4_ros_com` style PX4 ROS2 interfaces
- Create a dedicated real-hardware bringup path under project-owned workspace code

Out of scope for this phase:

- Multi-drone swarm execution
- Rewriting upstream planner algorithms
- Full MAVROS-based integration
- Vision/depth simulation via `local_sensing`
- Simulation launch cleanup beyond what is needed to keep real-hardware code separate

## Current Project Context

The upstream ROS2 port mixes planning logic and simulation support:

- `ego_planner_node` consumes world-frame odometry plus local obstacle information, then publishes B-spline trajectories.
- `traj_server` expands the B-spline into `quadrotor_msgs/PositionCommand`.
- Simulation launch files add `fake_drone`, `so3_control`, `so3_quadrotor_simulator`, `odom_visualization`, and `pcl_render_node`.

The current FAST-LIVO stack started by `/home/morphing01/Drone_SLAM/scripts/run_mid360_lio.sh` exposes the relevant topics:

- `/aft_mapped_to_init` as `nav_msgs/msg/Odometry`
- `/cloud_registered` as `sensor_msgs/msg/PointCloud2`
- `/Laser_map` as `sensor_msgs/msg/PointCloud2`
- `/LIVO2/imu_propagate` as `nav_msgs/msg/Odometry`
- `/mavros/vision_pose/pose` as `geometry_msgs/msg/PoseStamped`

The reference PX4 integration in `/home/morphing01/Desktop/AIM` uses:

- `px4_msgs`
- `px4_ros_com`
- `fmu/in/offboard_control_mode`
- `fmu/in/trajectory_setpoint`
- `fmu/in/vehicle_command`
- `fmu/in/vehicle_visual_odometry`

This makes `px4_msgs` the preferred interface for the current project.

## Design Principles

1. Preserve upstream planner integrity.
2. Keep all real-hardware integration in project-owned overlay code.
3. Keep planner-side coordinates in one ROS world frame.
4. Perform all PX4-specific frame conversion only at the bridge boundary.
5. Validate the pipeline in stages: odometry injection, fixed offboard control, planner output, then obstacle-aware flight.

## Recommended Architecture

### Planner Side

Planner-side logic remains unchanged in principle:

- `ego_planner_node` consumes odometry and obstacle cloud
- `traj_server` converts upstream B-spline trajectories to `quadrotor_msgs/PositionCommand`

The planner will remain a ROS-world-frame consumer and producer. It should not know about PX4 NED conventions or flight-controller-specific topics.

### Hardware Integration Side

Add two bridge nodes plus one dedicated real-hardware bringup package:

- `ego_swarm_real_bringup`
- `fastlivo_to_px4_odometry`
- `position_cmd_to_px4_bridge`

These packages will live in `/home/morphing01/Drone_SLAM/overlay_ws/src`.

## Package Layout

### `ego_swarm_real_bringup`

Purpose:

- Own all single-drone real-hardware launch files
- Declare and centralize planner parameters for real deployment
- Remap FAST-LIVO and PX4 topics into the reused planner nodes

Contents:

- `launch/single_real.launch.py`
- `config/ego_planner_real.yaml`
- `README.md`

### `fastlivo_to_px4_odometry`

Purpose:

- Subscribe to FAST-LIVO odometry
- Convert ROS world-frame odometry into PX4-compatible `px4_msgs/msg/VehicleOdometry`
- Publish to `fmu/in/vehicle_visual_odometry`

Contents:

- `src/fastlivo_to_px4_odometry_node.cpp`
- `include/...` only if needed
- `config/fastlivo_to_px4_odometry.yaml`

### `position_cmd_to_px4_bridge`

Purpose:

- Subscribe to `quadrotor_msgs/msg/PositionCommand`
- Convert ROS world-frame trajectory command to PX4 offboard setpoints
- Manage offboard activation, arming, command streaming, and safety fallback

Contents:

- `src/position_cmd_to_px4_bridge_node.cpp`
- `config/position_cmd_to_px4_bridge.yaml`

## Coordinate Frames

### ROS Planner Frame

The ROS-side global frame is FAST-LIVO's `camera_init`, treated as planner `world`.

The tracked body frame is `base_link`.

Planner odometry must therefore represent:

- pose of `base_link` relative to `camera_init/world`
- linear velocity consistent with the same world frame

The planner and obstacle map must use the same fixed frame.

### PX4 Frame

PX4 inputs will use NED semantics.

Therefore:

- planner side stays in ROS world frame
- both bridge nodes are solely responsible for ENU/world to NED conversion
- no planner package will embed PX4 frame assumptions

This keeps upstream planner logic reusable for future swarm expansion.

## Topic Contract

### FAST-LIVO to Planner

- `odom_world` <- `/aft_mapped_to_init`
- `grid_map/odom` <- `/aft_mapped_to_init`
- `grid_map/cloud` <- `/cloud_registered`

Initial recommendation:

- use `/aft_mapped_to_init` as the planner odometry source
- use `/cloud_registered` as the first obstacle cloud source

`/LIVO2/imu_propagate` is a backup candidate only if testing shows a stronger velocity estimate.

### Planner Internal Topics

For `drone_id = 0`:

- `planning/bspline` -> `/drone_0_planning/bspline`
- `planning/data_display` -> `/drone_0_planning/data_display`
- `position_cmd` -> `/drone_0_planning/pos_cmd`

### Goal Input

Phase 1 goal input remains manual:

- `/move_base_simple/goal`

This avoids adding preset-trigger logic during the first real-hardware integration.

### FAST-LIVO to PX4

- `/aft_mapped_to_init` -> `fastlivo_to_px4_odometry`
- `fastlivo_to_px4_odometry` -> `/px4_1/fmu/in/vehicle_visual_odometry`

### Planner to PX4

- `/drone_0_planning/pos_cmd` -> `position_cmd_to_px4_bridge`
- `position_cmd_to_px4_bridge` -> `/px4_1/fmu/in/offboard_control_mode`
- `position_cmd_to_px4_bridge` -> `/px4_1/fmu/in/trajectory_setpoint`
- `position_cmd_to_px4_bridge` -> `/px4_1/fmu/in/vehicle_command`

Optional status feedback to the bridge:

- `/px4_1/fmu/out/vehicle_status`
- `/px4_1/fmu/out/vehicle_local_position`
- `/px4_1/fmu/out/vehicle_odometry`

## Real-Hardware Launch Composition

The single-drone real-hardware launch should start exactly these nodes:

1. `ego_planner_node`
2. `traj_server`
3. `fastlivo_to_px4_odometry`
4. `position_cmd_to_px4_bridge`

It should not start:

- `poscmd_2_odom`
- `so3_control`
- `so3_quadrotor_simulator`
- `pcl_render_node`
- `map_generator`
- `mockamap`
- `odom_visualization` unless explicitly enabled for debug only

## Planner Parameter Strategy

The real-hardware bringup package should provide planner parameters directly instead of including the upstream simulation-oriented launch wrappers.

Recommended initial planner mode:

- `fsm/flight_type = 1`
- manual goal input through `/move_base_simple/goal`
- `fsm/realworld_experiment = false`

Reason:

- avoids the preset-trigger wait path during first hardware integration
- keeps startup logic simple
- allows operator-controlled target injection

Recommended first-pass safety parameters should stay conservative:

- low `manager/max_vel`
- low `manager/max_acc`
- limited planning horizon
- moderate obstacle inflation

These values should be tuned in configuration rather than hardcoded in bridge nodes.

## Bridge Node Behavior

### `fastlivo_to_px4_odometry`

Responsibilities:

- subscribe to FAST-LIVO `nav_msgs/msg/Odometry`
- convert position, orientation, and velocity from ROS world frame to PX4 NED
- publish `px4_msgs/msg/VehicleOdometry`
- set `pose_frame` and `velocity_frame` explicitly
- keep timestamp handling consistent and monotonic
- expose frame and axis conversion settings in parameters when possible

Safety and validation:

- reject invalid quaternion or NaN fields
- report odometry timeout
- optionally low-pass or validate velocity if FAST-LIVO velocity proves noisy

Reference style:

- reuse the interface structure demonstrated by AIM's `vrpn_to_px4_pkg`
- do not directly reuse motion-capture assumptions or topic names

### `position_cmd_to_px4_bridge`

Responsibilities:

- subscribe to `quadrotor_msgs/msg/PositionCommand`
- convert position, velocity, acceleration, and yaw to PX4 offboard setpoints
- publish `OffboardControlMode`
- publish `TrajectorySetpoint`
- publish `VehicleCommand`
- manage offboard-mode entry and arming sequence

Offboard strategy:

- continuously stream setpoints before requesting `OFFBOARD`
- enter `OFFBOARD`
- arm only after sufficient setpoint streaming and required state checks
- keep publishing setpoints at a fixed rate during active control

Initial setpoint policy:

- first version uses position plus velocity fields from `PositionCommand`
- yaw follows `PositionCommand.yaw`
- acceleration feedforward may be forwarded only if verified safe against PX4 version and behavior

## Offboard Safety Model

The bridge should implement a conservative state machine:

- `IDLE`
- `WAIT_FASTLIVO`
- `WAIT_PX4`
- `STREAM_SETPOINT`
- `ENTER_OFFBOARD`
- `ARM`
- `ACTIVE`
- `FAILSAFE`

Recommended transitions:

- no `OFFBOARD` without valid FAST-LIVO odometry
- no arming without valid PX4-side readiness
- no active trajectory following without fresh `PositionCommand`

Recommended timeout policy:

- short planner-command timeout -> hold current target / hover-like behavior
- long planner-command timeout -> land or exit offboard according to config
- FAST-LIVO odometry timeout -> immediate failsafe path
- manual PX4 mode override -> bridge returns to passive state

Important boundary:

- planner-side emergency stop is not a full flight-controller failsafe
- PX4 bridge owns actual hardware safety closure

## Implementation Staging

### Stage 1: PX4 External Odometry Only

Goal:

- validate `FAST-LIVO -> VehicleOdometry -> PX4 EKF2`

Success criteria:

- PX4 reports stable external odometry fusion
- vehicle estimate remains consistent while planner is not yet involved

### Stage 2: Fixed Offboard Setpoint

Goal:

- validate `px4_msgs` offboard control without planner

Success criteria:

- bridge can stream setpoints
- vehicle can enter `OFFBOARD`
- vehicle can arm
- vehicle can take off and hover at a fixed target
- vehicle can land through the bridge path

### Stage 3: Planner Output Without Obstacle Pressure

Goal:

- connect `traj_server` output to the PX4 bridge in a simplified environment

Success criteria:

- `PositionCommand` is consumed correctly
- vehicle tracks simple planner-generated targets in open space

### Stage 4: Manual Goal Navigation

Goal:

- allow operator to publish `/move_base_simple/goal`

Success criteria:

- `ego_planner` replans from current pose
- `traj_server` outputs valid commands
- PX4 bridge tracks the command stream stably

### Stage 5: Obstacle-Aware Flight

Goal:

- enable `/cloud_registered -> grid_map/cloud`

Success criteria:

- planner map updates in real time
- planner avoids observed obstacles conservatively
- end-to-end navigation remains stable

## Testing Strategy

### Unit-Level Checks

- frame conversion helpers in both bridge nodes
- NaN / invalid quaternion rejection
- timestamp conversion
- timeout and state-machine transitions

### Integration Checks on Bench

- inspect ROS topic flow with nodes running but motors safe
- verify `VehicleOdometry` publication rate and fields
- verify `TrajectorySetpoint` publication rate and fields
- verify correct topic namespace for the targeted PX4 instance

### Flight Progression

- tethered or protected environment first
- low altitude
- low velocity
- low acceleration
- manual takeover ready at all times

## Risks and Open Technical Checks

1. `/aft_mapped_to_init` velocity frame must be verified. The planner assumes the reported linear velocity can be used directly as world-frame velocity.
2. `/cloud_registered` frame must be verified. If it is not already in the same fixed frame as odometry, an adapter transform will be required.
3. PX4 version and `px4_msgs` version in the current environment must match the expected message layout and offboard behavior.
4. EKF2 external-vision configuration on the flight controller still needs to be validated during implementation.

These are implementation-time verification tasks, not reasons to change the architecture.

## Why This Design

This design preserves the swarm planner core while isolating all hardware-specific logic into project-owned overlay packages. It also follows the integration direction already used in the AIM workspace, reducing toolchain novelty and making it more likely that the first real-drone replication succeeds without invasive upstream changes.
