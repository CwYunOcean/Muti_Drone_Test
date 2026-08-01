# GVF + ISMC Path-Following Design

**Date:** 2026-05-06

## Goal

Add a real-flight path-following pipeline for the thesis vector-field experiment by reusing the current `FAST-LIO2 + PX4 offboard` integration path and inserting:

- a `GVF` reference generator as the upper-layer guidance law
- an `ISMC` outer-loop tracker that follows the GVF desired velocity

The first flight milestone should validate fixed-height three-leaf path following on real hardware without replacing PX4's onboard low-level attitude controller.

## Problem

The current repository already supports:

- real odometry from `FAST-LIO2`
- planner-facing leveled odometry and cloud topics
- `PositionCommand -> PX4` offboard bridging

However, the current real-flight path is centered on `ego_planner` and `traj_server`, which generate time-parameterized reference commands. That is not the right control structure for thesis vector-field path following.

The thesis control flow is:

`path geometry -> GVF -> desired velocity -> ISMC outer loop -> desired motion command -> inner loop controller`

Therefore the new experiment should bypass `ego_planner` and `traj_server` for the vector-field flight path and publish `PositionCommand` directly from the new outer-loop controller.

## Constraints

- Keep project-owned changes inside `overlay_ws/src/`.
- Avoid direct edits under `nav_ws/` and `slam_ws/`.
- Reuse the current `FAST-LIO2`, leveling, PX4 odometry bridge, and `position_cmd_to_px4_bridge` pipeline.
- Preserve a low-risk real-flight path by keeping PX4 as the low-level inner-loop controller for the first milestone.
- The first experiment should use a fixed-height three-leaf curve instead of a fully time-parameterized 3D trajectory.

## Existing Reusable Path

The current reusable data path is:

`FAST-LIO2 -> /aft_mapped_to_init`

Optional planner-facing leveling path:

`/aft_mapped_to_init + /cloud_registered -> fastlio2_to_ego_swarm_leveling -> /aft_mapped_to_init_level + /cloud_registered_level`

Current offboard command path:

`/drone_0_planning/pos_cmd -> position_cmd_to_px4_bridge -> /fmu/in/offboard_control_mode + /fmu/in/trajectory_setpoint + /fmu/in/vehicle_command`

This means the lowest-risk insertion point is to replace the current `pos_cmd` producer, not to redesign the PX4 bridge or SLAM stack.

## Chosen Architecture

### 1. Bypass `ego_planner` and `traj_server` for GVF experiments

The vector-field experiment should not reuse the `ego_planner -> traj_server` path. That path is built around time-parameterized trajectory execution, while GVF path following is built around state-dependent geometric guidance.

For the thesis experiment, the new controller stack should publish `PositionCommand` directly to:

- `/drone_0_planning/pos_cmd`

This keeps the downstream PX4 bridge unchanged while removing the mismatch between vector-field guidance and trajectory playback.

### 2. Split guidance and control into two local nodes

Create two new local ROS 2 nodes under `overlay_ws/src/`:

- `gvf_reference_node`
- `ismc_velocity_tracker_node`

Responsibilities:

`gvf_reference_node`:

- subscribes to real odometry, preferably leveled odometry
- computes the path implicit functions and gradients
- computes the GVF desired velocity `v_d`
- computes a smooth desired yaw from `v_d`
- publishes a compact reference message for the outer-loop tracker

`ismc_velocity_tracker_node`:

- subscribes to the current odometry and GVF reference
- computes velocity-tracking error
- applies the selected ISMC outer-loop law
- outputs `quadrotor_msgs/msg/PositionCommand`

This split keeps the system debuggable and preserves a clean interface between guidance and tracking.

### 3. Keep PX4 as the low-level inner loop for the first real-flight milestone

For the first thesis flight experiments, the onboard PX4 attitude controller should remain active as the low-level inner loop.

That means the first real-flight validation is:

- full `GVF` upper-layer guidance on the companion computer
- full `ISMC` outer-loop velocity tracking on the companion computer
- PX4 retained as the practical low-level attitude and thrust controller

This is the best tradeoff between thesis relevance and flight safety.

## Path Model

The initial real-flight target is a fixed-height three-leaf path.

The MATLAB simulation currently uses:

- `phi1 = z`
- `phi2 = r - (r0 + a cos(3 theta))`

For real flight, the fixed-height constraint should become:

- `phi1 = z - h`

where:

- `h` is the target flight height above the leveled world plane

The three-leaf radial constraint remains:

- `phi2 = r - (r0 + a cos(3 theta))`

The first milestone should use:

- constant target height
- low speed
- enlarged curve size
- bounded yaw-rate behavior

## Topic and Message Design

### Guidance input

Preferred odometry input:

- `/aft_mapped_to_init_level`

This keeps the path geometry aligned with a leveled world plane and avoids embedding the fixed-height experiment inside a tilted raw world frame.

### New GVF reference topic

Add a compact custom message for the guidance output named:

- `GVFReference.msg`

Recommended fields:

- `std_msgs/Header header`
- `geometry_msgs/Vector3 desired_velocity`
- `float64 desired_yaw`
- `float64 desired_yaw_rate`
- `float64 phi1`
- `float64 phi2`

Recommended topic:

- `/gvf/reference`

This keeps GVF outputs observable and makes plotting and rosbag analysis straightforward.

### Controller output

The `ismc_velocity_tracker_node` should publish:

- `/drone_0_planning/pos_cmd`

using `quadrotor_msgs/msg/PositionCommand`.

Recommended first-version filling strategy:

- `position = NaN`
- `velocity = desired or corrected velocity command`
- `acceleration = a_cmd`
- `yaw = yaw_d`
- `yaw_dot = yaw_dot_d`

The bridge already consumes this message type, so no PX4 topic contract changes are required.

## Control Interface Strategy

### First flight milestone

The first practical milestone should prioritize:

- `velocity + yaw`

with optional `acceleration` fields present but not yet relied on by PX4.

Specifically:

- keep `position_cmd_to_px4_bridge` acceleration feedforward disabled at first
- validate direction, continuity, and offboard stability using velocity references first

### Follow-up milestone

After velocity-tracking flight is stable:

- enable acceleration feedforward
- compare tracking quality with and without the ISMC acceleration term

This staged rollout reduces the risk of coupling outer-loop tuning errors directly into PX4 too early.

## Experimental Validation Stages

### Stage 1: MATLAB to ROS formula alignment

Before flight, align the ROS implementation with the MATLAB reference for:

- `phi1`
- `phi2`
- `grad(phi2)`
- `v_d`
- `yaw_d`

Acceptance criteria:

- same sign conventions
- same tangent direction around the three-leaf path
- no heading discontinuity at low-speed regions

### Stage 2: ROS dry-run without PX4 flight

Run:

- `gvf_reference_node`
- `ismc_velocity_tracker_node`

without propeller-driven flight.

Validate:

- continuous `gvf/reference` output
- stable `pos_cmd` publication
- bounded `phi2`
- bounded `a_cmd`
- no `NaN` bursts

### Stage 3: PX4 command-chain validation on the bench

Connect the real command pipeline but do not perform a live free-flight test yet.

Validate:

- bridge transitions to `ACTIVE`
- `/fmu/in/offboard_control_mode` streams continuously
- `/fmu/in/trajectory_setpoint` streams continuously
- command axes and yaw direction are correct

### Stage 4: Low-risk initial flight

Do not start with the three-leaf path.

Start with:

- manual `Position`-mode takeoff
- short hover stabilization
- controller takeover with very small reference motion
- simple line or large-radius circle tests

### Stage 5: Low-speed three-leaf flight

Only after the previous stages are stable:

- fly a fixed-height three-leaf path
- low speed
- low acceleration
- one or two loops per run

### Stage 6: Thesis data capture

Record at least:

- actual flight path
- target three-leaf path
- `|phi2|`
- height error `|z - h|`
- velocity-tracking error
- desired versus actual velocity
- yaw evolution
- offboard continuity
- PX4 log + ROS bag + video

## Recommended Initial Tuning Direction

The first real-flight version should not copy the MATLAB gains directly without adjustment because the real system retains PX4 as the inner loop.

Recommended first-pass direction:

- low GVF gains
- low speed cap
- low acceleration cap
- smoothed desired yaw
- weak or disabled adaptive term initially
- acceleration feedforward disabled initially

The experiment should be tuned in two passes:

1. `GVF + velocity tracking + PX4`
2. `GVF + ISMC acceleration term + PX4`

## Failure Modes and Mitigations

### 1. Axis or sign mismatch

Symptoms:

- motion along the wrong tangent direction
- incorrect height response
- yaw aligned opposite to the path direction

Mitigation:

- use leveled odometry first
- validate sign conventions on the bench before flight

### 2. Yaw oscillation near low-speed regions

Symptoms:

- heading jumps near petal transitions

Mitigation:

- hold previous yaw when planar speed is very small
- apply first-order yaw smoothing

### 3. Over-aggressive acceleration command

Symptoms:

- oscillation after enabling acceleration feedforward

Mitigation:

- limit `a_cmd`
- enable acceleration feedforward only after velocity-only flight is stable

### 4. Command timeout or offboard drop

Symptoms:

- bridge leaves active streaming state
- PX4 exits offboard or triggers failsafe

Mitigation:

- maintain a stable controller publication rate
- keep command output continuous at all times

### 5. Odom jump amplification

Symptoms:

- spikes in `phi2`, `v_d`, or `a_cmd`

Mitigation:

- reject invalid odometry samples
- bound command derivatives

## Non-Goals for the First Milestone

The first milestone should not attempt to:

- replace PX4's low-level inner loop
- claim full real-flight validation of the entire DDSMC stack
- combine `ego_planner` with GVF in the same control chain
- start with aggressive 3D path shaping or obstacle-rich flight

## Thesis Positioning

The simulation section may continue to validate the full proposed `GVF + DDSMC/ISMC` framework.

The first real-flight section should be positioned more carefully:

- it validates the practicality of GVF-guided outer-loop path following on the real platform
- it does not yet prove full onboard replacement of the low-level attitude controller

Recommended wording direction:

- real-flight validation of the GVF-guided outer-loop path-following strategy
- partial hardware validation of the proposed framework
- practical flight demonstration with PX4 retained as the low-level inner loop

Wording to avoid unless the full inner loop is also moved onboard:

- full real-flight validation of the complete DDSMC strategy
- full hardware implementation of both inner and outer loops

## Expected Outcome

After implementation, the repository should support a clean thesis experiment path:

`FAST-LIO2 -> leveled odometry -> GVF guidance -> ISMC outer-loop tracking -> PositionCommand -> PX4 offboard`

This should produce a repeatable real-flight workflow for fixed-height three-leaf path-following experiments while preserving the current SLAM and PX4 integration work already completed in the repository.
