# FAST-LIO2 EGO-Swarm Bridge Design

**Date:** 2026-04-26

## Goal

Integrate `FAST-LIO2` into the existing real-hardware `ego_swarm_real_bringup` path without introducing a new all-in-one launch script.

The integration must:

- keep planner inputs compatible with the current `ego_planner` wiring
- add a dedicated `FAST-LIO2 -> PX4` external-odometry bridge package
- preserve the current `FAST-LIVO2` path instead of replacing it

## Scope

This change should:

- keep `ego_swarm_real_bringup` consuming `/aft_mapped_to_init` and `/cloud_registered`
- keep the existing `FAST-LIVO2` flow usable
- add a new `fastlio2_to_px4_odometry` package under `overlay_ws/src/`
- allow `ego_swarm_real_bringup` to switch between `fastlivo_to_px4_odometry` and `fastlio2_to_px4_odometry`
- avoid upstream edits to `FAST_LIO`

This change should not:

- add a new top-level "start everything" script
- rename existing `FAST-LIVO2` bridge packages
- redesign the planner frame conventions
- change the PX4 topic contract

## Current Constraints

### Planner side

The current real-hardware bring-up already expects:

- odometry on `/aft_mapped_to_init`
- registered cloud on `/cloud_registered`

That is already satisfied by the new local `FAST-LIO2` wrapper launch through `/Odometry -> /aft_mapped_to_init` remapping, so planner-side integration should stay topic-compatible.

### PX4 external odometry side

The existing `fastlivo_to_px4_odometry` implementation is technically reusable because it only consumes `nav_msgs/msg/Odometry`. However, keeping that package name for `FAST-LIO2` would make the source path ambiguous and harder to tune independently later.

## Recommended Design

### 1. Keep planner inputs unchanged

Do not create a new planner-facing adapter.

`ego_swarm_real_bringup` should continue to use:

- `odom_topic:=/aft_mapped_to_init`
- `cloud_topic:=/cloud_registered`

That keeps both `FAST-LIVO2` and the new `FAST-LIO2` wrapper launch compatible with the same planner interface.

### 2. Add a dedicated `fastlio2_to_px4_odometry` package

Create a new local package:

- `overlay_ws/src/fastlio2_to_px4_odometry`

This package should mirror the current `fastlivo_to_px4_odometry` behavior closely:

- input: `/aft_mapped_to_init`
- output: `/fmu/in/vehicle_visual_odometry`
- same world/body axis remapping defaults
- same body extrinsic compensation pattern

The initial implementation should intentionally stay near-copy compatible with the existing bridge so that behavior differences come only from source odometry, not from a new conversion model.

### 3. Make `ego_swarm_real_bringup` source-selectable

Update `ego_swarm_real_bringup` so the bridge node it launches is configurable.

Recommended mechanism:

- add a launch argument such as `odom_bridge_type`
- accepted values:
  - `fastlivo`
  - `fastlio2`

Behavior:

- `fastlivo` starts `fastlivo_to_px4_odometry`
- `fastlio2` starts `fastlio2_to_px4_odometry`

Planner nodes continue to launch unchanged in both cases.

This keeps the bring-up dual-source compatible while making the PX4 bridge naming explicit.

### 4. Keep topic contracts stable

For both bridge types, keep the same ROS and PX4 interface:

- input odometry: `/aft_mapped_to_init`
- output PX4 visual odometry: `/fmu/in/vehicle_visual_odometry`

This avoids touching:

- `position_cmd_to_px4_bridge`
- planner topic remaps
- PX4-side consumers

## File Responsibilities

### New package

`fastlio2_to_px4_odometry` owns:

- odometry conversion node
- frame mapping helpers
- package-local config
- package-local tests

### Existing package changes

`ego_swarm_real_bringup` owns:

- launch-time selection of which odometry bridge package to start

It should not absorb source-specific conversion logic.

## Testing Strategy

Add focused package-local tests for:

### `fastlio2_to_px4_odometry`

- conversion contract matches the intended ENU/FLU -> NED/FRD mapping
- config file contains the expected topic defaults
- node still publishes `VehicleOdometry` with the same field-level conventions as the existing bridge

### `ego_swarm_real_bringup`

- launch contract verifies both supported bridge modes
- `fastlivo` mode still starts the old package
- `fastlio2` mode starts the new package
- planner nodes remain present in both modes

## Expected Outcome

After implementation:

- the new `FAST-LIO2` wrapper launch can feed planner topics without extra adapters
- `ego_swarm_real_bringup` can be pointed at `FAST-LIO2` without renaming planner inputs
- PX4 external odometry injection can use a source-specific bridge package with clear naming
- the old `FAST-LIVO2` path remains available for comparison and rollback

## Risks

### 1. Hidden divergence between the two odometry bridges

If the new bridge drifts from `fastlivo_to_px4_odometry` too early, it becomes harder to isolate whether runtime differences come from `FAST-LIO2` or from bridge behavior.

Mitigation:

- keep the first version intentionally close to the existing implementation

### 2. Launch branching complexity

Adding source selection inside `ego_swarm_real_bringup` can make launch contracts noisier.

Mitigation:

- keep selection to one explicit argument
- avoid adding parallel planner code paths

### 3. Frame compensation mismatch

If `FAST-LIO2` source-side tilt correction changes the effective body orientation, the PX4 bridge's body extrinsic compensation may need a small follow-up adjustment.

Mitigation:

- keep the initial bridge config aligned with the current `FAST-LIVO2` defaults
- tune only after the source-side map tilt is validated
