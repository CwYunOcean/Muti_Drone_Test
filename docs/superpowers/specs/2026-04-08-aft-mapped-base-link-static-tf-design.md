# Aft-Mapped To Base-Link Static TF Design

**Date:** 2026-04-08

## Goal

Add a static TF that converts the current sensor/body pose frame `aft_mapped` into a more useful UAV body frame `base_link`.

The immediate goal is not a full frame-tree redesign. The immediate goal is to compensate only the fixed sensor installation pitch so that `base_link` better represents the aircraft body attitude while leaving the existing `camera_init -> aft_mapped` estimation path untouched.

## Scope

This change should:

- keep the current `camera_init -> aft_mapped` output exactly as-is
- add one `aft_mapped -> base_link` static transform in the project-owned wrapper launch
- apply rotation compensation only
- keep translation at `0 0 0`
- expose the compensation as a launch parameter so it can be tuned later without editing code

This change should not:

- modify upstream `FAST-LIVO2`
- redefine `aft_mapped`
- introduce a new state-estimation node
- add LiDAR/body translation offsets yet

## Current Evidence

Observed static transform samples while the vehicle was stationary showed:

- roll near `0.2 deg`
- yaw near `0.1 deg`
- pitch near `29.75 deg`

This strongly suggests the current `aft_mapped` frame is stable and mainly reflects the fixed sensor installation pitch relative to the gravity-aligned world.

## Recommended Design

Add a `static_transform_publisher` node to `fast_livo2_mid360_bringup/launch/mid360_lio.launch.py` with:

- parent frame: `aft_mapped`
- child frame: `base_link`
- translation: `0 0 0`
- roll: `0`
- yaw: `0`
- pitch: launch-configurable, defaulting to the negative of the observed installation pitch

Initial default:

- `base_link_pitch_rad = -0.519`

This keeps `aft_mapped` available as the original sensor/body frame while providing a practical `base_link` for downstream vehicle-facing consumers.

## Expected Outcome

After the change:

- maps and point clouds remain in the same world frame
- `aft_mapped` remains unchanged
- `base_link` appears approximately level with the aircraft body when the vehicle is stationary on the ground
