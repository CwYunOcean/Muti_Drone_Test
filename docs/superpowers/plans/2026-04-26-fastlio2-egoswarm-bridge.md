# FAST-LIO2 EGO-Swarm Bridge Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a dedicated `fastlio2_to_px4_odometry` package and make `ego_swarm_real_bringup` switchable between `FAST-LIVO2` and `FAST-LIO2` PX4 odometry bridges while keeping planner topics unchanged.

**Architecture:** Keep planner-facing topics stable on `/aft_mapped_to_init` and `/cloud_registered`, create a near-copy `fastlio2_to_px4_odometry` package to preserve existing PX4 odometry conversion behavior, and add one launch argument in `ego_swarm_real_bringup` to choose which bridge node to start.

**Tech Stack:** ROS 2 Humble, `ament_cmake`, `ament_cmake_gtest`, `ament_cmake_pytest`, C++, Python launch files, `nav_msgs`, `px4_msgs`, `rclcpp`.

---

### Task 1: Add failing bridge and launch contract tests

**Files:**
- Create: `overlay_ws/src/fastlio2_to_px4_odometry/test/test_odometry_conversion.cpp`
- Create: `overlay_ws/src/fastlio2_to_px4_odometry/test/test_config_contract.py`
- Modify: `overlay_ws/src/ego_swarm_real_bringup/test/test_single_real_launch_contract.py`

- [ ] **Step 1: Write a failing conversion test for the new `fastlio2_to_px4_odometry` package**
- [ ] **Step 2: Write a failing config contract test for the new bridge package**
- [ ] **Step 3: Extend the real-hardware launch contract test to expect `odom_bridge_type` support and both bridge nodes**
- [ ] **Step 4: Run the targeted tests to verify they fail for the missing package and missing launch wiring**

### Task 2: Implement `fastlio2_to_px4_odometry`

**Files:**
- Create: `overlay_ws/src/fastlio2_to_px4_odometry/CMakeLists.txt`
- Create: `overlay_ws/src/fastlio2_to_px4_odometry/package.xml`
- Create: `overlay_ws/src/fastlio2_to_px4_odometry/config/fastlio2_to_px4_odometry.yaml`
- Create: `overlay_ws/src/fastlio2_to_px4_odometry/include/fastlio2_to_px4_odometry/frame_mapping.hpp`
- Create: `overlay_ws/src/fastlio2_to_px4_odometry/include/fastlio2_to_px4_odometry/odometry_conversion.hpp`
- Create: `overlay_ws/src/fastlio2_to_px4_odometry/src/fastlio2_to_px4_odometry_node.cpp`
- Create: `overlay_ws/src/fastlio2_to_px4_odometry/src/odometry_conversion.cpp`

- [ ] **Step 1: Add package metadata, install rules, and test registrations**
- [ ] **Step 2: Copy the current odometry conversion logic into the new package namespace**
- [ ] **Step 3: Add a source-specific config file with the current `/aft_mapped_to_init -> /fmu/in/vehicle_visual_odometry` contract**
- [ ] **Step 4: Build the new package and run its tests**

### Task 3: Make `ego_swarm_real_bringup` source-selectable

**Files:**
- Modify: `overlay_ws/src/ego_swarm_real_bringup/launch/single_real.launch.py`
- Modify: `overlay_ws/src/ego_swarm_real_bringup/package.xml`

- [ ] **Step 1: Add `odom_bridge_type` as a launch argument with default `fastlivo`**
- [ ] **Step 2: Keep planner nodes unchanged**
- [ ] **Step 3: Launch `fastlivo_to_px4_odometry` when `odom_bridge_type=fastlivo`**
- [ ] **Step 4: Launch `fastlio2_to_px4_odometry` when `odom_bridge_type=fastlio2`**
- [ ] **Step 5: Re-run the real-hardware launch contract test**

### Task 4: Verify the integrated behavior

**Files:**
- Modify: `overlay_ws/src/ego_swarm_real_bringup/test/test_single_real_launch_contract.py`

- [ ] **Step 1: Build `fastlio2_to_px4_odometry` and `ego_swarm_real_bringup` together**
- [ ] **Step 2: Run `colcon test --packages-select fastlio2_to_px4_odometry ego_swarm_real_bringup`**
- [ ] **Step 3: Run `colcon test-result --verbose` for the two target packages**
