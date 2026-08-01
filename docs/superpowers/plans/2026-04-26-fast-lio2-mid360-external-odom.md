# FAST-LIO2 MID360 External Odom Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a project-owned FAST-LIO2 MID360 bring-up that publishes compatible odometry on `/aft_mapped_to_init` and applies a fixed `-30 deg` source-side pitch compensation.

**Architecture:** Create a new overlay bring-up package instead of editing upstream FAST-LIO2 or the existing FAST-LIVO2 package. The package owns one launch file, one FAST-LIO2 config override, and package-local contract tests, while a repo-level script mirrors the existing helper-script style.

**Tech Stack:** ROS 2 Humble, `ament_cmake`, Python launch files, `livox_ros_driver2`, `fast_lio`, `pytest`, bash helper scripts.

---

### Task 1: Add failing contract tests for the new package

**Files:**
- Create: `overlay_ws/src/fast_lio2_mid360_bringup/test/test_mid360_fastlio2_config_contract.py`
- Create: `overlay_ws/src/fast_lio2_mid360_bringup/test/test_mid360_fastlio2_launch_contract.py`
- Create: `overlay_ws/src/fast_lio2_mid360_bringup/test/test_mid360_fastlio2_script_contract.py`

- [ ] **Step 1: Write the failing tests**
- [ ] **Step 2: Run direct `pytest` commands and confirm they fail because the package files do not exist yet**

### Task 2: Add the new project-owned FAST-LIO2 bring-up package

**Files:**
- Create: `overlay_ws/src/fast_lio2_mid360_bringup/CMakeLists.txt`
- Create: `overlay_ws/src/fast_lio2_mid360_bringup/package.xml`
- Create: `overlay_ws/src/fast_lio2_mid360_bringup/launch/mid360_fastlio2.launch.py`
- Create: `overlay_ws/src/fast_lio2_mid360_bringup/config/fast_lio2_mid360.yaml`

- [ ] **Step 1: Add package metadata and install rules**
- [ ] **Step 2: Add the FAST-LIO2 config override with `extrinsic_est_en: false` and `extrinsic_R` pitched by `-30 deg`**
- [ ] **Step 3: Add the launch file that starts `livox_ros_driver2_node` and `fastlio_mapping`, and remaps `/Odometry` to `/aft_mapped_to_init`**

### Task 3: Add the repo helper script

**Files:**
- Create: `scripts/run_mid360_fastlio2.sh`

- [ ] **Step 1: Mirror the existing workspace-source/build/launch pattern from `scripts/run_mid360_lio.sh`**
- [ ] **Step 2: Build only `fast_lio2_mid360_bringup`**
- [ ] **Step 3: Launch `fast_lio2_mid360_bringup mid360_fastlio2.launch.py` with the machine-local Livox JSON path**

### Task 4: Wire tests into the package and verify

**Files:**
- Modify: `overlay_ws/src/fast_lio2_mid360_bringup/CMakeLists.txt`

- [ ] **Step 1: Register the three pytest contract tests with `ament_add_pytest_test`**
- [ ] **Step 2: Run targeted `pytest` checks for the new files**
- [ ] **Step 3: Run `colcon build --symlink-install --packages-select fast_lio2_mid360_bringup`**
- [ ] **Step 4: Run `colcon test --packages-select fast_lio2_mid360_bringup` and `colcon test-result --verbose`**
