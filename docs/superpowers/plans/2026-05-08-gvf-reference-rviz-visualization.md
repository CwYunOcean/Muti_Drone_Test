# GVF Reference RViz Visualization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add static three-leaf reference `Path` and `Marker` visualization to `gvf_reference_node` and provide a dedicated RViz config under `scripts/`.

**Architecture:** Extend the existing `gvf_reference_node` to publish a precomputed fixed-height three-leaf curve as both `nav_msgs/Path` and `visualization_msgs/Marker`. Keep all control behavior unchanged, and add a standalone RViz configuration that displays leveled odometry, TF, and the reference curve topics.

**Tech Stack:** ROS 2 Humble, `rclcpp`, `nav_msgs`, `visualization_msgs`, RViz config YAML, `ament_cmake_pytest`

---

### Task 1: Add failing contracts for reference visualization outputs

**Files:**
- Modify: `overlay_ws/src/gvf_ismc_path_following/test/test_node_contract.py`
- Test: `overlay_ws/src/gvf_ismc_path_following/test/test_node_contract.py`

- [ ] **Step 1: Write the failing test**

Add assertions that `gvf_reference_node.cpp` creates:

```python
assert "create_publisher<nav_msgs::msg::Path>" in source
assert "create_publisher<visualization_msgs::msg::Marker>" in source
assert 'declare_parameter<std::string>("reference_path_topic"' in source
assert 'declare_parameter<std::string>("reference_marker_topic"' in source
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
cd /home/morphing01/Drone_SLAM/overlay_ws
source /opt/ros/humble/setup.bash
pytest src/gvf_ismc_path_following/test/test_node_contract.py -q
```

Expected: FAIL because the new publishers and parameters do not exist yet.

- [ ] **Step 3: Commit**

Do not commit yet. Continue to green step in Task 2.

### Task 2: Implement static Path and Marker publishers in gvf_reference_node

**Files:**
- Modify: `overlay_ws/src/gvf_ismc_path_following/src/gvf_reference_node.cpp`
- Modify: `overlay_ws/src/gvf_ismc_path_following/CMakeLists.txt`
- Modify: `overlay_ws/src/gvf_ismc_path_following/package.xml`
- Test: `overlay_ws/src/gvf_ismc_path_following/test/test_node_contract.py`

- [ ] **Step 1: Write minimal implementation**

In `gvf_reference_node.cpp`:

- add publishers for:
  - `nav_msgs::msg::Path`
  - `visualization_msgs::msg::Marker`
- add parameters:
  - `reference_path_topic`
  - `reference_marker_topic`
  - `reference_frame_id`
  - `reference_sample_count`
- precompute the fixed-height three-leaf reference curve from:
  - `target_height_m`
  - `base_radius_m`
  - `lobe_amplitude_m`
- publish the static path and marker on a timer so RViz started later can still receive them

In build metadata:

- add `find_package(visualization_msgs REQUIRED)`
- add `nav_msgs` and `visualization_msgs` dependencies to the node target if missing
- add runtime dependency in `package.xml`

- [ ] **Step 2: Run test to verify it passes**

Run:

```bash
cd /home/morphing01/Drone_SLAM/overlay_ws
source /opt/ros/humble/setup.bash
pytest src/gvf_ismc_path_following/test/test_node_contract.py -q
```

Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add \
  /home/morphing01/Drone_SLAM/overlay_ws/src/gvf_ismc_path_following/src/gvf_reference_node.cpp \
  /home/morphing01/Drone_SLAM/overlay_ws/src/gvf_ismc_path_following/CMakeLists.txt \
  /home/morphing01/Drone_SLAM/overlay_ws/src/gvf_ismc_path_following/package.xml \
  /home/morphing01/Drone_SLAM/overlay_ws/src/gvf_ismc_path_following/test/test_node_contract.py
git commit -m "feat: add gvf reference rviz publishers"
```

### Task 3: Add dedicated RViz config under scripts

**Files:**
- Create: `scripts/gvf_ismc_reference.rviz`
- Modify: `overlay_ws/src/gvf_ismc_real_bringup/test/test_single_real_script_contract.py`
- Test: `overlay_ws/src/gvf_ismc_real_bringup/test/test_single_real_script_contract.py`

- [ ] **Step 1: Write the failing test**

Add assertions that:

```python
rviz_path = Path(__file__).resolve().parents[4] / "scripts" / "gvf_ismc_reference.rviz"
assert rviz_path.exists()
assert "/gvf/reference_path" in rviz_text
assert "/gvf/reference_marker" in rviz_text
assert "/aft_mapped_to_init_level" in rviz_text
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
cd /home/morphing01/Drone_SLAM/overlay_ws
source /opt/ros/humble/setup.bash
pytest src/gvf_ismc_real_bringup/test/test_single_real_script_contract.py -q
```

Expected: FAIL because the RViz config file does not exist yet.

- [ ] **Step 3: Write minimal implementation**

Create `scripts/gvf_ismc_reference.rviz` with:

- fixed frame aligned to the leveled world frame used by the visualization topics
- `TF`
- `Odometry` on `/aft_mapped_to_init_level`
- `Path` on `/gvf/reference_path`
- `Marker` on `/gvf/reference_marker`

- [ ] **Step 4: Run test to verify it passes**

Run:

```bash
cd /home/morphing01/Drone_SLAM/overlay_ws
source /opt/ros/humble/setup.bash
pytest src/gvf_ismc_real_bringup/test/test_single_real_script_contract.py -q
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add \
  /home/morphing01/Drone_SLAM/scripts/gvf_ismc_reference.rviz \
  /home/morphing01/Drone_SLAM/overlay_ws/src/gvf_ismc_real_bringup/test/test_single_real_script_contract.py
git commit -m "feat: add gvf reference rviz config"
```

### Task 4: Full package verification

**Files:**
- Verify only

- [ ] **Step 1: Build the affected package**

Run:

```bash
cd /home/morphing01/Drone_SLAM/overlay_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select gvf_ismc_path_following gvf_ismc_real_bringup
```

Expected: exit code `0`.

- [ ] **Step 2: Run focused package tests**

Run:

```bash
cd /home/morphing01/Drone_SLAM/overlay_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select gvf_ismc_path_following gvf_ismc_real_bringup --event-handlers console_direct+
```

Expected: all tests pass.

- [ ] **Step 3: Confirm summarized results**

Run:

```bash
cd /home/morphing01/Drone_SLAM/overlay_ws
source /opt/ros/humble/setup.bash
colcon test-result --verbose --test-result-base build/gvf_ismc_path_following/test_results
colcon test-result --verbose --test-result-base build/gvf_ismc_real_bringup/test_results
```

Expected:

- `gvf_ismc_path_following`: `0 failures`
- `gvf_ismc_real_bringup`: `0 failures`

