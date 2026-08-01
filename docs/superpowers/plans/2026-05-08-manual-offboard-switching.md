# Manual Offboard Switching Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `gvf_ismc_real_bringup` stream Offboard setpoints without automatically requesting `Offboard` or `Arm`, so the operator controls the final mode switch.

**Architecture:** Add a bridge parameter that disables auto-request behavior while preserving setpoint streaming. Wire `gvf_ismc_real_bringup` to manual mode by default, and update tests and runbook so the documented flight flow matches the code.

**Tech Stack:** ROS 2 Humble, `rclcpp`, C++ gtests, pytest launch/contract tests, YAML config

---

### Task 1: Add failing bridge tests for manual Offboard mode

**Files:**
- Modify: `overlay_ws/src/position_cmd_to_px4_bridge/test/test_bridge_state_machine.cpp`
- Modify: `overlay_ws/src/position_cmd_to_px4_bridge/test/test_config_contract.cpp`
- Modify: `overlay_ws/src/gvf_ismc_real_bringup/test/test_single_real_launch_contract.py`

- [ ] **Step 1: Write the failing tests**

Add a state-machine test that expects no `request_offboard` or `request_arm` when auto mode is disabled.

Add config-contract assertions that:

```cpp
config.find("auto_request_offboard_and_arm: true")
node_source.find("declare_parameter<bool>(\"auto_request_offboard_and_arm\", true)")
```

Add bringup launch assertions that:

```python
assert params["auto_request_offboard_and_arm"] is False
```

- [ ] **Step 2: Run tests to verify they fail**

Run:

```bash
cd /home/morphing01/Drone_SLAM/overlay_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select position_cmd_to_px4_bridge gvf_ismc_real_bringup --ctest-args -R "bridge_state_machine|config_contract|single_real_launch_contract" --output-on-failure
```

Expected: failures because the new parameter and manual-mode behavior do not exist yet.

- [ ] **Step 3: Commit**

Do not commit yet. Continue to implementation.

### Task 2: Implement bridge parameter to disable auto Offboard/Arm requests

**Files:**
- Modify: `overlay_ws/src/position_cmd_to_px4_bridge/include/position_cmd_to_px4_bridge/bridge_state_machine.hpp`
- Modify: `overlay_ws/src/position_cmd_to_px4_bridge/src/bridge_state_machine.cpp`
- Modify: `overlay_ws/src/position_cmd_to_px4_bridge/src/position_cmd_to_px4_bridge_node.cpp`
- Modify: `overlay_ws/src/position_cmd_to_px4_bridge/config/position_cmd_to_px4_bridge.yaml`
- Test: `overlay_ws/src/position_cmd_to_px4_bridge/test/test_bridge_state_machine.cpp`
- Test: `overlay_ws/src/position_cmd_to_px4_bridge/test/test_config_contract.cpp`

- [ ] **Step 1: Write minimal implementation**

Add `auto_request_offboard_and_arm` to:

- bridge config yaml
- node parameter declarations

Thread the flag into the state machine, for example by extending the constructor.

Manual-mode behavior:

- continue `stream_setpoint`
- do not request `Offboard`
- do not request `Arm`
- enter `ACTIVE` only when `px4_offboard` and `px4_armed` are already true

Auto mode keeps current behavior.

- [ ] **Step 2: Run focused tests to verify they pass**

Run:

```bash
cd /home/morphing01/Drone_SLAM/overlay_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select position_cmd_to_px4_bridge --ctest-args -R "bridge_state_machine|config_contract" --output-on-failure
```

Expected: all targeted bridge tests pass.

- [ ] **Step 3: Commit**

```bash
git add \
  /home/morphing01/Drone_SLAM/overlay_ws/src/position_cmd_to_px4_bridge/include/position_cmd_to_px4_bridge/bridge_state_machine.hpp \
  /home/morphing01/Drone_SLAM/overlay_ws/src/position_cmd_to_px4_bridge/src/bridge_state_machine.cpp \
  /home/morphing01/Drone_SLAM/overlay_ws/src/position_cmd_to_px4_bridge/src/position_cmd_to_px4_bridge_node.cpp \
  /home/morphing01/Drone_SLAM/overlay_ws/src/position_cmd_to_px4_bridge/config/position_cmd_to_px4_bridge.yaml \
  /home/morphing01/Drone_SLAM/overlay_ws/src/position_cmd_to_px4_bridge/test/test_bridge_state_machine.cpp \
  /home/morphing01/Drone_SLAM/overlay_ws/src/position_cmd_to_px4_bridge/test/test_config_contract.cpp
git commit -m "feat: support manual offboard switching"
```

### Task 3: Set GVF real bringup to manual Offboard mode

**Files:**
- Modify: `overlay_ws/src/gvf_ismc_real_bringup/launch/single_real.launch.py`
- Modify: `overlay_ws/src/gvf_ismc_real_bringup/test/test_single_real_launch_contract.py`

- [ ] **Step 1: Write minimal implementation**

Add the bridge override:

```python
"auto_request_offboard_and_arm": False,
```

Keep the existing velocity-mode and acceleration-feedforward overrides intact.

- [ ] **Step 2: Run focused launch contract test**

Run:

```bash
cd /home/morphing01/Drone_SLAM/overlay_ws
source /opt/ros/humble/setup.bash
python3 -m pytest src/gvf_ismc_real_bringup/test/test_single_real_launch_contract.py -q
```

Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add \
  /home/morphing01/Drone_SLAM/overlay_ws/src/gvf_ismc_real_bringup/launch/single_real.launch.py \
  /home/morphing01/Drone_SLAM/overlay_ws/src/gvf_ismc_real_bringup/test/test_single_real_launch_contract.py
git commit -m "fix: default gvf real bringup to manual offboard"
```

### Task 4: Update runbook to match manual Offboard workflow

**Files:**
- Modify: `docs/runbooks/gvf-ismc-single-real.md`

- [ ] **Step 1: Update the workflow text**

Make the runbook explicitly state:

- launch only prepares setpoints
- user manually takes off in `Position`
- user manually switches to `Offboard`
- bridge no longer auto-arms or auto-switches modes in the GVF real-hardware path

- [ ] **Step 2: Commit**

```bash
git add /home/morphing01/Drone_SLAM/docs/runbooks/gvf-ismc-single-real.md
git commit -m "docs: align gvf runbook with manual offboard flow"
```

### Task 5: Full verification

**Files:**
- Verify only

- [ ] **Step 1: Build affected packages**

Run:

```bash
cd /home/morphing01/Drone_SLAM/overlay_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select position_cmd_to_px4_bridge gvf_ismc_real_bringup
```

- [ ] **Step 2: Run affected package tests**

Run:

```bash
cd /home/morphing01/Drone_SLAM/overlay_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select position_cmd_to_px4_bridge gvf_ismc_real_bringup --event-handlers console_direct+
```

- [ ] **Step 3: Confirm summary output**

Run:

```bash
cd /home/morphing01/Drone_SLAM/overlay_ws
source /opt/ros/humble/setup.bash
colcon test-result --verbose --test-result-base build/position_cmd_to_px4_bridge/test_results
colcon test-result --verbose --test-result-base build/gvf_ismc_real_bringup/test_results
```

Expected:

- `position_cmd_to_px4_bridge`: `0 failures`
- `gvf_ismc_real_bringup`: `0 failures`
