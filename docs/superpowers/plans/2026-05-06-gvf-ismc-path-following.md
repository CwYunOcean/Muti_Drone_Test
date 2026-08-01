# GVF + ISMC Path-Following Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a fixed-height three-leaf real-flight control path that reuses the current `FAST-LIO2 + PX4 offboard` pipeline, adds a `GVF` reference generator, adds an `ISMC` outer-loop tracker, and bypasses `ego_planner/traj_server`.

**Architecture:** Keep SLAM, leveling, PX4 odometry injection, and the `PositionCommand -> PX4` bridge in place. Add one small message package for `GVFReference`, add one C++ package that owns the three-leaf GVF math and ISMC outer loop, add one dedicated real-hardware bringup package, and minimally extend `position_cmd_to_px4_bridge` so it can run in velocity-dominant offboard mode for vector-field experiments.

**Tech Stack:** ROS 2 Humble, `ament_cmake`, `ament_cmake_gtest`, `ament_cmake_pytest`, `rosidl_default_generators`, C++, Python launch files, `rclcpp`, `nav_msgs`, `geometry_msgs`, `quadrotor_msgs`, `px4_msgs`, `Eigen3`.

---

## File Structure

- `overlay_ws/src/position_cmd_to_px4_bridge/`
  Owns the existing PX4 offboard bridge. This plan adds explicit `position_control_enabled` and `velocity_control_enabled` flags so the vector-field stack can publish velocity-dominant `PositionCommand` messages without relying on undocumented defaults.

- `overlay_ws/src/gvf_path_following_msgs/`
  New interface package. Owns `GVFReference.msg` and nothing else.

- `overlay_ws/src/gvf_ismc_path_following/`
  New controller package. Owns:
  - three-leaf implicit geometry and GVF computation
  - ISMC outer-loop math
  - `gvf_reference_node`
  - `ismc_velocity_tracker_node`
  - one package-local YAML config
  - unit and contract tests

- `overlay_ws/src/gvf_ismc_real_bringup/`
  New bringup wrapper package. Owns:
  - dedicated `single_real.launch.py`
  - launch contract tests
  - script contract tests

- `scripts/run_gvf_ismc_single_real.sh`
  Operator entry point for the controller stack after `FAST-LIO2` and `MicroXRCEAgent` are already running.

- `docs/runbooks/gvf-ismc-single-real.md`
  Operator runbook for bench checks, first takeover, low-speed three-leaf flight, and data capture.

### Task 1: Extend `position_cmd_to_px4_bridge` for velocity-dominant offboard mode

**Files:**
- Create: `overlay_ws/src/position_cmd_to_px4_bridge/include/position_cmd_to_px4_bridge/offboard_control_mode_builder.hpp`
- Create: `overlay_ws/src/position_cmd_to_px4_bridge/src/offboard_control_mode_builder.cpp`
- Create: `overlay_ws/src/position_cmd_to_px4_bridge/test/test_offboard_control_mode_builder.cpp`
- Modify: `overlay_ws/src/position_cmd_to_px4_bridge/CMakeLists.txt`
- Modify: `overlay_ws/src/position_cmd_to_px4_bridge/config/position_cmd_to_px4_bridge.yaml`
- Modify: `overlay_ws/src/position_cmd_to_px4_bridge/src/position_cmd_to_px4_bridge_node.cpp`
- Modify: `overlay_ws/src/position_cmd_to_px4_bridge/test/test_config_contract.cpp`

- [ ] **Step 1: Write the failing tests for explicit offboard-control-mode selection**

```cpp
// overlay_ws/src/position_cmd_to_px4_bridge/test/test_offboard_control_mode_builder.cpp
#include <gtest/gtest.h>

#include "position_cmd_to_px4_bridge/offboard_control_mode_builder.hpp"

using position_cmd_to_px4_bridge::build_offboard_control_mode;

TEST(OffboardControlModeBuilder, SupportsVelocityDominantControlMode)
{
  const auto msg = build_offboard_control_mode(
    1234U,
    false,
    true,
    false);

  EXPECT_FALSE(msg.position);
  EXPECT_TRUE(msg.velocity);
  EXPECT_FALSE(msg.acceleration);
  EXPECT_EQ(msg.timestamp, 1234U);
}

TEST(OffboardControlModeBuilder, EnablesAccelerationWhenRequested)
{
  const auto msg = build_offboard_control_mode(
    5678U,
    false,
    true,
    true);

  EXPECT_FALSE(msg.position);
  EXPECT_TRUE(msg.velocity);
  EXPECT_TRUE(msg.acceleration);
}
```

```cpp
// overlay_ws/src/position_cmd_to_px4_bridge/test/test_config_contract.cpp
TEST(ConfigContract, ControlModeDefaultsRemainExplicit)
{
  const auto package_root =
    std::filesystem::path{__FILE__}.parent_path().parent_path();
  const auto config_path = package_root / "config" / "position_cmd_to_px4_bridge.yaml";
  const auto node_path = package_root / "src" / "position_cmd_to_px4_bridge_node.cpp";

  const std::string config = read_file(config_path);
  const std::string node_source = read_file(node_path);

  EXPECT_NE(config.find("position_control_enabled: true"), std::string::npos);
  EXPECT_NE(config.find("velocity_control_enabled: true"), std::string::npos);
  EXPECT_NE(
    node_source.find("declare_parameter<bool>(\"position_control_enabled\", true)"),
    std::string::npos);
  EXPECT_NE(
    node_source.find("declare_parameter<bool>(\"velocity_control_enabled\", true)"),
    std::string::npos);
}
```

- [ ] **Step 2: Run the targeted tests to verify they fail before the helper exists**

Run:

```bash
source /opt/ros/humble/setup.bash
cd /home/morphing01/Drone_SLAM/overlay_ws
colcon build --symlink-install --packages-select position_cmd_to_px4_bridge
colcon test --packages-select position_cmd_to_px4_bridge \
  --ctest-args -R "offboard_control_mode_builder|config_contract" --output-on-failure
```

Expected:

- build or test failure because `offboard_control_mode_builder.hpp` and `build_offboard_control_mode(...)` do not exist yet
- config contract failure because the YAML and node source do not yet declare `position_control_enabled` or `velocity_control_enabled`

- [ ] **Step 3: Implement the helper, wire it into the node, and add the explicit config defaults**

```cpp
// overlay_ws/src/position_cmd_to_px4_bridge/include/position_cmd_to_px4_bridge/offboard_control_mode_builder.hpp
#pragma once

#include <cstdint>

#include <px4_msgs/msg/offboard_control_mode.hpp>

namespace position_cmd_to_px4_bridge
{

px4_msgs::msg::OffboardControlMode build_offboard_control_mode(
  uint64_t timestamp,
  bool position_enabled,
  bool velocity_enabled,
  bool acceleration_enabled);

}  // namespace position_cmd_to_px4_bridge
```

```cpp
// overlay_ws/src/position_cmd_to_px4_bridge/src/offboard_control_mode_builder.cpp
#include "position_cmd_to_px4_bridge/offboard_control_mode_builder.hpp"

namespace position_cmd_to_px4_bridge
{

px4_msgs::msg::OffboardControlMode build_offboard_control_mode(
  uint64_t timestamp,
  bool position_enabled,
  bool velocity_enabled,
  bool acceleration_enabled)
{
  px4_msgs::msg::OffboardControlMode msg{};
  msg.timestamp = timestamp;
  msg.position = position_enabled;
  msg.velocity = velocity_enabled;
  msg.acceleration = acceleration_enabled;
  msg.attitude = false;
  msg.body_rate = false;
  return msg;
}

}  // namespace position_cmd_to_px4_bridge
```

```cpp
// overlay_ws/src/position_cmd_to_px4_bridge/src/position_cmd_to_px4_bridge_node.cpp
#include "position_cmd_to_px4_bridge/offboard_control_mode_builder.hpp"

// inside constructor
position_control_enabled_ =
  declare_parameter<bool>("position_control_enabled", true);
velocity_control_enabled_ =
  declare_parameter<bool>("velocity_control_enabled", true);

// replace publish_offboard_control_mode()
void publish_offboard_control_mode() const
{
  offboard_mode_pub_->publish(
    build_offboard_control_mode(
      now_micros(),
      position_control_enabled_,
      velocity_control_enabled_,
      use_acceleration_feedforward_));
}
```

```yaml
# overlay_ws/src/position_cmd_to_px4_bridge/config/position_cmd_to_px4_bridge.yaml
/position_cmd_to_px4_bridge:
  ros__parameters:
    enabled: true
    command_topic: "/drone_0_planning/pos_cmd"
    offboard_mode_topic: "/fmu/in/offboard_control_mode"
    trajectory_setpoint_topic: "/fmu/in/trajectory_setpoint"
    vehicle_command_topic: "/fmu/in/vehicle_command"
    vehicle_odometry_topic: "/fmu/out/vehicle_odometry"
    vehicle_status_topic: ""
    fastlivo_odom_topic: "/aft_mapped_to_init"
    warmup_cycles: 10
    command_timeout_ms: 300
    status_timeout_ms: 1000
    odom_timeout_ms: 300
    position_control_enabled: true
    velocity_control_enabled: true
    use_acceleration_feedforward: false
    world_axis: [1, 0, 2]
    world_sign: [1.0, 1.0, -1.0]
```

```cmake
# overlay_ws/src/position_cmd_to_px4_bridge/CMakeLists.txt
add_library(${PROJECT_NAME}_core
  src/bridge_input_builder.cpp
  src/offboard_control_mode_builder.cpp
  src/trajectory_setpoint_conversion.cpp
  src/bridge_state_machine.cpp
)

ament_add_gtest(
  test_offboard_control_mode_builder
  test/test_offboard_control_mode_builder.cpp
)
target_link_libraries(test_offboard_control_mode_builder
  ${PROJECT_NAME}_core
)
ament_target_dependencies(test_offboard_control_mode_builder
  px4_msgs
)
```

- [ ] **Step 4: Rebuild the bridge package and rerun the targeted tests**

Run:

```bash
source /opt/ros/humble/setup.bash
cd /home/morphing01/Drone_SLAM/overlay_ws
colcon build --symlink-install --packages-select position_cmd_to_px4_bridge
colcon test --packages-select position_cmd_to_px4_bridge \
  --ctest-args -R "offboard_control_mode_builder|config_contract" --output-on-failure
cd /home/morphing01/Drone_SLAM/overlay_ws
colcon test-result --verbose --test-result-base build/position_cmd_to_px4_bridge/test_results
```

Expected:

- `test_offboard_control_mode_builder`: PASS
- `test_config_contract`: PASS

- [ ] **Step 5: Commit the bridge-mode support**

```bash
git add \
  overlay_ws/src/position_cmd_to_px4_bridge/CMakeLists.txt \
  overlay_ws/src/position_cmd_to_px4_bridge/config/position_cmd_to_px4_bridge.yaml \
  overlay_ws/src/position_cmd_to_px4_bridge/include/position_cmd_to_px4_bridge/offboard_control_mode_builder.hpp \
  overlay_ws/src/position_cmd_to_px4_bridge/src/offboard_control_mode_builder.cpp \
  overlay_ws/src/position_cmd_to_px4_bridge/src/position_cmd_to_px4_bridge_node.cpp \
  overlay_ws/src/position_cmd_to_px4_bridge/test/test_config_contract.cpp \
  overlay_ws/src/position_cmd_to_px4_bridge/test/test_offboard_control_mode_builder.cpp
git commit -m "feat: add explicit px4 offboard mode selection"
```

### Task 2: Create the `GVFReference` message package

**Files:**
- Create: `overlay_ws/src/gvf_path_following_msgs/CMakeLists.txt`
- Create: `overlay_ws/src/gvf_path_following_msgs/package.xml`
- Create: `overlay_ws/src/gvf_path_following_msgs/msg/GVFReference.msg`
- Create: `overlay_ws/src/gvf_path_following_msgs/test/test_message_contract.py`

- [ ] **Step 1: Create the package skeleton and a failing contract test before the message exists**

```cmake
# overlay_ws/src/gvf_path_following_msgs/CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(gvf_path_following_msgs)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GVFReference.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

if(BUILD_TESTING)
  ament_add_pytest_test(
    test_message_contract
    test/test_message_contract.py
  )
endif()

ament_export_dependencies(geometry_msgs std_msgs)
ament_package()
```

```xml
<!-- overlay_ws/src/gvf_path_following_msgs/package.xml -->
<?xml version="1.0"?>
<package format="3">
  <name>gvf_path_following_msgs</name>
  <version>0.0.1</version>
  <description>Interfaces for GVF-guided path-following experiments.</description>
  <maintainer email="dev@example.com">Drone SLAM Maintainers</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  <depend>rosidl_default_generators</depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <test_depend>ament_cmake_pytest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

```python
# overlay_ws/src/gvf_path_following_msgs/test/test_message_contract.py
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MESSAGE_PATH = PACKAGE_ROOT / "msg" / "GVFReference.msg"


def test_gvf_reference_message_exists() -> None:
    assert MESSAGE_PATH.exists(), f"Missing message file: {MESSAGE_PATH}"


def test_gvf_reference_message_fields() -> None:
    message = MESSAGE_PATH.read_text(encoding="utf-8")
    assert "std_msgs/Header header" in message
    assert "geometry_msgs/Vector3 desired_velocity" in message
    assert "float64 desired_yaw" in message
    assert "float64 desired_yaw_rate" in message
    assert "float64 phi1" in message
    assert "float64 phi2" in message
```

- [ ] **Step 2: Run the package build to verify it fails without the message definition**

Run:

```bash
source /opt/ros/humble/setup.bash
cd /home/morphing01/Drone_SLAM/overlay_ws
colcon build --symlink-install --packages-select gvf_path_following_msgs
```

Expected:

- build failure because `msg/GVFReference.msg` does not exist yet

- [ ] **Step 3: Add the `GVFReference.msg` interface**

```text
# overlay_ws/src/gvf_path_following_msgs/msg/GVFReference.msg
std_msgs/Header header
geometry_msgs/Vector3 desired_velocity
float64 desired_yaw
float64 desired_yaw_rate
float64 phi1
float64 phi2
```

- [ ] **Step 4: Build and test the message package**

Run:

```bash
source /opt/ros/humble/setup.bash
cd /home/morphing01/Drone_SLAM/overlay_ws
colcon build --symlink-install --packages-select gvf_path_following_msgs
colcon test --packages-select gvf_path_following_msgs --event-handlers console_direct+
cd /home/morphing01/Drone_SLAM/overlay_ws
colcon test-result --verbose --test-result-base build/gvf_path_following_msgs/test_results
```

Expected:

- package builds successfully
- `test_message_contract`: PASS

- [ ] **Step 5: Commit the new interface package**

```bash
git add \
  overlay_ws/src/gvf_path_following_msgs/CMakeLists.txt \
  overlay_ws/src/gvf_path_following_msgs/package.xml \
  overlay_ws/src/gvf_path_following_msgs/msg/GVFReference.msg \
  overlay_ws/src/gvf_path_following_msgs/test/test_message_contract.py
git commit -m "feat: add gvf path-following reference message"
```

### Task 3: Implement the three-leaf GVF core package

**Files:**
- Create: `overlay_ws/src/gvf_ismc_path_following/CMakeLists.txt`
- Create: `overlay_ws/src/gvf_ismc_path_following/package.xml`
- Create: `overlay_ws/src/gvf_ismc_path_following/config/gvf_ismc_path_following.yaml`
- Create: `overlay_ws/src/gvf_ismc_path_following/include/gvf_ismc_path_following/three_leaf_gvf.hpp`
- Create: `overlay_ws/src/gvf_ismc_path_following/src/three_leaf_gvf.cpp`
- Create: `overlay_ws/src/gvf_ismc_path_following/test/test_three_leaf_gvf.cpp`
- Create: `overlay_ws/src/gvf_ismc_path_following/test/test_config_contract.py`

- [ ] **Step 1: Create a failing GVF unit test and config contract before the math implementation exists**

```cpp
// overlay_ws/src/gvf_ismc_path_following/test/test_three_leaf_gvf.cpp
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "gvf_ismc_path_following/three_leaf_gvf.hpp"

using gvf_ismc_path_following::ThreeLeafGvfParameters;
using gvf_ismc_path_following::evaluate_three_leaf_gvf;

TEST(ThreeLeafGvf, EvaluatesZeroImplicitErrorOnNominalCurve)
{
  ThreeLeafGvfParameters params{};
  params.target_height_m = 1.2;
  params.base_radius_m = 2.0;
  params.lobe_amplitude_m = 0.5;

  const auto result = evaluate_three_leaf_gvf(
    Eigen::Vector3d(2.5, 0.0, 1.2),
    0.0,
    params);

  EXPECT_NEAR(result.phi1, 0.0, 1e-6);
  EXPECT_NEAR(result.phi2, 0.0, 1e-6);
}

TEST(ThreeLeafGvf, CommandsNegativeVerticalVelocityAboveTargetHeight)
{
  ThreeLeafGvfParameters params{};
  params.target_height_m = 1.2;
  params.base_radius_m = 2.0;
  params.lobe_amplitude_m = 0.5;
  params.vertical_gain = 1.0;

  const auto result = evaluate_three_leaf_gvf(
    Eigen::Vector3d(2.5, 0.0, 1.7),
    0.2,
    params);

  EXPECT_LT(result.desired_velocity.z(), 0.0);
}

TEST(ThreeLeafGvf, HoldsPreviousYawWhenPlanarSpeedIsTiny)
{
  ThreeLeafGvfParameters params{};
  params.target_height_m = 0.0;
  params.base_radius_m = 2.0;
  params.lobe_amplitude_m = 1.0;
  params.min_planar_speed_for_yaw_mps = 10.0;

  const auto result = evaluate_three_leaf_gvf(
    Eigen::Vector3d(2.0, 0.0, 0.0),
    1.0,
    params);

  EXPECT_NEAR(result.desired_yaw, 1.0, 1e-6);
}
```

```python
# overlay_ws/src/gvf_ismc_path_following/test/test_config_contract.py
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PACKAGE_ROOT / "config" / "gvf_ismc_path_following.yaml"


def test_config_exists() -> None:
    assert CONFIG_PATH.exists(), f"Missing config file: {CONFIG_PATH}"


def test_gvf_defaults_match_first_flight_scope() -> None:
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    params = config["/gvf_reference_node"]["ros__parameters"]

    assert params["odom_topic"] == "/aft_mapped_to_init_level"
    assert params["reference_topic"] == "/gvf/reference"
    assert params["target_height_m"] == 1.2
    assert params["base_radius_m"] == 2.0
    assert params["lobe_amplitude_m"] == 0.5
    assert params["max_speed_mps"] == 0.5
```

- [ ] **Step 2: Run the package build to verify the tests fail before the header and config exist**

Run:

```bash
source /opt/ros/humble/setup.bash
cd /home/morphing01/Drone_SLAM/overlay_ws
colcon build --symlink-install --packages-select gvf_ismc_path_following
```

Expected:

- build failure because the package, header, config, and library sources do not exist yet

- [ ] **Step 3: Implement the package metadata, config, and GVF core**

```cmake
# overlay_ws/src/gvf_ismc_path_following/CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(gvf_ismc_path_following)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME}_core
  src/three_leaf_gvf.cpp
)
target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core
  Eigen3::Eigen
)

install(
  TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  ament_add_gtest(
    test_three_leaf_gvf
    test/test_three_leaf_gvf.cpp
  )
  target_link_libraries(test_three_leaf_gvf
    ${PROJECT_NAME}_core
    Eigen3::Eigen
  )

  ament_add_pytest_test(
    test_config_contract
    test/test_config_contract.py
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(Eigen3)
ament_package()
```

```xml
<!-- overlay_ws/src/gvf_ismc_path_following/package.xml -->
<?xml version="1.0"?>
<package format="3">
  <name>gvf_ismc_path_following</name>
  <version>0.0.1</version>
  <description>GVF guidance and ISMC outer-loop tracking for real-flight path following.</description>
  <maintainer email="dev@example.com">Drone SLAM Maintainers</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <build_depend>eigen3_cmake_module</build_depend>
  <build_depend>Eigen3</build_depend>

  <test_depend>ament_cmake_gtest</test_depend>
  <test_depend>ament_cmake_pytest</test_depend>
  <test_depend>python3-yaml</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

```yaml
# overlay_ws/src/gvf_ismc_path_following/config/gvf_ismc_path_following.yaml
/gvf_reference_node:
  ros__parameters:
    odom_topic: "/aft_mapped_to_init_level"
    reference_topic: "/gvf/reference"
    target_height_m: 1.2
    base_radius_m: 2.0
    lobe_amplitude_m: 0.5
    lateral_gain: 1.0
    vertical_gain: 1.0
    max_speed_mps: 0.5
    min_planar_speed_for_yaw_mps: 0.05
    yaw_alpha: 0.1
```

```cpp
// overlay_ws/src/gvf_ismc_path_following/include/gvf_ismc_path_following/three_leaf_gvf.hpp
#pragma once

#include <Eigen/Core>

namespace gvf_ismc_path_following
{

struct ThreeLeafGvfParameters
{
  double target_height_m{1.2};
  double base_radius_m{2.0};
  double lobe_amplitude_m{0.5};
  double lateral_gain{1.0};
  double vertical_gain{1.0};
  double max_speed_mps{0.5};
  double min_planar_speed_for_yaw_mps{0.05};
  double yaw_alpha{0.1};
};

struct ThreeLeafGvfResult
{
  double phi1{0.0};
  double phi2{0.0};
  Eigen::Vector3d desired_velocity{Eigen::Vector3d::Zero()};
  double desired_yaw{0.0};
};

ThreeLeafGvfResult evaluate_three_leaf_gvf(
  const Eigen::Vector3d & position,
  double previous_yaw,
  const ThreeLeafGvfParameters & params);

}  // namespace gvf_ismc_path_following
```

```cpp
// overlay_ws/src/gvf_ismc_path_following/src/three_leaf_gvf.cpp
#include "gvf_ismc_path_following/three_leaf_gvf.hpp"

#include <algorithm>
#include <cmath>

namespace gvf_ismc_path_following
{

namespace
{

Eigen::Vector3d clamp_norm(const Eigen::Vector3d & value, double max_norm)
{
  if (max_norm <= 0.0) {
    return value;
  }
  const double norm = value.norm();
  if (norm <= max_norm || norm < 1e-9) {
    return value;
  }
  return value * (max_norm / norm);
}

double wrap_to_pi(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

}  // namespace

ThreeLeafGvfResult evaluate_three_leaf_gvf(
  const Eigen::Vector3d & position,
  double previous_yaw,
  const ThreeLeafGvfParameters & params)
{
  const double x = position.x();
  const double y = position.y();
  const double z = position.z();
  const double r = std::hypot(x, y);
  const double theta = std::atan2(y, x);
  const double denom = std::max(1e-9, x * x + y * y);

  const double phi1 = z - params.target_height_m;
  const double phi2 = r - (params.base_radius_m + params.lobe_amplitude_m * std::cos(3.0 * theta));

  Eigen::Vector3d n2(0.0, 0.0, 1.0);
  Eigen::Vector3d n1(0.0, 0.0, 0.0);
  if (r >= 1e-9) {
    const double drdx = x / r;
    const double drdy = y / r;
    const double dthdx = -y / denom;
    const double dthdy = x / denom;
    n1.x() = drdx + 3.0 * params.lobe_amplitude_m * std::sin(3.0 * theta) * dthdx;
    n1.y() = drdy + 3.0 * params.lobe_amplitude_m * std::sin(3.0 * theta) * dthdy;
  }

  Eigen::Vector3d desired_velocity =
    n1.cross(n2) -
    params.lateral_gain * phi2 * n1 -
    params.vertical_gain * phi1 * n2;
  desired_velocity = clamp_norm(desired_velocity, params.max_speed_mps);

  double desired_yaw = previous_yaw;
  const double planar_speed = std::hypot(desired_velocity.x(), desired_velocity.y());
  if (planar_speed >= params.min_planar_speed_for_yaw_mps) {
    const double raw_yaw = std::atan2(desired_velocity.y(), desired_velocity.x());
    const double delta = wrap_to_pi(raw_yaw - previous_yaw);
    desired_yaw = wrap_to_pi(previous_yaw + params.yaw_alpha * delta);
  }

  ThreeLeafGvfResult result{};
  result.phi1 = phi1;
  result.phi2 = phi2;
  result.desired_velocity = desired_velocity;
  result.desired_yaw = desired_yaw;
  return result;
}

}  // namespace gvf_ismc_path_following
```

- [ ] **Step 4: Build and run the GVF package tests**

Run:

```bash
source /opt/ros/humble/setup.bash
cd /home/morphing01/Drone_SLAM/overlay_ws
colcon build --symlink-install --packages-select gvf_ismc_path_following
colcon test --packages-select gvf_ismc_path_following --event-handlers console_direct+
cd /home/morphing01/Drone_SLAM/overlay_ws
colcon test-result --verbose --test-result-base build/gvf_ismc_path_following/test_results
```

Expected:

- `test_three_leaf_gvf`: PASS
- `test_config_contract`: PASS

- [ ] **Step 5: Commit the GVF core package**

```bash
git add \
  overlay_ws/src/gvf_ismc_path_following/CMakeLists.txt \
  overlay_ws/src/gvf_ismc_path_following/package.xml \
  overlay_ws/src/gvf_ismc_path_following/config/gvf_ismc_path_following.yaml \
  overlay_ws/src/gvf_ismc_path_following/include/gvf_ismc_path_following/three_leaf_gvf.hpp \
  overlay_ws/src/gvf_ismc_path_following/src/three_leaf_gvf.cpp \
  overlay_ws/src/gvf_ismc_path_following/test/test_three_leaf_gvf.cpp \
  overlay_ws/src/gvf_ismc_path_following/test/test_config_contract.py
git commit -m "feat: add three-leaf gvf core package"
```

### Task 4: Implement the ISMC outer loop and the two controller nodes

**Files:**
- Create: `overlay_ws/src/gvf_ismc_path_following/include/gvf_ismc_path_following/ismc_outer_loop.hpp`
- Create: `overlay_ws/src/gvf_ismc_path_following/src/ismc_outer_loop.cpp`
- Create: `overlay_ws/src/gvf_ismc_path_following/src/gvf_reference_node.cpp`
- Create: `overlay_ws/src/gvf_ismc_path_following/src/ismc_velocity_tracker_node.cpp`
- Create: `overlay_ws/src/gvf_ismc_path_following/test/test_ismc_outer_loop.cpp`
- Create: `overlay_ws/src/gvf_ismc_path_following/test/test_node_contract.py`
- Modify: `overlay_ws/src/gvf_ismc_path_following/CMakeLists.txt`
- Modify: `overlay_ws/src/gvf_ismc_path_following/package.xml`
- Modify: `overlay_ws/src/gvf_ismc_path_following/config/gvf_ismc_path_following.yaml`

- [ ] **Step 1: Write the failing ISMC and node contract tests**

```cpp
// overlay_ws/src/gvf_ismc_path_following/test/test_ismc_outer_loop.cpp
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "gvf_ismc_path_following/ismc_outer_loop.hpp"

using gvf_ismc_path_following::IsmcOuterLoopParameters;
using gvf_ismc_path_following::IsmcOuterLoopState;
using gvf_ismc_path_following::step_ismc_outer_loop;

TEST(IsmcOuterLoop, ReturnsZeroAccelerationForZeroVelocityError)
{
  IsmcOuterLoopParameters params{};
  IsmcOuterLoopState state{};

  const auto output = step_ismc_outer_loop(
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(),
    0.02,
    params,
    &state);

  EXPECT_NEAR(output.acceleration_command.norm(), 0.0, 1e-9);
}

TEST(IsmcOuterLoop, CommandsPositiveAccelerationForPositiveXVelocityError)
{
  IsmcOuterLoopParameters params{};
  IsmcOuterLoopState state{};

  const auto output = step_ismc_outer_loop(
    Eigen::Vector3d(0.4, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    0.02,
    params,
    &state);

  EXPECT_GT(output.acceleration_command.x(), 0.0);
}

TEST(IsmcOuterLoop, RespectsAccelerationLimits)
{
  IsmcOuterLoopParameters params{};
  params.max_acc_mps2 = Eigen::Vector3d(0.2, 0.2, 0.2);
  IsmcOuterLoopState state{};

  const auto output = step_ismc_outer_loop(
    Eigen::Vector3d(5.0, 0.0, 0.0),
    Eigen::Vector3d::Zero(),
    0.02,
    params,
    &state);

  EXPECT_LE(output.acceleration_command.cwiseAbs().maxCoeff(), 0.2 + 1e-9);
}
```

```python
# overlay_ws/src/gvf_ismc_path_following/test/test_node_contract.py
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_reference_node_uses_gvf_reference_topic() -> None:
    source = (PACKAGE_ROOT / "src" / "gvf_reference_node.cpp").read_text(encoding="utf-8")
    assert 'declare_parameter<std::string>("reference_topic", "/gvf/reference")' in source
    assert "create_publisher<gvf_path_following_msgs::msg::GVFReference>" in source


def test_tracker_node_publishes_position_command() -> None:
    source = (PACKAGE_ROOT / "src" / "ismc_velocity_tracker_node.cpp").read_text(encoding="utf-8")
    assert 'declare_parameter<std::string>("command_topic", "/drone_0_planning/pos_cmd")' in source
    assert "create_publisher<quadrotor_msgs::msg::PositionCommand>" in source
    assert "std::numeric_limits<double>::quiet_NaN()" in source
```

- [ ] **Step 2: Run the controller-package tests to verify they fail before the ISMC and node sources exist**

Run:

```bash
source /opt/ros/humble/setup.bash
cd /home/morphing01/Drone_SLAM/overlay_ws
colcon build --symlink-install --packages-select gvf_ismc_path_following
colcon test --packages-select gvf_ismc_path_following --event-handlers console_direct+
```

Expected:

- build or test failure because `ismc_outer_loop.hpp`, `ismc_outer_loop.cpp`, `gvf_reference_node.cpp`, and `ismc_velocity_tracker_node.cpp` do not exist yet

- [ ] **Step 3: Implement the ISMC library, add the two nodes, and expand the package metadata**

```cpp
// overlay_ws/src/gvf_ismc_path_following/include/gvf_ismc_path_following/ismc_outer_loop.hpp
#pragma once

#include <Eigen/Core>

namespace gvf_ismc_path_following
{

struct IsmcOuterLoopParameters
{
  Eigen::Vector3d mu{Eigen::Vector3d(0.3, 0.3, 0.3)};
  Eigen::Vector3d c1{Eigen::Vector3d(0.05, 0.05, 0.05)};
  Eigen::Vector3d k_slide{Eigen::Vector3d(0.1, 0.1, 0.1)};
  Eigen::Vector3d lambda{Eigen::Vector3d(0.05, 0.05, 0.05)};
  Eigen::Vector3d epsilon{Eigen::Vector3d(0.05, 0.05, 0.05)};
  Eigen::Vector3d max_acc_mps2{Eigen::Vector3d(0.8, 0.8, 0.6)};
  double gamma{0.0};
  bool enable_adaptation{false};
};

struct IsmcOuterLoopState
{
  Eigen::Vector3d adaptive_bias{Eigen::Vector3d::Zero()};
};

struct IsmcOuterLoopOutput
{
  Eigen::Vector3d acceleration_command{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_error{Eigen::Vector3d::Zero()};
  Eigen::Vector3d sliding_surface{Eigen::Vector3d::Zero()};
};

IsmcOuterLoopOutput step_ismc_outer_loop(
  const Eigen::Vector3d & desired_velocity,
  const Eigen::Vector3d & current_velocity,
  double dt_s,
  const IsmcOuterLoopParameters & params,
  IsmcOuterLoopState * state);

}  // namespace gvf_ismc_path_following
```

```cpp
// overlay_ws/src/gvf_ismc_path_following/src/ismc_outer_loop.cpp
#include "gvf_ismc_path_following/ismc_outer_loop.hpp"

#include <algorithm>
#include <cmath>

namespace gvf_ismc_path_following
{

namespace
{

Eigen::Vector3d saturate_surface(
  const Eigen::Vector3d & surface,
  const Eigen::Vector3d & epsilon)
{
  Eigen::Vector3d result = Eigen::Vector3d::Zero();
  for (int i = 0; i < 3; ++i) {
    const double denom = std::max(1e-6, epsilon[i]);
    result[i] = std::tanh(surface[i] / denom);
  }
  return result;
}

}  // namespace

IsmcOuterLoopOutput step_ismc_outer_loop(
  const Eigen::Vector3d & desired_velocity,
  const Eigen::Vector3d & current_velocity,
  double dt_s,
  const IsmcOuterLoopParameters & params,
  IsmcOuterLoopState * state)
{
  const double safe_dt = std::max(1e-3, dt_s);
  const Eigen::Vector3d velocity_error = desired_velocity - current_velocity;
  const Eigen::Vector3d error_rate = velocity_error / safe_dt;
  const Eigen::Vector3d sliding_surface =
    error_rate + params.mu.cwiseProduct(velocity_error);

  if (state != nullptr && params.enable_adaptation) {
    state->adaptive_bias += safe_dt * params.gamma * sliding_surface;
  }

  const Eigen::Vector3d adaptive_bias =
    state == nullptr ? Eigen::Vector3d::Zero() : state->adaptive_bias;
  Eigen::Vector3d acceleration_command =
    params.c1.cwiseProduct(error_rate) +
    params.k_slide.cwiseProduct(sliding_surface) +
    (params.lambda + params.epsilon).cwiseProduct(
      saturate_surface(sliding_surface, params.epsilon)) +
    adaptive_bias;

  for (int i = 0; i < 3; ++i) {
    acceleration_command[i] = std::clamp(
      acceleration_command[i],
      -params.max_acc_mps2[i],
      params.max_acc_mps2[i]);
  }

  IsmcOuterLoopOutput output{};
  output.acceleration_command = acceleration_command;
  output.velocity_error = velocity_error;
  output.sliding_surface = sliding_surface;
  return output;
}

}  // namespace gvf_ismc_path_following
```

```cpp
// overlay_ws/src/gvf_ismc_path_following/src/gvf_reference_node.cpp
#include <memory>
#include <string>

#include <geometry_msgs/msg/vector3.hpp>
#include <gvf_path_following_msgs/msg/gvf_reference.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "gvf_ismc_path_following/three_leaf_gvf.hpp"

namespace gvf_ismc_path_following
{

class GvfReferenceNode final : public rclcpp::Node
{
public:
  GvfReferenceNode()
  : Node("gvf_reference_node")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/aft_mapped_to_init_level");
    reference_topic_ = declare_parameter<std::string>("reference_topic", "/gvf/reference");
    params_.target_height_m = declare_parameter<double>("target_height_m", 1.2);
    params_.base_radius_m = declare_parameter<double>("base_radius_m", 2.0);
    params_.lobe_amplitude_m = declare_parameter<double>("lobe_amplitude_m", 0.5);
    params_.lateral_gain = declare_parameter<double>("lateral_gain", 1.0);
    params_.vertical_gain = declare_parameter<double>("vertical_gain", 1.0);
    params_.max_speed_mps = declare_parameter<double>("max_speed_mps", 0.5);
    params_.min_planar_speed_for_yaw_mps =
      declare_parameter<double>("min_planar_speed_for_yaw_mps", 0.05);
    params_.yaw_alpha = declare_parameter<double>("yaw_alpha", 0.1);

    publisher_ = create_publisher<gvf_path_following_msgs::msg::GVFReference>(
      reference_topic_, 20);
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20,
      std::bind(&GvfReferenceNode::handle_odom, this, std::placeholders::_1));
  }

private:
  void handle_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const Eigen::Vector3d position(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z);

    const auto result = evaluate_three_leaf_gvf(position, previous_yaw_, params_);
    previous_yaw_ = result.desired_yaw;

    gvf_path_following_msgs::msg::GVFReference out{};
    out.header = msg->header;
    out.desired_velocity.x = result.desired_velocity.x();
    out.desired_velocity.y = result.desired_velocity.y();
    out.desired_velocity.z = result.desired_velocity.z();
    out.desired_yaw = result.desired_yaw;
    out.desired_yaw_rate = 0.0;
    out.phi1 = result.phi1;
    out.phi2 = result.phi2;
    publisher_->publish(out);
  }

  ThreeLeafGvfParameters params_{};
  std::string odom_topic_;
  std::string reference_topic_;
  double previous_yaw_{0.0};
  rclcpp::Publisher<gvf_path_following_msgs::msg::GVFReference>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

}  // namespace gvf_ismc_path_following

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gvf_ismc_path_following::GvfReferenceNode>());
  rclcpp::shutdown();
  return 0;
}
```

```cpp
// overlay_ws/src/gvf_ismc_path_following/src/ismc_velocity_tracker_node.cpp
#include <limits>
#include <memory>
#include <string>

#include <gvf_path_following_msgs/msg/gvf_reference.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>

#include "gvf_ismc_path_following/ismc_outer_loop.hpp"

namespace gvf_ismc_path_following
{

class IsmcVelocityTrackerNode final : public rclcpp::Node
{
public:
  IsmcVelocityTrackerNode()
  : Node("ismc_velocity_tracker_node")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/aft_mapped_to_init_level");
    reference_topic_ = declare_parameter<std::string>("reference_topic", "/gvf/reference");
    command_topic_ =
      declare_parameter<std::string>("command_topic", "/drone_0_planning/pos_cmd");
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 50.0);
    publish_position_nan_ = declare_parameter<bool>("publish_position_nan", true);

    params_.mu = Eigen::Vector3d(0.3, 0.3, 0.3);
    params_.c1 = Eigen::Vector3d(0.05, 0.05, 0.05);
    params_.k_slide = Eigen::Vector3d(0.1, 0.1, 0.1);
    params_.lambda = Eigen::Vector3d(0.05, 0.05, 0.05);
    params_.epsilon = Eigen::Vector3d(0.05, 0.05, 0.05);
    params_.max_acc_mps2 = Eigen::Vector3d(0.8, 0.8, 0.6);
    params_.gamma = declare_parameter<double>("gamma", 0.0);
    params_.enable_adaptation = declare_parameter<bool>("enable_adaptation", false);

    command_pub_ = create_publisher<quadrotor_msgs::msg::PositionCommand>(
      command_topic_, 20);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20,
      std::bind(&IsmcVelocityTrackerNode::handle_odom, this, std::placeholders::_1));
    reference_sub_ = create_subscription<gvf_path_following_msgs::msg::GVFReference>(
      reference_topic_, 20,
      std::bind(&IsmcVelocityTrackerNode::handle_reference, this, std::placeholders::_1));

    const auto period_ms = static_cast<int>(1000.0 / control_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&IsmcVelocityTrackerNode::publish_command, this));
  }

private:
  void handle_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_ = msg;
  }

  void handle_reference(const gvf_path_following_msgs::msg::GVFReference::SharedPtr msg)
  {
    last_reference_ = msg;
  }

  void publish_command()
  {
    if (!last_odom_ || !last_reference_) {
      return;
    }

    const Eigen::Vector3d current_velocity(
      last_odom_->twist.twist.linear.x,
      last_odom_->twist.twist.linear.y,
      last_odom_->twist.twist.linear.z);
    const Eigen::Vector3d desired_velocity(
      last_reference_->desired_velocity.x,
      last_reference_->desired_velocity.y,
      last_reference_->desired_velocity.z);

    const auto output = step_ismc_outer_loop(
      desired_velocity,
      current_velocity,
      1.0 / control_rate_hz_,
      params_,
      &state_);

    quadrotor_msgs::msg::PositionCommand cmd{};
    cmd.header.stamp = now();
    cmd.header.frame_id = frame_id_;
    cmd.trajectory_id = 1;
    cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;

    const double nan = std::numeric_limits<double>::quiet_NaN();
    if (publish_position_nan_) {
      cmd.position.x = nan;
      cmd.position.y = nan;
      cmd.position.z = nan;
    } else {
      cmd.position.x = last_odom_->pose.pose.position.x;
      cmd.position.y = last_odom_->pose.pose.position.y;
      cmd.position.z = last_odom_->pose.pose.position.z;
    }

    cmd.velocity.x = desired_velocity.x();
    cmd.velocity.y = desired_velocity.y();
    cmd.velocity.z = desired_velocity.z();
    cmd.acceleration.x = output.acceleration_command.x();
    cmd.acceleration.y = output.acceleration_command.y();
    cmd.acceleration.z = output.acceleration_command.z();
    cmd.yaw = last_reference_->desired_yaw;
    cmd.yaw_dot = last_reference_->desired_yaw_rate;
    command_pub_->publish(cmd);
  }

  IsmcOuterLoopParameters params_{};
  IsmcOuterLoopState state_{};
  std::string odom_topic_;
  std::string reference_topic_;
  std::string command_topic_;
  std::string frame_id_;
  double control_rate_hz_{50.0};
  bool publish_position_nan_{true};
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  gvf_path_following_msgs::msg::GVFReference::SharedPtr last_reference_;
  rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<gvf_path_following_msgs::msg::GVFReference>::SharedPtr reference_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace gvf_ismc_path_following

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gvf_ismc_path_following::IsmcVelocityTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
```

```yaml
# overlay_ws/src/gvf_ismc_path_following/config/gvf_ismc_path_following.yaml
/gvf_reference_node:
  ros__parameters:
    odom_topic: "/aft_mapped_to_init_level"
    reference_topic: "/gvf/reference"
    target_height_m: 1.2
    base_radius_m: 2.0
    lobe_amplitude_m: 0.5
    lateral_gain: 1.0
    vertical_gain: 1.0
    max_speed_mps: 0.5
    min_planar_speed_for_yaw_mps: 0.05
    yaw_alpha: 0.1

/ismc_velocity_tracker_node:
  ros__parameters:
    odom_topic: "/aft_mapped_to_init_level"
    reference_topic: "/gvf/reference"
    command_topic: "/drone_0_planning/pos_cmd"
    frame_id: "world"
    control_rate_hz: 50.0
    publish_position_nan: true
    gamma: 0.0
    enable_adaptation: false
```

```cmake
# overlay_ws/src/gvf_ismc_path_following/CMakeLists.txt
find_package(geometry_msgs REQUIRED)
find_package(gvf_path_following_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME}_core
  src/three_leaf_gvf.cpp
  src/ismc_outer_loop.cpp
)

add_executable(gvf_reference_node
  src/gvf_reference_node.cpp
)
target_link_libraries(gvf_reference_node
  ${PROJECT_NAME}_core
  Eigen3::Eigen
)
ament_target_dependencies(gvf_reference_node
  gvf_path_following_msgs
  nav_msgs
  rclcpp
)

add_executable(ismc_velocity_tracker_node
  src/ismc_velocity_tracker_node.cpp
)
target_link_libraries(ismc_velocity_tracker_node
  ${PROJECT_NAME}_core
  Eigen3::Eigen
)
ament_target_dependencies(ismc_velocity_tracker_node
  gvf_path_following_msgs
  nav_msgs
  quadrotor_msgs
  rclcpp
)

install(
  TARGETS ${PROJECT_NAME}_core gvf_reference_node ismc_velocity_tracker_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_add_gtest(
  test_ismc_outer_loop
  test/test_ismc_outer_loop.cpp
)
target_link_libraries(test_ismc_outer_loop
  ${PROJECT_NAME}_core
  Eigen3::Eigen
)

ament_add_pytest_test(
  test_node_contract
  test/test_node_contract.py
)
```

```xml
<!-- overlay_ws/src/gvf_ismc_path_following/package.xml -->
<depend>geometry_msgs</depend>
<depend>gvf_path_following_msgs</depend>
<depend>nav_msgs</depend>
<depend>quadrotor_msgs</depend>
<depend>rclcpp</depend>
```

- [ ] **Step 4: Build the controller package and rerun all its tests**

Run:

```bash
source /opt/ros/humble/setup.bash
cd /home/morphing01/Drone_SLAM/overlay_ws
colcon build --symlink-install --packages-select gvf_path_following_msgs gvf_ismc_path_following
colcon test --packages-select gvf_ismc_path_following --event-handlers console_direct+
cd /home/morphing01/Drone_SLAM/overlay_ws
colcon test-result --verbose --test-result-base build/gvf_ismc_path_following/test_results
```

Expected:

- `test_three_leaf_gvf`: PASS
- `test_ismc_outer_loop`: PASS
- `test_config_contract`: PASS
- `test_node_contract`: PASS

- [ ] **Step 5: Commit the controller nodes**

```bash
git add \
  overlay_ws/src/gvf_ismc_path_following/CMakeLists.txt \
  overlay_ws/src/gvf_ismc_path_following/package.xml \
  overlay_ws/src/gvf_ismc_path_following/config/gvf_ismc_path_following.yaml \
  overlay_ws/src/gvf_ismc_path_following/include/gvf_ismc_path_following/ismc_outer_loop.hpp \
  overlay_ws/src/gvf_ismc_path_following/src/ismc_outer_loop.cpp \
  overlay_ws/src/gvf_ismc_path_following/src/gvf_reference_node.cpp \
  overlay_ws/src/gvf_ismc_path_following/src/ismc_velocity_tracker_node.cpp \
  overlay_ws/src/gvf_ismc_path_following/test/test_ismc_outer_loop.cpp \
  overlay_ws/src/gvf_ismc_path_following/test/test_node_contract.py
git commit -m "feat: add gvf ismc controller nodes"
```

### Task 5: Add a dedicated real-hardware bringup package and launch script

**Files:**
- Create: `overlay_ws/src/gvf_ismc_real_bringup/CMakeLists.txt`
- Create: `overlay_ws/src/gvf_ismc_real_bringup/package.xml`
- Create: `overlay_ws/src/gvf_ismc_real_bringup/launch/single_real.launch.py`
- Create: `overlay_ws/src/gvf_ismc_real_bringup/test/test_single_real_launch_contract.py`
- Create: `overlay_ws/src/gvf_ismc_real_bringup/test/test_single_real_script_contract.py`
- Create: `scripts/run_gvf_ismc_single_real.sh`

- [ ] **Step 1: Write the failing launch and script contract tests**

```python
# overlay_ws/src/gvf_ismc_real_bringup/test/test_single_real_launch_contract.py
from importlib.util import module_from_spec
from importlib.util import spec_from_file_location
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "single_real.launch.py"
EXPECTED_PACKAGES = {
    ("fastlio2_to_px4_odometry", "fastlio2_to_px4_odometry_node"),
    ("fastlio2_to_ego_swarm_leveling", "fastlio2_to_ego_swarm_leveling_node"),
    ("gvf_ismc_path_following", "gvf_reference_node"),
    ("gvf_ismc_path_following", "ismc_velocity_tracker_node"),
    ("position_cmd_to_px4_bridge", "position_cmd_to_px4_bridge_node"),
}


def _load_launch_description() -> LaunchDescription:
    spec = spec_from_file_location("gvf_ismc_single_real_launch", LAUNCH_PATH)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def test_launch_exists() -> None:
    assert LAUNCH_PATH.exists(), f"Missing launch file: {LAUNCH_PATH}"


def test_launch_starts_expected_nodes() -> None:
    launch_description = _load_launch_description()
    nodes = [entity for entity in launch_description.entities if isinstance(entity, Node)]
    actual = {(node.node_package, node.node_executable) for node in nodes}
    assert actual == EXPECTED_PACKAGES
```

```python
# overlay_ws/src/gvf_ismc_real_bringup/test/test_single_real_script_contract.py
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[4]
SCRIPT_PATH = REPO_ROOT / "scripts" / "run_gvf_ismc_single_real.sh"


def test_script_exists() -> None:
    assert SCRIPT_PATH.exists(), f"Missing script: {SCRIPT_PATH}"


def test_script_uses_fastdds_and_new_launch() -> None:
    script = SCRIPT_PATH.read_text(encoding="utf-8")
    assert "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" in script
    assert "ros2 launch gvf_ismc_real_bringup single_real.launch.py" in script
    assert 'USE_ACCELERATION_FF="${USE_ACCELERATION_FF:-false}"' in script
    assert 'use_acceleration_feedforward:="$USE_ACCELERATION_FF"' in script
```

- [ ] **Step 2: Run the new package tests to verify they fail before the bringup package exists**

Run:

```bash
source /opt/ros/humble/setup.bash
cd /home/morphing01/Drone_SLAM/overlay_ws
colcon build --symlink-install --packages-select gvf_ismc_real_bringup
```

Expected:

- build failure because the package and launch file do not exist yet

- [ ] **Step 3: Implement the bringup package and script**

```cmake
# overlay_ws/src/gvf_ismc_real_bringup/CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(gvf_ismc_real_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_single_real_launch_contract
    test/test_single_real_launch_contract.py
  )
  ament_add_pytest_test(
    test_single_real_script_contract
    test/test_single_real_script_contract.py
  )
endif()

ament_package()
```

```xml
<!-- overlay_ws/src/gvf_ismc_real_bringup/package.xml -->
<?xml version="1.0"?>
<package format="3">
  <name>gvf_ismc_real_bringup</name>
  <version>0.0.1</version>
  <description>Real-hardware bringup wrapper for GVF + ISMC path-following experiments.</description>
  <maintainer email="dev@example.com">Drone SLAM Maintainers</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <test_depend>ament_cmake_pytest</test_depend>
  <test_depend>python3-yaml</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

```python
# overlay_ws/src/gvf_ismc_real_bringup/launch/single_real.launch.py
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_acceleration_feedforward = LaunchConfiguration("use_acceleration_feedforward")

    controller_config = Path(
        get_package_share_directory("gvf_ismc_path_following")
    ) / "config" / "gvf_ismc_path_following.yaml"
    leveling_config = Path(
        get_package_share_directory("fastlio2_to_ego_swarm_leveling")
    ) / "config" / "fastlio2_to_ego_swarm_leveling.yaml"
    px4_odom_config = Path(
        get_package_share_directory("fastlio2_to_px4_odometry")
    ) / "config" / "fastlio2_to_px4_odometry.yaml"
    bridge_config = Path(
        get_package_share_directory("position_cmd_to_px4_bridge")
    ) / "config" / "position_cmd_to_px4_bridge.yaml"

    return LaunchDescription([
        DeclareLaunchArgument("use_acceleration_feedforward", default_value="false"),
        Node(
            package="fastlio2_to_px4_odometry",
            executable="fastlio2_to_px4_odometry_node",
            name="fastlio2_to_px4_odometry",
            output="screen",
            parameters=[str(px4_odom_config)],
        ),
        Node(
            package="fastlio2_to_ego_swarm_leveling",
            executable="fastlio2_to_ego_swarm_leveling_node",
            name="fastlio2_to_ego_swarm_leveling",
            output="screen",
            parameters=[str(leveling_config)],
        ),
        Node(
            package="gvf_ismc_path_following",
            executable="gvf_reference_node",
            name="gvf_reference_node",
            output="screen",
            parameters=[str(controller_config)],
        ),
        Node(
            package="gvf_ismc_path_following",
            executable="ismc_velocity_tracker_node",
            name="ismc_velocity_tracker_node",
            output="screen",
            parameters=[str(controller_config)],
        ),
        Node(
            package="position_cmd_to_px4_bridge",
            executable="position_cmd_to_px4_bridge_node",
            name="position_cmd_to_px4_bridge",
            output="screen",
            parameters=[
                str(bridge_config),
                {
                    "fastlivo_odom_topic": "/aft_mapped_to_init_level",
                    "position_control_enabled": False,
                    "velocity_control_enabled": True,
                    "use_acceleration_feedforward": use_acceleration_feedforward,
                },
            ],
        ),
    ])
```

```bash
# scripts/run_gvf_ismc_single_real.sh
#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
OVERLAY_WS="$BASE_REPO_ROOT/overlay_ws"
RVIZ_CONFIG_FILE="$BASE_REPO_ROOT/overlay_ws/src/ego_swarm_real_bringup/rviz/ego_single_real.rviz"

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

source /opt/ros/humble/setup.bash
source "$OVERLAY_WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

USE_ACCELERATION_FF="${USE_ACCELERATION_FF:-false}"

cd "$OVERLAY_WS"
echo "Starting GVF + ISMC real-hardware bringup..."
ros2 launch gvf_ismc_real_bringup single_real.launch.py \
  use_acceleration_feedforward:="$USE_ACCELERATION_FF" &
LAUNCH_PID=$!

sleep 3

echo "Opening RViz with single-drone real-hardware config..."
# rviz2 -d "$RVIZ_CONFIG_FILE"
wait "$LAUNCH_PID"
```

- [ ] **Step 4: Build the bringup package and rerun its contract tests**

Run:

```bash
source /opt/ros/humble/setup.bash
cd /home/morphing01/Drone_SLAM/overlay_ws
colcon build --symlink-install --packages-select gvf_ismc_real_bringup
colcon test --packages-select gvf_ismc_real_bringup --event-handlers console_direct+
cd /home/morphing01/Drone_SLAM/overlay_ws
colcon test-result --verbose --test-result-base build/gvf_ismc_real_bringup/test_results
```

Expected:

- `test_single_real_launch_contract`: PASS
- `test_single_real_script_contract`: PASS

- [ ] **Step 5: Commit the bringup wrapper**

```bash
git add \
  overlay_ws/src/gvf_ismc_real_bringup/CMakeLists.txt \
  overlay_ws/src/gvf_ismc_real_bringup/package.xml \
  overlay_ws/src/gvf_ismc_real_bringup/launch/single_real.launch.py \
  overlay_ws/src/gvf_ismc_real_bringup/test/test_single_real_launch_contract.py \
  overlay_ws/src/gvf_ismc_real_bringup/test/test_single_real_script_contract.py \
  scripts/run_gvf_ismc_single_real.sh
git commit -m "feat: add gvf ismc real-hardware bringup"
```

### Task 6: Add the runbook and verify the integrated stack

**Files:**
- Create: `docs/runbooks/gvf-ismc-single-real.md`

- [ ] **Step 1: Write the operator runbook for bench checks, first takeover, and data capture**

````md
# GVF + ISMC 单机实机测试手册

## 1. 目的

验证以下链路：

1. `FAST-LIO2 -> /aft_mapped_to_init`
2. `/aft_mapped_to_init -> fastlio2_to_ego_swarm_leveling -> /aft_mapped_to_init_level`
3. `/aft_mapped_to_init_level -> gvf_reference_node -> /gvf/reference`
4. `/gvf/reference + /aft_mapped_to_init_level -> ismc_velocity_tracker_node -> /drone_0_planning/pos_cmd`
5. `/drone_0_planning/pos_cmd -> position_cmd_to_px4_bridge -> /fmu/in/*`

## 2. 启动顺序

### 终端 A

```bash
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 2000000
```

### 终端 B

```bash
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/slam_ws/install/setup.bash
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch fast_lio2_mid360_bringup mid360_fastlio2.launch.py \
  livox_config_file:=/home/morphing01/Drone_SLAM/slam_ws/src/livox_ros_driver2/config/MID360_config.json
```

### 终端 C

```bash
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch gvf_ismc_real_bringup single_real.launch.py \
  use_acceleration_feedforward:=false
```

## 3. 先看什么

- `ros2 topic hz /aft_mapped_to_init_level`
- `ros2 topic echo /gvf/reference --once`
- `ros2 topic hz /drone_0_planning/pos_cmd`
- bridge 日志是否进入 `ACTIVE`

## 4. 首次实飞顺序

1. 手动 `Position` 模式起飞到 `1.2m`
2. 先只验证小速度参考
3. 再验证大半径圆
4. 最后验证低速三叶曲线

## 5. 论文采数

- rosbag
- PX4 ULog
- 视频
- 记录 `|phi2|`、高度误差、速度误差、offboard 连续性
````

- [ ] **Step 2: Build the full package set that the new path depends on**

Run:

```bash
source /opt/ros/humble/setup.bash
cd /home/morphing01/Drone_SLAM/overlay_ws
colcon build --symlink-install --packages-up-to gvf_ismc_real_bringup
```

Expected:

- all four modified/new packages build:
  - `position_cmd_to_px4_bridge`
  - `gvf_path_following_msgs`
  - `gvf_ismc_path_following`
  - `gvf_ismc_real_bringup`

- [ ] **Step 3: Run the full targeted test suite for the new experiment path**

Run:

```bash
source /opt/ros/humble/setup.bash
cd /home/morphing01/Drone_SLAM/overlay_ws
colcon test --packages-select \
  position_cmd_to_px4_bridge \
  gvf_path_following_msgs \
  gvf_ismc_path_following \
  gvf_ismc_real_bringup \
  --event-handlers console_direct+
cd /home/morphing01/Drone_SLAM/overlay_ws
colcon test-result --verbose --test-result-base build/position_cmd_to_px4_bridge/test_results
colcon test-result --verbose --test-result-base build/gvf_path_following_msgs/test_results
colcon test-result --verbose --test-result-base build/gvf_ismc_path_following/test_results
colcon test-result --verbose --test-result-base build/gvf_ismc_real_bringup/test_results
```

Expected:

- all new and modified package-local tests pass

- [ ] **Step 4: Run one launch smoke test without flight**

Run:

```bash
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
timeout 10 ros2 launch gvf_ismc_real_bringup single_real.launch.py \
  use_acceleration_feedforward:=false
```

Expected:

- process exits because of `timeout`
- no immediate Python launch exception
- nodes begin startup without missing package or missing config errors

- [ ] **Step 5: Commit the runbook and final verification artifacts**

```bash
git add \
  docs/runbooks/gvf-ismc-single-real.md
git commit -m "docs: add gvf ismc real-flight runbook"
```
