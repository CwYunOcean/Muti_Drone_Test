# FAST-LIO2 EGO-Swarm Leveling Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a local adapter that rotates raw `FAST-LIO2` odometry and point clouds into a leveled planning frame, then let `ego_swarm_real_bringup` switch planner inputs between raw and leveled modes without changing the PX4 raw odometry path.

**Architecture:** Create a new `overlay_ws` package that owns the fixed-rotation transform logic and a single ROS 2 node. The node republishes leveled odometry and point clouds on planner-only topics. `single_real.launch.py` gains a planner mode switch that either keeps the current raw remaps or routes the planner through the new adapter while overriding `grid_map/frame_id`.

**Tech Stack:** ROS 2 Humble, `rclcpp`, `nav_msgs`, `sensor_msgs`, `Eigen3`, `PCL`, `ament_cmake`, `ament_cmake_gtest`, `ament_cmake_pytest`, Python launch testing.

---

### Task 1: Add the leveling package skeleton and config contract

**Files:**
- Create: `overlay_ws/src/fastlio2_to_ego_swarm_leveling/CMakeLists.txt`
- Create: `overlay_ws/src/fastlio2_to_ego_swarm_leveling/package.xml`
- Create: `overlay_ws/src/fastlio2_to_ego_swarm_leveling/config/fastlio2_to_ego_swarm_leveling.yaml`
- Create: `overlay_ws/src/fastlio2_to_ego_swarm_leveling/test/test_config_contract.py`

- [ ] **Step 1: Write the failing config contract test**

```python
from pathlib import Path

import yaml


def test_leveling_config_contract() -> None:
    config_path = (
        Path(__file__).resolve().parents[1] /
        "config" /
        "fastlio2_to_ego_swarm_leveling.yaml"
    )
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    params = config["/**"]["ros__parameters"]

    assert params["input_odom_topic"] == "/aft_mapped_to_init"
    assert params["input_cloud_topic"] == "/cloud_registered"
    assert params["output_odom_topic"] == "/aft_mapped_to_init_level"
    assert params["output_cloud_topic"] == "/cloud_registered_level"
    assert params["output_frame_id"] == "camera_init_level"
    assert params["output_child_frame_id"] == "body_level"
    assert params["level_rpy_rad"] == [0.0, 0.5235987755982988, 0.0]
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest -q overlay_ws/src/fastlio2_to_ego_swarm_leveling/test/test_config_contract.py`

Expected: FAIL because the package files do not exist yet.

- [ ] **Step 3: Add the package metadata and config**

```cmake
cmake_minimum_required(VERSION 3.8)
project(fastlio2_to_ego_swarm_leveling)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_transform
  src/leveled_frame_transform.cpp
)
target_include_directories(${PROJECT_NAME}_transform
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_transform
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
  src/fastlio2_to_ego_swarm_leveling_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_transform
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_transform ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  ament_add_gtest(
    test_leveled_frame_transform
    test/test_leveled_frame_transform.cpp
  )
  target_link_libraries(test_leveled_frame_transform
    ${PROJECT_NAME}_transform
    Eigen3::Eigen
    ${PCL_LIBRARIES}
  )

  ament_add_pytest_test(
    test_config_contract
    test/test_config_contract.py
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_transform)
ament_export_dependencies(Eigen3)

ament_package()
```

```xml
<?xml version="1.0"?>
<package format="3">
  <name>fastlio2_to_ego_swarm_leveling</name>
  <version>0.0.1</version>
  <description>Rotate FAST-LIO2 planner inputs into a leveled EGO-Swarm frame.</description>
  <maintainer email="dev@example.com">Drone SLAM Maintainers</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <build_depend>eigen3_cmake_module</build_depend>
  <build_depend>Eigen3</build_depend>

  <depend>nav_msgs</depend>
  <depend>pcl_conversions</depend>
  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>

  <test_depend>ament_cmake_gtest</test_depend>
  <test_depend>ament_cmake_pytest</test_depend>
  <test_depend>python3-yaml</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

```yaml
/**:
  ros__parameters:
    input_odom_topic: "/aft_mapped_to_init"
    input_cloud_topic: "/cloud_registered"
    output_odom_topic: "/aft_mapped_to_init_level"
    output_cloud_topic: "/cloud_registered_level"
    output_frame_id: "camera_init_level"
    output_child_frame_id: "body_level"
    level_rpy_rad: [0.0, 0.5235987755982988, 0.0]
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest -q overlay_ws/src/fastlio2_to_ego_swarm_leveling/test/test_config_contract.py`

Expected: PASS

### Task 2: Add failing transform tests for odometry and point clouds

**Files:**
- Create: `overlay_ws/src/fastlio2_to_ego_swarm_leveling/include/fastlio2_to_ego_swarm_leveling/leveled_frame_transform.hpp`
- Create: `overlay_ws/src/fastlio2_to_ego_swarm_leveling/test/test_leveled_frame_transform.cpp`

- [ ] **Step 1: Write the failing C++ unit tests**

```cpp
#include "fastlio2_to_ego_swarm_leveling/leveled_frame_transform.hpp"

#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>

TEST(LeveledFrameTransform, RotatesOdometryPoseAndTwist) {
  fastlio2_to_ego_swarm_leveling::LeveledFrameTransform transform(
      Eigen::Vector3d(0.0, M_PI_2, 0.0));

  nav_msgs::msg::Odometry input;
  input.header.frame_id = "camera_init";
  input.child_frame_id = "body";
  input.pose.pose.position.x = 1.0;
  input.twist.twist.linear.x = 1.0;
  input.twist.twist.angular.z = 1.0;
  input.pose.pose.orientation.w = 1.0;

  const auto output =
      transform.transform_odometry(input, "camera_init_level", "body_level");

  EXPECT_EQ(output.header.frame_id, "camera_init_level");
  EXPECT_EQ(output.child_frame_id, "body_level");
  EXPECT_NEAR(output.pose.pose.position.x, 0.0, 1e-9);
  EXPECT_NEAR(output.pose.pose.position.z, -1.0, 1e-9);
  EXPECT_NEAR(output.twist.twist.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(output.twist.twist.linear.z, -1.0, 1e-9);
}

TEST(LeveledFrameTransform, RotatesPointCloudCoordinatesAndFrameId) {
  fastlio2_to_ego_swarm_leveling::LeveledFrameTransform transform(
      Eigen::Vector3d(0.0, M_PI_2, 0.0));

  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(pcl::PointXYZI{1.0f, 0.0f, 0.0f, 5.0f});
  cloud.push_back(pcl::PointXYZI{0.0f, 0.0f, 1.0f, 7.0f});

  sensor_msgs::msg::PointCloud2 input;
  pcl::toROSMsg(cloud, input);
  input.header.frame_id = "camera_init";

  const auto output = transform.transform_cloud(input, "camera_init_level");

  pcl::PointCloud<pcl::PointXYZI> transformed;
  pcl::fromROSMsg(output, transformed);

  ASSERT_EQ(transformed.size(), 2u);
  EXPECT_EQ(output.header.frame_id, "camera_init_level");
  EXPECT_NEAR(transformed[0].x, 0.0, 1e-6);
  EXPECT_NEAR(transformed[0].z, -1.0, 1e-6);
  EXPECT_NEAR(transformed[0].intensity, 5.0, 1e-6);
  EXPECT_NEAR(transformed[1].x, 1.0, 1e-6);
  EXPECT_NEAR(transformed[1].z, 0.0, 1e-6);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `source /opt/ros/humble/setup.bash && cd overlay_ws && colcon test --packages-select fastlio2_to_ego_swarm_leveling --ctest-args -R test_leveled_frame_transform`

Expected: FAIL because the transform header and implementation do not exist.

- [ ] **Step 3: Add the transform interface**

```cpp
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace fastlio2_to_ego_swarm_leveling {

class LeveledFrameTransform {
 public:
  explicit LeveledFrameTransform(const Eigen::Vector3d& level_rpy_rad);

  nav_msgs::msg::Odometry transform_odometry(
      const nav_msgs::msg::Odometry& input,
      const std::string& output_frame_id,
      const std::string& output_child_frame_id) const;

  sensor_msgs::msg::PointCloud2 transform_cloud(
      const sensor_msgs::msg::PointCloud2& input,
      const std::string& output_frame_id) const;

 private:
  Eigen::Matrix3d rotation_matrix_;
  Eigen::Quaterniond rotation_quaternion_;
};

}  // namespace fastlio2_to_ego_swarm_leveling
```

- [ ] **Step 4: Run test to verify it still fails for missing implementation**

Run: `source /opt/ros/humble/setup.bash && cd overlay_ws && colcon test --packages-select fastlio2_to_ego_swarm_leveling --ctest-args -R test_leveled_frame_transform`

Expected: FAIL during build or link because implementation is still missing.

### Task 3: Implement the minimal fixed-rotation transform library

**Files:**
- Create: `overlay_ws/src/fastlio2_to_ego_swarm_leveling/src/leveled_frame_transform.cpp`
- Modify: `overlay_ws/src/fastlio2_to_ego_swarm_leveling/CMakeLists.txt`

- [ ] **Step 1: Write the minimal transform implementation**

```cpp
#include "fastlio2_to_ego_swarm_leveling/leveled_frame_transform.hpp"

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

namespace fastlio2_to_ego_swarm_leveling {

namespace {

Eigen::Quaterniond quaternion_from_rpy(const Eigen::Vector3d& rpy_rad) {
  const Eigen::AngleAxisd roll_angle(rpy_rad.x(), Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch_angle(rpy_rad.y(), Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw_angle(rpy_rad.z(), Eigen::Vector3d::UnitZ());
  return yaw_angle * pitch_angle * roll_angle;
}

}  // namespace

LeveledFrameTransform::LeveledFrameTransform(const Eigen::Vector3d& level_rpy_rad)
    : rotation_quaternion_(quaternion_from_rpy(level_rpy_rad)) {
  rotation_matrix_ = rotation_quaternion_.toRotationMatrix();
}

nav_msgs::msg::Odometry LeveledFrameTransform::transform_odometry(
    const nav_msgs::msg::Odometry& input,
    const std::string& output_frame_id,
    const std::string& output_child_frame_id) const {
  nav_msgs::msg::Odometry output = input;
  output.header.frame_id = output_frame_id;
  output.child_frame_id = output_child_frame_id;

  const Eigen::Vector3d position(
      input.pose.pose.position.x,
      input.pose.pose.position.y,
      input.pose.pose.position.z);
  const Eigen::Vector3d linear_velocity(
      input.twist.twist.linear.x,
      input.twist.twist.linear.y,
      input.twist.twist.linear.z);
  const Eigen::Vector3d angular_velocity(
      input.twist.twist.angular.x,
      input.twist.twist.angular.y,
      input.twist.twist.angular.z);
  const Eigen::Quaterniond input_quaternion(
      input.pose.pose.orientation.w,
      input.pose.pose.orientation.x,
      input.pose.pose.orientation.y,
      input.pose.pose.orientation.z);

  const Eigen::Vector3d rotated_position = rotation_matrix_ * position;
  const Eigen::Vector3d rotated_linear_velocity = rotation_matrix_ * linear_velocity;
  const Eigen::Vector3d rotated_angular_velocity = rotation_matrix_ * angular_velocity;
  const Eigen::Quaterniond rotated_quaternion = rotation_quaternion_ * input_quaternion;

  output.pose.pose.position.x = rotated_position.x();
  output.pose.pose.position.y = rotated_position.y();
  output.pose.pose.position.z = rotated_position.z();
  output.pose.pose.orientation.w = rotated_quaternion.w();
  output.pose.pose.orientation.x = rotated_quaternion.x();
  output.pose.pose.orientation.y = rotated_quaternion.y();
  output.pose.pose.orientation.z = rotated_quaternion.z();
  output.twist.twist.linear.x = rotated_linear_velocity.x();
  output.twist.twist.linear.y = rotated_linear_velocity.y();
  output.twist.twist.linear.z = rotated_linear_velocity.z();
  output.twist.twist.angular.x = rotated_angular_velocity.x();
  output.twist.twist.angular.y = rotated_angular_velocity.y();
  output.twist.twist.angular.z = rotated_angular_velocity.z();
  return output;
}

sensor_msgs::msg::PointCloud2 LeveledFrameTransform::transform_cloud(
    const sensor_msgs::msg::PointCloud2& input,
    const std::string& output_frame_id) const {
  pcl::PointCloud<pcl::PointXYZI> cloud_in;
  pcl::fromROSMsg(input, cloud_in);

  pcl::PointCloud<pcl::PointXYZI> cloud_out;
  pcl::transformPointCloud(cloud_in, cloud_out, rotation_matrix_.cast<float>());

  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(cloud_out, output);
  output.header = input.header;
  output.header.frame_id = output_frame_id;
  return output;
}

}  // namespace fastlio2_to_ego_swarm_leveling
```

- [ ] **Step 2: Run tests to verify they pass**

Run: `source /opt/ros/humble/setup.bash && cd overlay_ws && colcon test --packages-select fastlio2_to_ego_swarm_leveling --ctest-args -R test_leveled_frame_transform`

Expected: PASS

### Task 4: Add the ROS node and node-level wiring

**Files:**
- Create: `overlay_ws/src/fastlio2_to_ego_swarm_leveling/src/fastlio2_to_ego_swarm_leveling_node.cpp`

- [ ] **Step 1: Write the failing launch/build test indirectly via package build**

Run: `source /opt/ros/humble/setup.bash && cd overlay_ws && colcon build --symlink-install --packages-select fastlio2_to_ego_swarm_leveling`

Expected: FAIL because the node source does not exist but is referenced by CMake.

- [ ] **Step 2: Implement the minimal ROS node**

```cpp
#include "fastlio2_to_ego_swarm_leveling/leveled_frame_transform.hpp"

#include <rclcpp/rclcpp.hpp>

class Fastlio2ToEgoSwarmLevelingNode : public rclcpp::Node {
 public:
  Fastlio2ToEgoSwarmLevelingNode()
      : Node("fastlio2_to_ego_swarm_leveling"),
        transform_(declare_level_rpy()) {
    input_odom_topic_ = declare_parameter<std::string>("input_odom_topic", "/aft_mapped_to_init");
    input_cloud_topic_ = declare_parameter<std::string>("input_cloud_topic", "/cloud_registered");
    output_odom_topic_ = declare_parameter<std::string>("output_odom_topic", "/aft_mapped_to_init_level");
    output_cloud_topic_ = declare_parameter<std::string>("output_cloud_topic", "/cloud_registered_level");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "camera_init_level");
    output_child_frame_id_ = declare_parameter<std::string>("output_child_frame_id", "body_level");

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 20);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, 20);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        input_odom_topic_, 20,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          odom_pub_->publish(
              transform_.transform_odometry(*msg, output_frame_id_, output_child_frame_id_));
        });

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_cloud_topic_, 20,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          cloud_pub_->publish(transform_.transform_cloud(*msg, output_frame_id_));
        });
  }

 private:
  Eigen::Vector3d declare_level_rpy() {
    const auto rpy = declare_parameter<std::vector<double>>(
        "level_rpy_rad", std::vector<double>{0.0, 0.5235987755982988, 0.0});
    return Eigen::Vector3d(rpy.at(0), rpy.at(1), rpy.at(2));
  }

  fastlio2_to_ego_swarm_leveling::LeveledFrameTransform transform_;
  std::string input_odom_topic_;
  std::string input_cloud_topic_;
  std::string output_odom_topic_;
  std::string output_cloud_topic_;
  std::string output_frame_id_;
  std::string output_child_frame_id_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Fastlio2ToEgoSwarmLevelingNode>());
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 3: Run build to verify it passes**

Run: `source /opt/ros/humble/setup.bash && cd overlay_ws && colcon build --symlink-install --packages-select fastlio2_to_ego_swarm_leveling`

Expected: PASS

### Task 5: Extend bringup launch contracts for planner leveling mode

**Files:**
- Modify: `overlay_ws/src/ego_swarm_real_bringup/launch/single_real.launch.py`
- Modify: `overlay_ws/src/ego_swarm_real_bringup/package.xml`
- Modify: `overlay_ws/src/ego_swarm_real_bringup/test/test_single_real_launch_contract.py`

- [ ] **Step 1: Write the failing launch contract expectations**

```python
EXPECTED_PACKAGES = {
    ("ego_planner", "ego_planner_node"),
    ("ego_planner", "traj_server"),
    ("fastlivo_to_px4_odometry", "fastlivo_to_px4_odometry_node"),
    ("fastlio2_to_px4_odometry", "fastlio2_to_px4_odometry_node"),
    ("fastlio2_to_ego_swarm_leveling", "fastlio2_to_ego_swarm_leveling_node"),
    ("position_cmd_to_px4_bridge", "position_cmd_to_px4_bridge_node"),
}

def test_launch_declares_planner_mode_argument() -> None:
    launch_description = _load_launch_description()
    arguments = [
        entity for entity in launch_description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]
    planner_mode = next(
        argument for argument in arguments
        if argument.name == "planner_input_mode"
    )
    assert _to_text(planner_mode.default_value) == "raw"
```

- [ ] **Step 2: Run the failing launch contract test**

Run: `python3 -m pytest -q overlay_ws/src/ego_swarm_real_bringup/test/test_single_real_launch_contract.py`

Expected: FAIL because the new package and argument are not wired yet.

- [ ] **Step 3: Update launch and package wiring**

```python
planner_input_mode = LaunchConfiguration("planner_input_mode")
leveled_odom_topic = LaunchConfiguration("leveled_odom_topic")
leveled_cloud_topic = LaunchConfiguration("leveled_cloud_topic")
leveled_frame_id = LaunchConfiguration("leveled_frame_id")
planner_odom_topic = PythonExpression([
    "'", leveled_odom_topic, "' if '", planner_input_mode,
    "' == 'fastlio2_leveled' else '", odom_topic, "'"
])
planner_cloud_topic = PythonExpression([
    "'", leveled_cloud_topic, "' if '", planner_input_mode,
    "' == 'fastlio2_leveled' else '", cloud_topic, "'"
])
planner_frame_id = PythonExpression([
    "'", leveled_frame_id, "' if '", planner_input_mode,
    "' == 'fastlio2_leveled' else 'camera_init'"
])
```

```python
DeclareLaunchArgument("planner_input_mode", default_value="raw"),
DeclareLaunchArgument("leveled_odom_topic", default_value="/aft_mapped_to_init_level"),
DeclareLaunchArgument("leveled_cloud_topic", default_value="/cloud_registered_level"),
DeclareLaunchArgument("leveled_frame_id", default_value="camera_init_level"),
```

```python
Node(
    package="fastlio2_to_ego_swarm_leveling",
    executable="fastlio2_to_ego_swarm_leveling_node",
    name="fastlio2_to_ego_swarm_leveling",
    output="screen",
    parameters=[str(leveling_config)],
    condition=IfCondition(
        PythonExpression(["'", planner_input_mode, "' == 'fastlio2_leveled'"])
    ),
),
```

```python
Node(
    package="ego_planner",
    executable="ego_planner_node",
    name="drone_0_ego_planner_node",
    output="screen",
    parameters=[str(planner_config), {"grid_map/frame_id": planner_frame_id}],
    remappings=[
        ("odom_world", planner_odom_topic),
        ("grid_map/odom", planner_odom_topic),
        ("grid_map/cloud", planner_cloud_topic),
    ],
),
```

```xml
<exec_depend>fastlio2_to_ego_swarm_leveling</exec_depend>
```

- [ ] **Step 4: Run launch contract test to verify it passes**

Run: `python3 -m pytest -q overlay_ws/src/ego_swarm_real_bringup/test/test_single_real_launch_contract.py`

Expected: PASS

### Task 6: Add script support for planner mode switching

**Files:**
- Modify: `scripts/run_ego_single_real.sh`
- Modify: `overlay_ws/src/ego_swarm_real_bringup/test/test_single_real_script_contract.py`

- [ ] **Step 1: Write the failing script contract**

```python
def test_single_real_script_supports_planner_input_mode() -> None:
    script = _script_path().read_text(encoding="utf-8")
    assert 'PLANNER_INPUT_MODE="${PLANNER_INPUT_MODE:-raw}"' in script
    assert 'planner_input_mode:="$PLANNER_INPUT_MODE"' in script
```

- [ ] **Step 2: Run the failing script contract**

Run: `python3 -m pytest -q overlay_ws/src/ego_swarm_real_bringup/test/test_single_real_script_contract.py`

Expected: FAIL because the script does not expose the planner mode yet.

- [ ] **Step 3: Update the wrapper script**

```bash
PLANNER_INPUT_MODE="${PLANNER_INPUT_MODE:-raw}"

ros2 launch ego_swarm_real_bringup single_real.launch.py \
  odom_bridge_type:="$ODOM_BRIDGE_TYPE" \
  planner_input_mode:="$PLANNER_INPUT_MODE" &
```

- [ ] **Step 4: Run the script contract test**

Run: `python3 -m pytest -q overlay_ws/src/ego_swarm_real_bringup/test/test_single_real_script_contract.py`

Expected: PASS

### Task 7: Verify targeted build and test coverage

**Files:**
- Modify: `overlay_ws/src/ego_swarm_real_bringup/test/test_single_real_launch_contract.py`
- Modify: `overlay_ws/src/ego_swarm_real_bringup/test/test_single_real_script_contract.py`
- Modify: `overlay_ws/src/fastlio2_to_ego_swarm_leveling/test/test_config_contract.py`
- Modify: `overlay_ws/src/fastlio2_to_ego_swarm_leveling/test/test_leveled_frame_transform.cpp`

- [ ] **Step 1: Run targeted package builds**

Run: `source /opt/ros/humble/setup.bash && source slam_ws/install/setup.bash && cd overlay_ws && colcon build --symlink-install --packages-select fastlio2_to_ego_swarm_leveling ego_swarm_real_bringup`

Expected: PASS

- [ ] **Step 2: Run targeted tests**

Run: `source /opt/ros/humble/setup.bash && source slam_ws/install/setup.bash && source overlay_ws/install/setup.bash && cd overlay_ws && colcon test --packages-select fastlio2_to_ego_swarm_leveling ego_swarm_real_bringup`

Expected: PASS

- [ ] **Step 3: Inspect detailed test results**

Run: `cd overlay_ws && colcon test-result --verbose --test-result-base build/fastlio2_to_ego_swarm_leveling/test_results && colcon test-result --verbose --test-result-base build/ego_swarm_real_bringup/test_results`

Expected: `0 errors, 0 failures`

- [ ] **Step 4: Run script syntax validation**

Run: `bash -n scripts/run_ego_single_real.sh`

Expected: PASS

### Task 8: Real-hardware operator handoff

**Files:**
- No code changes expected

- [ ] **Step 1: Provide the exact runtime commands**

```bash
./scripts/run_mid360_fastlio2.sh
PLANNER_INPUT_MODE=fastlio2_leveled ODOM_BRIDGE_TYPE=fastlio2 ./scripts/run_ego_single_real.sh
```

- [ ] **Step 2: Provide the exact runtime checks**

```bash
ros2 topic hz /aft_mapped_to_init
ros2 topic hz /aft_mapped_to_init_level
ros2 topic hz /cloud_registered
ros2 topic hz /cloud_registered_level
ros2 topic echo /aft_mapped_to_init_level --once
```

- [ ] **Step 3: Document the sign-tuning fallback**

If the leveled cloud tilts in the opposite direction, flip the sign of `level_rpy_rad[1]` in `overlay_ws/src/fastlio2_to_ego_swarm_leveling/config/fastlio2_to_ego_swarm_leveling.yaml` and re-run the same checks.
