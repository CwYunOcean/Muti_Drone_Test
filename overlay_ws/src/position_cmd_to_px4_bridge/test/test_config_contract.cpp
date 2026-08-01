#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace
{

std::string read_file(const std::filesystem::path & path)
{
  std::ifstream stream(path);
  EXPECT_TRUE(stream.is_open()) << "Failed to open " << path;
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

TEST(ConfigContract, VehicleStatusTopicIsDisabledByDefault)
{
  const auto package_root =
    std::filesystem::path{__FILE__}.parent_path().parent_path();
  const auto config_path = package_root / "config" / "position_cmd_to_px4_bridge.yaml";
  const auto node_path = package_root / "src" / "position_cmd_to_px4_bridge_node.cpp";

  const std::string config = read_file(config_path);
  const std::string node_source = read_file(node_path);

  EXPECT_NE(
    config.find("vehicle_status_topic: \"\""),
    std::string::npos);
  EXPECT_NE(
    node_source.find("declare_parameter<std::string>(\"vehicle_status_topic\", \"\")"),
    std::string::npos);
}

TEST(ConfigContract, VehicleOdometryHeartbeatDefaultsToCurrentPx4Topic)
{
  const auto package_root =
    std::filesystem::path{__FILE__}.parent_path().parent_path();
  const auto config_path = package_root / "config" / "position_cmd_to_px4_bridge.yaml";
  const auto node_path = package_root / "src" / "position_cmd_to_px4_bridge_node.cpp";

  const std::string config = read_file(config_path);
  const std::string node_source = read_file(node_path);

  EXPECT_NE(
    config.find("vehicle_odometry_topic: \"/drone_1/fmu/out/vehicle_odometry\""),
    std::string::npos);
  EXPECT_NE(
    node_source.find("declare_parameter<std::string>(\n      \"vehicle_odometry_topic\", \"/drone_1/fmu/out/vehicle_odometry\")"),
    std::string::npos);
  EXPECT_NE(
    node_source.find("create_subscription<px4_msgs::msg::VehicleOdometry>"),
    std::string::npos);
  EXPECT_NE(
    node_source.find("vehicle_odometry_topic_"),
    std::string::npos);
}

TEST(ConfigContract, OffboardControlModeDefaultsAreDeclaredInConfigAndNode)
{
  const auto package_root =
    std::filesystem::path{__FILE__}.parent_path().parent_path();
  const auto config_path = package_root / "config" / "position_cmd_to_px4_bridge.yaml";
  const auto node_path = package_root / "src" / "position_cmd_to_px4_bridge_node.cpp";

  const std::string config = read_file(config_path);
  const std::string node_source = read_file(node_path);

  EXPECT_NE(
    config.find("position_control_enabled: true"),
    std::string::npos);
  EXPECT_NE(
    config.find("velocity_control_enabled: true"),
    std::string::npos);
  EXPECT_NE(
    node_source.find("declare_parameter<bool>(\"position_control_enabled\", true)"),
    std::string::npos);
  EXPECT_NE(
    node_source.find("declare_parameter<bool>(\"velocity_control_enabled\", true)"),
    std::string::npos);
  EXPECT_NE(
    config.find("auto_request_offboard_and_arm: false"),
    std::string::npos);
  EXPECT_NE(
    node_source.find("declare_parameter<bool>(\"auto_request_offboard_and_arm\", false)"),
    std::string::npos);
}

TEST(ConfigContract, Drone1DefaultsAreDeclaredInConfigAndNode)
{
  const auto package_root =
    std::filesystem::path{__FILE__}.parent_path().parent_path();
  const auto config_path = package_root / "config" / "position_cmd_to_px4_bridge.yaml";
  const auto node_path = package_root / "src" / "position_cmd_to_px4_bridge_node.cpp";

  const std::string config = read_file(config_path);
  const std::string node_source = read_file(node_path);

  EXPECT_NE(
    config.find("command_topic: \"/drone_1_planning/pos_cmd\""),
    std::string::npos);
  EXPECT_NE(
    config.find("offboard_mode_topic: \"/drone_1/fmu/in/offboard_control_mode\""),
    std::string::npos);
  EXPECT_NE(
    config.find("trajectory_setpoint_topic: \"/drone_1/fmu/in/trajectory_setpoint\""),
    std::string::npos);
  EXPECT_NE(
    config.find("vehicle_command_topic: \"/drone_1/fmu/in/vehicle_command\""),
    std::string::npos);
  EXPECT_NE(
    config.find("fastlivo_odom_topic: \"/drone_1/aft_mapped_to_init_level\""),
    std::string::npos);
  EXPECT_NE(
    config.find("target_system: 2"),
    std::string::npos);
  EXPECT_NE(
    node_source.find("declare_parameter<std::string>(\"command_topic\", \"/drone_1_planning/pos_cmd\")"),
    std::string::npos);
  EXPECT_NE(
    node_source.find("\"offboard_mode_topic\", \"/drone_1/fmu/in/offboard_control_mode\""),
    std::string::npos);
  EXPECT_NE(
    node_source.find("\"trajectory_setpoint_topic\", \"/drone_1/fmu/in/trajectory_setpoint\""),
    std::string::npos);
  EXPECT_NE(
    node_source.find("\"vehicle_command_topic\", \"/drone_1/fmu/in/vehicle_command\""),
    std::string::npos);
  EXPECT_NE(
    node_source.find("declare_parameter<int>(\"target_system\", 2)"),
    std::string::npos);
  EXPECT_NE(
    node_source.find("msg.target_system = static_cast<uint8_t>(target_system_)"),
    std::string::npos);
}

TEST(ConfigContract, InvalidOffboardControlConfigFallsBackToVelocityInNode)
{
  const auto package_root =
    std::filesystem::path{__FILE__}.parent_path().parent_path();
  const auto node_path = package_root / "src" / "position_cmd_to_px4_bridge_node.cpp";

  const std::string node_source = read_file(node_path);

  EXPECT_NE(
    node_source.find("!position_control_enabled_ && !velocity_control_enabled_"),
    std::string::npos);
  EXPECT_NE(
    node_source.find("falling back to velocity control for OffboardControlMode."),
    std::string::npos);
  EXPECT_NE(
    node_source.find(
      "resolve_offboard_control_mode_selection(\n      position_control_enabled_,\n      velocity_control_enabled_)"),
    std::string::npos);
  EXPECT_NE(
    node_source.find("selection.position"),
    std::string::npos);
  EXPECT_NE(
    node_source.find("selection.velocity"),
    std::string::npos);
}

}  // namespace
