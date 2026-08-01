#include "position_cmd_to_px4_bridge/trajectory_setpoint_conversion.hpp"

#include <cmath>

#include <Eigen/Core>

namespace position_cmd_to_px4_bridge
{

ConvertedTrajectorySetpoint convert_position_command(
  const quadrotor_msgs::msg::PositionCommand & command,
  const FrameMapping & mapping)
{
  const Eigen::Matrix3d transform = mapping.world_transform();

  const Eigen::Vector3d ros_position(
    command.position.x, command.position.y, command.position.z);
  const Eigen::Vector3d ros_velocity(
    command.velocity.x, command.velocity.y, command.velocity.z);
  const Eigen::Vector3d ros_acceleration(
    command.acceleration.x, command.acceleration.y, command.acceleration.z);

  const Eigen::Vector3d px4_position = transform * ros_position;
  const Eigen::Vector3d px4_velocity = transform * ros_velocity;
  const Eigen::Vector3d px4_acceleration = transform * ros_acceleration;

  const Eigen::Vector3d ros_heading(std::cos(command.yaw), std::sin(command.yaw), 0.0);
  const Eigen::Vector3d px4_heading = transform * ros_heading;

  ConvertedTrajectorySetpoint converted{};
  converted.position = {
    static_cast<float>(px4_position.x()),
    static_cast<float>(px4_position.y()),
    static_cast<float>(px4_position.z())};
  converted.velocity = {
    static_cast<float>(px4_velocity.x()),
    static_cast<float>(px4_velocity.y()),
    static_cast<float>(px4_velocity.z())};
  converted.acceleration = {
    static_cast<float>(px4_acceleration.x()),
    static_cast<float>(px4_acceleration.y()),
    static_cast<float>(px4_acceleration.z())};
  converted.yaw = static_cast<float>(std::atan2(px4_heading.y(), px4_heading.x()));
  converted.yaw_rate = static_cast<float>(-command.yaw_dot);
  return converted;
}

}  // namespace position_cmd_to_px4_bridge
