#pragma once

#include <array>

#include <quadrotor_msgs/msg/position_command.hpp>

#include "position_cmd_to_px4_bridge/frame_mapping.hpp"

namespace position_cmd_to_px4_bridge
{

struct ConvertedTrajectorySetpoint
{
  std::array<float, 3> position{};
  std::array<float, 3> velocity{};
  std::array<float, 3> acceleration{};
  float yaw{0.0f};
  float yaw_rate{0.0f};
};

ConvertedTrajectorySetpoint convert_position_command(
  const quadrotor_msgs::msg::PositionCommand & command,
  const FrameMapping & mapping);

}  // namespace position_cmd_to_px4_bridge
