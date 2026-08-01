#ifndef POSITION_CMD_TO_PX4_BRIDGE__OFFBOARD_CONTROL_MODE_BUILDER_HPP_
#define POSITION_CMD_TO_PX4_BRIDGE__OFFBOARD_CONTROL_MODE_BUILDER_HPP_

#include <cstdint>

#include <px4_msgs/msg/offboard_control_mode.hpp>

namespace position_cmd_to_px4_bridge
{

struct OffboardControlModeSelection
{
  bool position;
  bool velocity;
};

OffboardControlModeSelection resolve_offboard_control_mode_selection(
  bool position_enabled,
  bool velocity_enabled);

px4_msgs::msg::OffboardControlMode build_offboard_control_mode(
  uint64_t timestamp,
  bool position_enabled,
  bool velocity_enabled,
  bool acceleration_enabled);

}  // namespace position_cmd_to_px4_bridge

#endif  // POSITION_CMD_TO_PX4_BRIDGE__OFFBOARD_CONTROL_MODE_BUILDER_HPP_
