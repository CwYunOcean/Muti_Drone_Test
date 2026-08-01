#include "position_cmd_to_px4_bridge/offboard_control_mode_builder.hpp"

namespace position_cmd_to_px4_bridge
{

OffboardControlModeSelection resolve_offboard_control_mode_selection(
  const bool position_enabled,
  const bool velocity_enabled)
{
  if (!position_enabled && !velocity_enabled) {
    return OffboardControlModeSelection{false, true};
  }

  return OffboardControlModeSelection{position_enabled, velocity_enabled};
}

px4_msgs::msg::OffboardControlMode build_offboard_control_mode(
  const uint64_t timestamp,
  const bool position_enabled,
  const bool velocity_enabled,
  const bool acceleration_enabled)
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
