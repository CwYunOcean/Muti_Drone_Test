#include "position_cmd_to_px4_bridge/bridge_input_builder.hpp"

#include <px4_msgs/msg/vehicle_status.hpp>

namespace position_cmd_to_px4_bridge
{

BridgeInputs build_bridge_inputs(
  bool enabled,
  bool fastlivo_fresh,
  bool px4_heartbeat_fresh,
  bool command_fresh,
  const VehicleStatusSnapshot & vehicle_status)
{
  BridgeInputs inputs{};
  inputs.enabled = enabled;
  inputs.fastlivo_fresh = fastlivo_fresh;
  inputs.px4_status_fresh = px4_heartbeat_fresh;
  inputs.command_fresh = command_fresh;
  inputs.px4_offboard = vehicle_status.available &&
    vehicle_status.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  inputs.px4_armed = vehicle_status.available &&
    vehicle_status.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
  return inputs;
}

}  // namespace position_cmd_to_px4_bridge
