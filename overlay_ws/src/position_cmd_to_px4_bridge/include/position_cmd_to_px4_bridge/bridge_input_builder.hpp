#pragma once

#include <cstdint>

#include "position_cmd_to_px4_bridge/bridge_state_machine.hpp"

namespace position_cmd_to_px4_bridge
{

struct VehicleStatusSnapshot
{
  bool available{false};
  uint8_t nav_state{0};
  uint8_t arming_state{0};
};

BridgeInputs build_bridge_inputs(
  bool enabled,
  bool fastlivo_fresh,
  bool px4_heartbeat_fresh,
  bool command_fresh,
  const VehicleStatusSnapshot & vehicle_status);

}  // namespace position_cmd_to_px4_bridge
