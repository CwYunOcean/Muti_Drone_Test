#include <gtest/gtest.h>

#include <px4_msgs/msg/vehicle_status.hpp>

#include "position_cmd_to_px4_bridge/bridge_input_builder.hpp"

using position_cmd_to_px4_bridge::BridgeInputs;
using position_cmd_to_px4_bridge::VehicleStatusSnapshot;
using position_cmd_to_px4_bridge::build_bridge_inputs;

TEST(BridgeInputBuilder, UsesHeartbeatFreshnessToGateStreamingReadiness)
{
  const BridgeInputs inputs = build_bridge_inputs(
    true, true, true, true, VehicleStatusSnapshot{});

  EXPECT_TRUE(inputs.enabled);
  EXPECT_TRUE(inputs.fastlivo_fresh);
  EXPECT_TRUE(inputs.px4_status_fresh);
  EXPECT_TRUE(inputs.command_fresh);
}

TEST(BridgeInputBuilder, UsesVehicleStatusOnlyForModeAndArmFlags)
{
  VehicleStatusSnapshot status{};
  status.available = true;
  status.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  status.arming_state = px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;

  const BridgeInputs inputs = build_bridge_inputs(
    true, true, false, true, status);

  EXPECT_FALSE(inputs.px4_status_fresh);
  EXPECT_TRUE(inputs.px4_offboard);
  EXPECT_TRUE(inputs.px4_armed);
}
