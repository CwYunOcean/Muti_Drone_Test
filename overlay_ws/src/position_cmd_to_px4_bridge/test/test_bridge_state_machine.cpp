#include <gtest/gtest.h>

#include "position_cmd_to_px4_bridge/bridge_state_machine.hpp"

using position_cmd_to_px4_bridge::BridgeInputs;
using position_cmd_to_px4_bridge::BridgeState;
using position_cmd_to_px4_bridge::BridgeStateMachine;

TEST(BridgeStateMachine, ProgressesToStreamingWhenInputsBecomeReady)
{
  BridgeStateMachine machine(10);
  BridgeInputs inputs{};

  inputs.enabled = true;
  EXPECT_EQ(machine.tick(inputs).state, BridgeState::WAIT_FASTLIVO);

  inputs.fastlivo_fresh = true;
  EXPECT_EQ(machine.tick(inputs).state, BridgeState::WAIT_PX4);

  inputs.px4_status_fresh = true;
  inputs.command_fresh = true;
  EXPECT_EQ(machine.tick(inputs).state, BridgeState::STREAM_SETPOINT);
}

TEST(BridgeStateMachine, RequestsOffboardAndArmAfterWarmupCyclesWithoutStatusFeedback)
{
  BridgeStateMachine machine(2);
  BridgeInputs inputs{};
  inputs.enabled = true;
  inputs.fastlivo_fresh = true;
  inputs.px4_status_fresh = true;
  inputs.command_fresh = true;

  machine.tick(inputs);
  machine.tick(inputs);
  auto output = machine.tick(inputs);
  EXPECT_EQ(output.state, BridgeState::STREAM_SETPOINT);
  output = machine.tick(inputs);
  EXPECT_EQ(output.state, BridgeState::STREAM_SETPOINT);
  output = machine.tick(inputs);
  EXPECT_EQ(output.state, BridgeState::ACTIVE);
  EXPECT_TRUE(output.request_offboard);
  EXPECT_TRUE(output.request_arm);
  EXPECT_TRUE(output.stream_setpoint);
}

TEST(BridgeStateMachine, DoesNotRequestOffboardOrArmWhenManualSwitchingIsConfigured)
{
  BridgeStateMachine machine(2, false);
  BridgeInputs inputs{};
  inputs.enabled = true;
  inputs.fastlivo_fresh = true;
  inputs.px4_status_fresh = true;
  inputs.command_fresh = true;

  machine.tick(inputs);
  machine.tick(inputs);
  auto output = machine.tick(inputs);
  EXPECT_EQ(output.state, BridgeState::STREAM_SETPOINT);
  output = machine.tick(inputs);
  EXPECT_EQ(output.state, BridgeState::STREAM_SETPOINT);
  output = machine.tick(inputs);
  EXPECT_EQ(output.state, BridgeState::STREAM_SETPOINT);
  EXPECT_FALSE(output.request_offboard);
  EXPECT_FALSE(output.request_arm);
  EXPECT_TRUE(output.stream_setpoint);
}

TEST(BridgeStateMachine, FallsBackToFailsafeOnFastlivoLoss)
{
  BridgeStateMachine machine(1);
  BridgeInputs inputs{};
  inputs.enabled = true;
  inputs.fastlivo_fresh = true;
  inputs.px4_status_fresh = true;
  inputs.command_fresh = true;

  machine.tick(inputs);
  machine.tick(inputs);
  machine.tick(inputs);
  machine.tick(inputs);
  auto output = machine.tick(inputs);
  EXPECT_EQ(output.state, BridgeState::ACTIVE);

  inputs.fastlivo_fresh = false;
  output = machine.tick(inputs);
  EXPECT_EQ(output.state, BridgeState::FAILSAFE);
  EXPECT_TRUE(output.request_land);
}
