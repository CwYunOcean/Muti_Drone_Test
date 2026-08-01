#include <gtest/gtest.h>
#include <quadrotor_msgs/msg/position_command.hpp>

#include "position_cmd_to_px4_bridge/frame_mapping.hpp"
#include "position_cmd_to_px4_bridge/trajectory_setpoint_conversion.hpp"

using position_cmd_to_px4_bridge::FrameMapping;
using position_cmd_to_px4_bridge::ConvertedTrajectorySetpoint;
using position_cmd_to_px4_bridge::convert_position_command;

TEST(TrajectorySetpointConversion, ConvertsRosWorldPositionVelocityAndAcceleration)
{
  quadrotor_msgs::msg::PositionCommand command{};
  command.position.x = 1.0;
  command.position.y = 2.0;
  command.position.z = 3.0;
  command.velocity.x = 0.2;
  command.velocity.y = 0.4;
  command.velocity.z = 0.6;
  command.acceleration.x = 0.1;
  command.acceleration.y = 0.2;
  command.acceleration.z = 0.3;
  command.yaw = 0.0;
  command.yaw_dot = 0.5;

  const ConvertedTrajectorySetpoint converted =
    convert_position_command(command, FrameMapping::default_enu_world_to_ned_world());

  EXPECT_FLOAT_EQ(converted.position[0], 2.0f);
  EXPECT_FLOAT_EQ(converted.position[1], 1.0f);
  EXPECT_FLOAT_EQ(converted.position[2], -3.0f);
  EXPECT_FLOAT_EQ(converted.velocity[0], 0.4f);
  EXPECT_FLOAT_EQ(converted.velocity[1], 0.2f);
  EXPECT_FLOAT_EQ(converted.velocity[2], -0.6f);
  EXPECT_FLOAT_EQ(converted.acceleration[0], 0.2f);
  EXPECT_FLOAT_EQ(converted.acceleration[1], 0.1f);
  EXPECT_FLOAT_EQ(converted.acceleration[2], -0.3f);
}

TEST(TrajectorySetpointConversion, ConvertsYawThroughHeadingVectorTransform)
{
  quadrotor_msgs::msg::PositionCommand command{};
  command.yaw = 0.0;
  command.yaw_dot = 0.0;

  const ConvertedTrajectorySetpoint converted =
    convert_position_command(command, FrameMapping::default_enu_world_to_ned_world());

  EXPECT_NEAR(converted.yaw, static_cast<float>(M_PI_2), 1e-5f);
}
