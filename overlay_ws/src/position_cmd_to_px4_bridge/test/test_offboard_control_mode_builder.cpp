#include <gtest/gtest.h>

#include <cstdint>

#include "position_cmd_to_px4_bridge/offboard_control_mode_builder.hpp"

using position_cmd_to_px4_bridge::build_offboard_control_mode;
using position_cmd_to_px4_bridge::resolve_offboard_control_mode_selection;

TEST(OffboardControlModeBuilder, ResolverPreservesLegacyDefaultSelection)
{
  const auto selection = resolve_offboard_control_mode_selection(true, true);

  EXPECT_TRUE(selection.position);
  EXPECT_TRUE(selection.velocity);
}

TEST(OffboardControlModeBuilder, ResolverPreservesVelocityDominantSelection)
{
  const auto selection = resolve_offboard_control_mode_selection(false, true);

  EXPECT_FALSE(selection.position);
  EXPECT_TRUE(selection.velocity);
}

TEST(OffboardControlModeBuilder, ResolverPreservesPositionOnlySelection)
{
  const auto selection = resolve_offboard_control_mode_selection(true, false);

  EXPECT_TRUE(selection.position);
  EXPECT_FALSE(selection.velocity);
}

TEST(OffboardControlModeBuilder, ResolverFallsBackToVelocityForInvalidSelection)
{
  const auto selection = resolve_offboard_control_mode_selection(false, false);

  EXPECT_FALSE(selection.position);
  EXPECT_TRUE(selection.velocity);
}

TEST(OffboardControlModeBuilder, PreservesLegacyDefaultMode)
{
  const auto msg = build_offboard_control_mode(7ULL, true, true, false);

  EXPECT_TRUE(msg.position);
  EXPECT_TRUE(msg.velocity);
  EXPECT_FALSE(msg.acceleration);
  EXPECT_FALSE(msg.attitude);
  EXPECT_FALSE(msg.body_rate);
}

TEST(OffboardControlModeBuilder, PreservesPositionOnlyMode)
{
  const auto msg = build_offboard_control_mode(8ULL, true, false, false);

  EXPECT_TRUE(msg.position);
  EXPECT_FALSE(msg.velocity);
  EXPECT_FALSE(msg.acceleration);
  EXPECT_FALSE(msg.attitude);
  EXPECT_FALSE(msg.body_rate);
}

TEST(OffboardControlModeBuilder, BuildsVelocityDominantModeWhenOnlyVelocityIsEnabled)
{
  const uint64_t timestamp = 123456789ULL;
  const auto msg = build_offboard_control_mode(timestamp, false, true, false);

  EXPECT_EQ(msg.timestamp, timestamp);
  EXPECT_FALSE(msg.position);
  EXPECT_TRUE(msg.velocity);
  EXPECT_FALSE(msg.acceleration);
  EXPECT_FALSE(msg.attitude);
  EXPECT_FALSE(msg.body_rate);
}

TEST(OffboardControlModeBuilder, PreservesAccelerationFlagWhenFeedforwardEnabled)
{
  const auto msg = build_offboard_control_mode(42ULL, false, true, true);

  EXPECT_FALSE(msg.position);
  EXPECT_TRUE(msg.velocity);
  EXPECT_TRUE(msg.acceleration);
  EXPECT_FALSE(msg.attitude);
  EXPECT_FALSE(msg.body_rate);
}
