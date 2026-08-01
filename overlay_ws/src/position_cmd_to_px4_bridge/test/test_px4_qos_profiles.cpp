#include <gtest/gtest.h>

#include <rmw/types.h>

#include "position_cmd_to_px4_bridge/px4_qos_profiles.hpp"

namespace position_cmd_to_px4_bridge
{

TEST(Px4QosProfiles, InputTopicsUseBestEffortVolatileKeepLastTen)
{
  const auto qos = make_px4_input_qos();
  const auto profile = qos.get_rmw_qos_profile();

  EXPECT_EQ(profile.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(profile.depth, 10u);
  EXPECT_EQ(profile.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  EXPECT_EQ(profile.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

}  // namespace position_cmd_to_px4_bridge
