#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>

#include "distribute_control/core/world_transform.hpp"

namespace {

TEST(WorldTransform, ConvertsPx4NedPositionToWorldEnu) {
  const distribute_control::WorldTransform transform(
      Eigen::Vector3d(10.0, 20.0, 1.5), M_PI / 2.0);

  const Eigen::Vector3d world =
      transform.positionFromPx4Ned(Eigen::Vector3d(2.0, 3.0, -4.0));

  EXPECT_NEAR(world.x(), 8.0, 1e-9);
  EXPECT_NEAR(world.y(), 23.0, 1e-9);
  EXPECT_NEAR(world.z(), 5.5, 1e-9);
}

TEST(WorldTransform, RotatesVelocityWithoutApplyingOrigin) {
  const distribute_control::WorldTransform transform(
      Eigen::Vector3d(10.0, 20.0, 1.5), M_PI / 2.0);

  const Eigen::Vector3d world =
      transform.velocityFromPx4Ned(Eigen::Vector3d(2.0, 3.0, -4.0));

  EXPECT_NEAR(world.x(), -2.0, 1e-9);
  EXPECT_NEAR(world.y(), 3.0, 1e-9);
  EXPECT_NEAR(world.z(), 4.0, 1e-9);
}

}  // namespace
