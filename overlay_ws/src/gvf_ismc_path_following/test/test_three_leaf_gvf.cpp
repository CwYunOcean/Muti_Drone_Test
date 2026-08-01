#include <gtest/gtest.h>

#include <cmath>

#include <Eigen/Core>

#include "gvf_ismc_path_following/three_leaf_gvf.hpp"

namespace
{

gvf_ismc_path_following::ThreeLeafGvfParameters make_parameters()
{
  gvf_ismc_path_following::ThreeLeafGvfParameters params{};
  params.target_height_m = 1.2;
  params.base_radius_m = 2.0;
  params.lobe_amplitude_m = 0.5;
  params.lateral_gain = 1.0;
  params.vertical_gain = 1.0;
  params.max_speed_mps = 0.5;
  params.min_planar_speed_for_yaw_mps = 0.05;
  params.yaw_alpha = 0.1;
  return params;
}

Eigen::Vector2d planar_phi2_gradient(
  const Eigen::Vector3d & position,
  const gvf_ismc_path_following::ThreeLeafGvfParameters & params)
{
  const double x = position.x();
  const double y = position.y();
  const double r_sq = (x * x) + (y * y);
  const double r = std::sqrt(r_sq);
  const double theta = std::atan2(y, x);
  const double dphi_dr = 1.0;
  const double dphi_dtheta = 3.0 * params.lobe_amplitude_m * std::sin(3.0 * theta);

  const double dtheta_dx = -y / r_sq;
  const double dtheta_dy = x / r_sq;
  const double dr_dx = x / r;
  const double dr_dy = y / r;

  return Eigen::Vector2d(
    dphi_dr * dr_dx + dphi_dtheta * dtheta_dx,
    dphi_dr * dr_dy + dphi_dtheta * dtheta_dy);
}

Eigen::Vector3d on_curve_position(
  double theta,
  const gvf_ismc_path_following::ThreeLeafGvfParameters & params)
{
  const double radius =
    params.base_radius_m + params.lobe_amplitude_m * std::cos(3.0 * theta);
  return Eigen::Vector3d(
    radius * std::cos(theta),
    radius * std::sin(theta),
    params.target_height_m);
}

double wrap_to_pi(double angle)
{
  constexpr double two_pi = 2.0 * M_PI;
  while (angle > M_PI) {
    angle -= two_pi;
  }
  while (angle < -M_PI) {
    angle += two_pi;
  }
  return angle;
}

}  // namespace

TEST(ThreeLeafGvf, ReportsZeroImplicitErrorOnNominalCurvePoint)
{
  const auto params = make_parameters();
  const Eigen::Vector3d position(2.5, 0.0, 1.2);

  const auto result =
    gvf_ismc_path_following::evaluate_three_leaf_gvf(position, 0.3, params);

  EXPECT_NEAR(result.phi1, 0.0, 1e-9);
  EXPECT_NEAR(result.phi2, 0.0, 1e-9);
}

TEST(ThreeLeafGvf, CommandsNegativeVerticalVelocityAboveTargetHeight)
{
  const auto params = make_parameters();
  const Eigen::Vector3d position(2.0, 0.0, 1.5);

  const auto result =
    gvf_ismc_path_following::evaluate_three_leaf_gvf(position, -0.2, params);

  EXPECT_LT(result.desired_velocity.z(), 0.0);
}

TEST(ThreeLeafGvf, PreservesPreviousYawWhenPlanarSpeedBelowThreshold)
{
  auto params = make_parameters();
  params.min_planar_speed_for_yaw_mps = 10.0;
  const Eigen::Vector3d position(2.5, 0.0, 1.2);
  const double previous_yaw = 1.234;

  const auto result =
    gvf_ismc_path_following::evaluate_three_leaf_gvf(position, previous_yaw, params);

  EXPECT_NEAR(result.desired_yaw, previous_yaw, 1e-9);
  EXPECT_LT(result.desired_velocity.head<2>().norm(), params.min_planar_speed_for_yaw_mps);
}

TEST(ThreeLeafGvf, CommandsPlanarVelocityTangentToThreeLeafContourOnCurve)
{
  const auto params = make_parameters();
  const double theta = M_PI / 6.0;
  const Eigen::Vector3d position = on_curve_position(theta, params);

  const auto result =
    gvf_ismc_path_following::evaluate_three_leaf_gvf(position, 0.0, params);
  const Eigen::Vector2d gradient = planar_phi2_gradient(position, params);
  const double tangency_measure = gradient.dot(result.desired_velocity.head<2>());

  EXPECT_NEAR(result.phi2, 0.0, 1e-9);
  EXPECT_NEAR(tangency_measure, 0.0, 1e-9);
  EXPECT_GT(result.desired_velocity.head<2>().norm(), 0.0);
}

TEST(ThreeLeafGvf, DrivesInwardWhenOutsideContour)
{
  const auto params = make_parameters();
  const double theta = M_PI / 6.0;
  Eigen::Vector3d position = on_curve_position(theta, params);
  position.x() *= 1.2;
  position.y() *= 1.2;

  const auto result =
    gvf_ismc_path_following::evaluate_three_leaf_gvf(position, 0.0, params);
  const Eigen::Vector2d gradient = planar_phi2_gradient(position, params);

  EXPECT_GT(result.phi2, 0.0);
  EXPECT_LT(gradient.dot(result.desired_velocity.head<2>()), 0.0);
}

TEST(ThreeLeafGvf, DrivesOutwardWhenInsideContour)
{
  const auto params = make_parameters();
  const double theta = M_PI / 6.0;
  Eigen::Vector3d position = on_curve_position(theta, params);
  position.x() *= 0.8;
  position.y() *= 0.8;

  const auto result =
    gvf_ismc_path_following::evaluate_three_leaf_gvf(position, 0.0, params);
  const Eigen::Vector2d gradient = planar_phi2_gradient(position, params);

  EXPECT_LT(result.phi2, 0.0);
  EXPECT_GT(gradient.dot(result.desired_velocity.head<2>()), 0.0);
}

TEST(ThreeLeafGvf, SaturatesDesiredVelocityNormAtConfiguredMaximum)
{
  auto params = make_parameters();
  params.max_speed_mps = 0.25;
  const Eigen::Vector3d position(4.0, 0.0, 2.2);

  const auto result =
    gvf_ismc_path_following::evaluate_three_leaf_gvf(position, 0.0, params);

  EXPECT_NEAR(result.desired_velocity.norm(), params.max_speed_mps, 1e-9);
}

TEST(ThreeLeafGvf, CommandsPositiveVerticalVelocityBelowTargetHeight)
{
  const auto params = make_parameters();
  const Eigen::Vector3d position(2.0, 0.0, 0.9);

  const auto result =
    gvf_ismc_path_following::evaluate_three_leaf_gvf(position, 0.2, params);

  EXPECT_GT(result.desired_velocity.z(), 0.0);
}

TEST(ThreeLeafGvf, SmoothsYawTowardRawYawAboveThreshold)
{
  auto params = make_parameters();
  params.yaw_alpha = 0.25;
  const Eigen::Vector3d position(2.5, 0.0, params.target_height_m);
  const double previous_yaw = -2.5;

  const auto result =
    gvf_ismc_path_following::evaluate_three_leaf_gvf(position, previous_yaw, params);

  const double raw_yaw =
    std::atan2(result.desired_velocity.y(), result.desired_velocity.x());
  const double initial_error = wrap_to_pi(raw_yaw - previous_yaw);
  const double final_error = wrap_to_pi(raw_yaw - result.desired_yaw);

  EXPECT_GT(result.desired_velocity.head<2>().norm(), params.min_planar_speed_for_yaw_mps);
  EXPECT_GT(std::abs(initial_error), 1.0);
  EXPECT_NEAR(result.desired_yaw, wrap_to_pi(previous_yaw + params.yaw_alpha * initial_error), 1e-9);
  EXPECT_LT(std::abs(final_error), std::abs(initial_error));
  EXPECT_NE(result.desired_yaw, raw_yaw);
}

TEST(ThreeLeafGvf, WrapsYawAcrossPiBoundaryUsingShortestTurn)
{
  auto params = make_parameters();
  params.yaw_alpha = 0.5;
  const Eigen::Vector3d position = on_curve_position(1.9, params);
  const double previous_yaw = -M_PI + 0.05;

  const auto result =
    gvf_ismc_path_following::evaluate_three_leaf_gvf(position, previous_yaw, params);

  const double raw_yaw =
    std::atan2(result.desired_velocity.y(), result.desired_velocity.x());
  const double wrapped_error = wrap_to_pi(raw_yaw - previous_yaw);

  EXPECT_GT(raw_yaw, 0.0);
  EXPECT_LT(std::abs(wrapped_error), M_PI);
  EXPECT_LT(wrapped_error, 0.5);
  EXPECT_NEAR(result.desired_yaw, wrap_to_pi(previous_yaw + params.yaw_alpha * wrapped_error), 1e-9);
}

TEST(ThreeLeafGvf, ReturnsFiniteValuesNearOrigin)
{
  const auto params = make_parameters();
  const Eigen::Vector3d position(1e-9, -1e-9, params.target_height_m + 1e-9);

  const auto result =
    gvf_ismc_path_following::evaluate_three_leaf_gvf(position, 0.0, params);

  EXPECT_TRUE(std::isfinite(result.phi1));
  EXPECT_TRUE(std::isfinite(result.phi2));
  EXPECT_TRUE(result.desired_velocity.allFinite());
  EXPECT_TRUE(std::isfinite(result.desired_yaw));
}

// ===== Circular GVF tests =====

namespace
{

gvf_ismc_path_following::CircularGvfParameters make_circular_parameters()
{
  gvf_ismc_path_following::CircularGvfParameters params{};
  params.target_height_m = 1.2;
  params.radius_m = 2.0;
  params.lateral_gain = 1.0;
  params.vertical_gain = 1.0;
  params.max_speed_mps = 0.5;
  params.min_planar_speed_for_yaw_mps = 0.05;
  params.yaw_alpha = 0.1;
  return params;
}

}  // namespace

TEST(CircularGvf, ReportsZeroImplicitErrorOnCirclePoint)
{
  const auto params = make_circular_parameters();
  const Eigen::Vector3d position(2.0, 0.0, 1.2);

  const auto result =
    gvf_ismc_path_following::evaluate_circular_gvf(position, 0.0, params);

  EXPECT_NEAR(result.phi1, 0.0, 1e-9);
  EXPECT_NEAR(result.phi2, 0.0, 1e-9);
}

TEST(CircularGvf, CommandsNegativeVerticalVelocityAboveTargetHeight)
{
  const auto params = make_circular_parameters();
  const Eigen::Vector3d position(2.0, 0.0, 1.5);

  const auto result =
    gvf_ismc_path_following::evaluate_circular_gvf(position, -0.2, params);

  EXPECT_LT(result.desired_velocity.z(), 0.0);
}

TEST(CircularGvf, CommandsPositiveVerticalVelocityBelowTargetHeight)
{
  const auto params = make_circular_parameters();
  const Eigen::Vector3d position(2.0, 0.0, 0.9);

  const auto result =
    gvf_ismc_path_following::evaluate_circular_gvf(position, 0.2, params);

  EXPECT_GT(result.desired_velocity.z(), 0.0);
}

TEST(CircularGvf, CommandsCounterClockwiseTangentOnCircle)
{
  const auto params = make_circular_parameters();
  const Eigen::Vector3d position(2.0, 0.0, 1.2);

  const auto result =
    gvf_ismc_path_following::evaluate_circular_gvf(position, 0.0, params);

  EXPECT_NEAR(result.phi2, 0.0, 1e-9);
  // At (R, 0), counter-clockwise tangent points in +y direction
  EXPECT_GT(result.desired_velocity.y(), 0.0);
  EXPECT_NEAR(result.desired_velocity.x(), 0.0, 1e-6);
}

TEST(CircularGvf, DrivesInwardWhenOutsideCircle)
{
  const auto params = make_circular_parameters();
  const Eigen::Vector3d position(3.0, 0.0, 1.2);

  const auto result =
    gvf_ismc_path_following::evaluate_circular_gvf(position, 0.0, params);

  EXPECT_GT(result.phi2, 0.0);
  // Correction is radially outward; lateral_gain * phi2 > 0 subtracts it → drives inward
  EXPECT_LT(result.desired_velocity.x(), 0.0);
}

TEST(CircularGvf, DrivesOutwardWhenInsideCircle)
{
  const auto params = make_circular_parameters();
  const Eigen::Vector3d position(1.0, 0.0, 1.2);

  const auto result =
    gvf_ismc_path_following::evaluate_circular_gvf(position, 0.0, params);

  EXPECT_LT(result.phi2, 0.0);
  // phi2 < 0 so -lateral_gain * phi2 * correction is positive in x → drives outward
  EXPECT_GT(result.desired_velocity.x(), 0.0);
}

TEST(CircularGvf, SaturatesDesiredVelocityNormAtConfiguredMaximum)
{
  auto params = make_circular_parameters();
  params.max_speed_mps = 0.25;
  const Eigen::Vector3d position(4.0, 0.0, 2.2);

  const auto result =
    gvf_ismc_path_following::evaluate_circular_gvf(position, 0.0, params);

  EXPECT_NEAR(result.desired_velocity.norm(), params.max_speed_mps, 1e-9);
}

TEST(CircularGvf, ReturnsFiniteValuesNearOrigin)
{
  const auto params = make_circular_parameters();
  const Eigen::Vector3d position(1e-9, -1e-9, params.target_height_m + 1e-9);

  const auto result =
    gvf_ismc_path_following::evaluate_circular_gvf(position, 0.0, params);

  EXPECT_TRUE(std::isfinite(result.phi1));
  EXPECT_TRUE(std::isfinite(result.phi2));
  EXPECT_TRUE(result.desired_velocity.allFinite());
  EXPECT_TRUE(std::isfinite(result.desired_yaw));
}
