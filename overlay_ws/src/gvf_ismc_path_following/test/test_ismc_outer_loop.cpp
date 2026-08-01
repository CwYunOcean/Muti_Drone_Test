#include <gtest/gtest.h>

#include <Eigen/Core>

#include "gvf_ismc_path_following/ismc_outer_loop.hpp"

namespace
{

gvf_ismc_path_following::IsmcOuterLoopParameters make_parameters()
{
  gvf_ismc_path_following::IsmcOuterLoopParameters params{};
  params.lambda = 2.0;
  params.k = 1.5;
  params.gamma = 0.0;
  params.c1 = 0.05;
  params.epsilon = 0.01;
  params.max_acceleration_mps2 = 3.0;
  params.adaptation_gain = 0.5;
  return params;
}

}  // namespace

TEST(IsmcOuterLoop, ProducesZeroAccelerationForZeroVelocityError)
{
  const auto params = make_parameters();
  gvf_ismc_path_following::IsmcOuterLoopState state{};

  const auto output = gvf_ismc_path_following::step_ismc_outer_loop(
    Eigen::Vector3d(0.5, -0.2, 0.1),
    Eigen::Vector3d(0.5, -0.2, 0.1),
    0.02,
    params,
    &state);

  EXPECT_TRUE(output.velocity_error.isZero(1e-9));
  EXPECT_TRUE(output.sliding_surface.isZero(1e-9));
  EXPECT_TRUE(output.acceleration_command.isZero(1e-9));
}

TEST(IsmcOuterLoop, CommandsPositiveXAccelerationForPositiveXVelocityError)
{
  const auto params = make_parameters();
  gvf_ismc_path_following::IsmcOuterLoopState state{};

  const auto output = gvf_ismc_path_following::step_ismc_outer_loop(
    Eigen::Vector3d(1.0, 0.0, 0.0),
    Eigen::Vector3d::Zero(),
    0.02,
    params,
    &state);

  EXPECT_GT(output.velocity_error.x(), 0.0);
  EXPECT_GT(output.sliding_surface.x(), 0.0);
  EXPECT_GT(output.acceleration_command.x(), 0.0);
}

TEST(IsmcOuterLoop, DoesNotProduceDerivativeKickOnFirstInitializedUpdate)
{
  auto params = make_parameters();
  params.lambda = 0.0;
  params.k = 1.0;
  params.gamma = 0.0;
  params.c1 = 0.0;
  params.epsilon = 0.0;
  params.max_acceleration_mps2 = 100.0;

  gvf_ismc_path_following::IsmcOuterLoopState state{};

  const auto output = gvf_ismc_path_following::step_ismc_outer_loop(
    Eigen::Vector3d(1.0, -0.5, 0.25),
    Eigen::Vector3d::Zero(),
    0.01,
    params,
    &state);

  EXPECT_TRUE(output.error_rate.isZero(1e-9));
  EXPECT_TRUE(output.sliding_surface.isZero(1e-9));
  EXPECT_TRUE(output.acceleration_command.isZero(1e-9));
  EXPECT_TRUE(state.previous_velocity_error.isApprox(output.velocity_error, 1e-9));
  EXPECT_TRUE(state.initialized);
}

TEST(IsmcOuterLoop, RespectsAccelerationClamp)
{
  auto params = make_parameters();
  params.max_acceleration_mps2 = 0.75;
  gvf_ismc_path_following::IsmcOuterLoopState state{};

  const auto output = gvf_ismc_path_following::step_ismc_outer_loop(
    Eigen::Vector3d(10.0, -10.0, 5.0),
    Eigen::Vector3d::Zero(),
    0.01,
    params,
    &state);

  EXPECT_LE(output.acceleration_command.cwiseAbs().maxCoeff(), params.max_acceleration_mps2 + 1e-9);
}

TEST(IsmcOuterLoop, TanhReachingLawIsSmoothAtZero)
{
  // Unlike signum which jumps discontinuously, tanh produces continuous output
  auto params = make_parameters();
  params.lambda = 1.0;
  params.epsilon = 0.1;
  params.k = 0.0;
  params.gamma = 0.0;
  params.c1 = 0.0;
  params.enable_adaptation = false;
  params.max_acceleration_mps2 = 100.0;

  gvf_ismc_path_following::IsmcOuterLoopState state{};

  // First step to initialize
  gvf_ismc_path_following::step_ismc_outer_loop(
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    0.02, params, &state);

  // Small velocity error → small smooth output (not ±1)
  const auto output = gvf_ismc_path_following::step_ismc_outer_loop(
    Eigen::Vector3d(0.01, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    0.02, params, &state);

  // tanh of a small sliding surface should be small, not ±1 like signum
  EXPECT_GT(output.acceleration_command.x(), 0.0);
  EXPECT_LT(output.acceleration_command.x(), 1.0);
}

TEST(IsmcOuterLoop, C1DampingAddsErrorRateContribution)
{
  auto params_with_c1 = make_parameters();
  params_with_c1.c1 = 0.5;
  params_with_c1.k = 0.0;
  params_with_c1.gamma = 0.0;
  params_with_c1.lambda = 0.0;
  params_with_c1.epsilon = 0.0;
  params_with_c1.enable_adaptation = false;
  params_with_c1.max_acceleration_mps2 = 100.0;

  auto params_without_c1 = params_with_c1;
  params_without_c1.c1 = 0.0;

  gvf_ismc_path_following::IsmcOuterLoopState state_with{};
  gvf_ismc_path_following::IsmcOuterLoopState state_without{};

  // Initialize both states
  gvf_ismc_path_following::step_ismc_outer_loop(
    Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d::Zero(),
    0.02, params_with_c1, &state_with);
  gvf_ismc_path_following::step_ismc_outer_loop(
    Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d::Zero(),
    0.02, params_without_c1, &state_without);

  // Now introduce a velocity error — error_rate will be nonzero
  const auto out_with = gvf_ismc_path_following::step_ismc_outer_loop(
    Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d::Zero(),
    0.02, params_with_c1, &state_with);
  const auto out_without = gvf_ismc_path_following::step_ismc_outer_loop(
    Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d::Zero(),
    0.02, params_without_c1, &state_without);

  // With c1 > 0, acceleration should include the error_rate contribution
  EXPECT_GT(out_with.acceleration_command.x(), out_without.acceleration_command.x());
}
