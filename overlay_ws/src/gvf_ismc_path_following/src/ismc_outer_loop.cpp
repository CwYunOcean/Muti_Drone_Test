#include "gvf_ismc_path_following/ismc_outer_loop.hpp"

#include <algorithm>
#include <cmath>

namespace gvf_ismc_path_following
{

namespace
{

double sanitized_dt(double dt_seconds)
{
  if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0) {
    return 1e-3;
  }
  return dt_seconds;
}

}  // namespace

IsmcOuterLoopOutput step_ismc_outer_loop(
  const Eigen::Vector3d & desired_velocity,
  const Eigen::Vector3d & current_velocity,
  double dt_seconds,
  const IsmcOuterLoopParameters & parameters,
  IsmcOuterLoopState * state)
{
  IsmcOuterLoopOutput output{};
  output.velocity_error = desired_velocity - current_velocity;

  const double dt = sanitized_dt(dt_seconds);
  Eigen::Vector3d previous_error = Eigen::Vector3d::Zero();
  if (state != nullptr) {
    if (!state->initialized) {
      state->previous_velocity_error = output.velocity_error;
      state->initialized = true;
    }
    previous_error = state->previous_velocity_error;
  }

  output.error_rate = (output.velocity_error - previous_error) / dt;
  output.sliding_surface = output.error_rate + parameters.lambda * output.velocity_error;

  Eigen::Vector3d adaptive_bias = Eigen::Vector3d::Zero();
  if (parameters.enable_adaptation && state != nullptr) {
    state->adaptive_bias +=
      parameters.adaptation_gain * output.sliding_surface * dt;
    adaptive_bias = state->adaptive_bias;
  } else if (state != nullptr) {
    state->adaptive_bias.setZero();
  }
  output.adaptive_bias = adaptive_bias;

  // Smooth reaching law using tanh (matches MATLAB: (lambda + epsilon) .* tanh(s1))
  const Eigen::Vector3d reaching_law =
    (parameters.lambda + parameters.epsilon) * output.sliding_surface.array().tanh().matrix();

  // Full ISMC acceleration: c1 * e_dot + gamma * e + k * s + reaching_law + P_hat
  Eigen::Vector3d acceleration_command =
    parameters.c1 * output.error_rate +
    parameters.gamma * output.velocity_error +
    parameters.k * output.sliding_surface +
    reaching_law +
    adaptive_bias;

  const double max_acceleration =
    std::max(0.0, parameters.max_acceleration_mps2);
  acceleration_command =
    acceleration_command.cwiseMax(Eigen::Vector3d::Constant(-max_acceleration))
    .cwiseMin(Eigen::Vector3d::Constant(max_acceleration));
  output.acceleration_command = acceleration_command;

  if (state != nullptr) {
    state->previous_velocity_error = output.velocity_error;
  }

  return output;
}

}  // namespace gvf_ismc_path_following
