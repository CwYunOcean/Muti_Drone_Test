#include "gvf_ismc_path_following/three_leaf_gvf.hpp"

#include <algorithm>
#include <cmath>

namespace gvf_ismc_path_following
{

namespace
{

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

Eigen::Vector2d compute_phi2_gradient_xy(
  const Eigen::Vector3d & position,
  double lobe_amplitude_m)
{
  const double x = position.x();
  const double y = position.y();
  const double r_sq = (x * x) + (y * y);
  if (r_sq < 1e-12) {
    return Eigen::Vector2d::UnitX();
  }

  const double r = std::sqrt(r_sq);
  const double theta = std::atan2(y, x);
  const double dphi_dr = 1.0;
  const double dphi_dtheta = 3.0 * lobe_amplitude_m * std::sin(3.0 * theta);

  const double dr_dx = x / r;
  const double dr_dy = y / r;
  const double dtheta_dx = -y / r_sq;
  const double dtheta_dy = x / r_sq;

  return Eigen::Vector2d(
    dphi_dr * dr_dx + dphi_dtheta * dtheta_dx,
    dphi_dr * dr_dy + dphi_dtheta * dtheta_dy);
}

}  // namespace

ThreeLeafGvfResult evaluate_three_leaf_gvf(
  const Eigen::Vector3d & position,
  double previous_yaw,
  const ThreeLeafGvfParameters & parameters)
{
  const double x = position.x();
  const double y = position.y();
  const double z = position.z();

  const double r = std::hypot(x, y);
  const double theta = std::atan2(y, x);
  const double desired_radius =
    parameters.base_radius_m + parameters.lobe_amplitude_m * std::cos(3.0 * theta);

  const double phi1 = z - parameters.target_height_m;
  const double phi2 = r - desired_radius;

  const Eigen::Vector2d gradient_xy =
    compute_phi2_gradient_xy(position, parameters.lobe_amplitude_m);
  Eigen::Vector2d tangent_xy(-gradient_xy.y(), gradient_xy.x());
  if (tangent_xy.norm() < 1e-12) {
    tangent_xy = Eigen::Vector2d::UnitY();
  } else {
    tangent_xy.normalize();
  }

  Eigen::Vector2d correction_xy = gradient_xy;
  if (correction_xy.norm() < 1e-12) {
    correction_xy = Eigen::Vector2d::UnitX();
  } else {
    correction_xy.normalize();
  }

  Eigen::Vector3d desired_velocity =
    Eigen::Vector3d(tangent_xy.x(), tangent_xy.y(), 0.0) -
    parameters.lateral_gain * phi2 *
    Eigen::Vector3d(correction_xy.x(), correction_xy.y(), 0.0) -
    parameters.vertical_gain * phi1 * Eigen::Vector3d::UnitZ();

  const double velocity_norm = desired_velocity.norm();
  if (velocity_norm > parameters.max_speed_mps && velocity_norm > 1e-9) {
    desired_velocity *= parameters.max_speed_mps / velocity_norm;
  }

  const double planar_speed = desired_velocity.head<2>().norm();
  double desired_yaw = previous_yaw;
  if (planar_speed >= parameters.min_planar_speed_for_yaw_mps) {
    const double raw_yaw = std::atan2(desired_velocity.y(), desired_velocity.x());
    const double yaw_error = wrap_to_pi(raw_yaw - previous_yaw);
    const double alpha = std::clamp(parameters.yaw_alpha, 0.0, 1.0);
    desired_yaw = wrap_to_pi(previous_yaw + alpha * yaw_error);
  }

  ThreeLeafGvfResult result;
  result.phi1 = phi1;
  result.phi2 = phi2;
  result.desired_velocity = desired_velocity;
  result.desired_yaw = desired_yaw;
  return result;
}

CircularGvfResult evaluate_circular_gvf(
  const Eigen::Vector3d & position,
  double previous_yaw,
  const CircularGvfParameters & parameters)
{
  const double x = position.x();
  const double y = position.y();
  const double z = position.z();

  const double r = std::hypot(x, y);

  const double phi1 = z - parameters.target_height_m;
  const double phi2 = r - parameters.radius_m;

  Eigen::Vector2d gradient_xy = Eigen::Vector2d::UnitX();
  if (r > 1e-9) {
    gradient_xy = Eigen::Vector2d(x / r, y / r);
  }

  Eigen::Vector2d tangent_xy(-gradient_xy.y(), gradient_xy.x());

  Eigen::Vector3d desired_velocity =
    Eigen::Vector3d(tangent_xy.x(), tangent_xy.y(), 0.0) -
    parameters.lateral_gain * phi2 *
    Eigen::Vector3d(gradient_xy.x(), gradient_xy.y(), 0.0) -
    parameters.vertical_gain * phi1 * Eigen::Vector3d::UnitZ();

  const double velocity_norm = desired_velocity.norm();
  if (velocity_norm > parameters.max_speed_mps && velocity_norm > 1e-9) {
    desired_velocity *= parameters.max_speed_mps / velocity_norm;
  }

  const double planar_speed = desired_velocity.head<2>().norm();
  double desired_yaw = previous_yaw;
  if (planar_speed >= parameters.min_planar_speed_for_yaw_mps) {
    const double raw_yaw = std::atan2(desired_velocity.y(), desired_velocity.x());
    const double yaw_error = wrap_to_pi(raw_yaw - previous_yaw);
    const double alpha = std::clamp(parameters.yaw_alpha, 0.0, 1.0);
    desired_yaw = wrap_to_pi(previous_yaw + alpha * yaw_error);
  }

  CircularGvfResult result;
  result.phi1 = phi1;
  result.phi2 = phi2;
  result.desired_velocity = desired_velocity;
  result.desired_yaw = desired_yaw;
  return result;
}

}  // namespace gvf_ismc_path_following
