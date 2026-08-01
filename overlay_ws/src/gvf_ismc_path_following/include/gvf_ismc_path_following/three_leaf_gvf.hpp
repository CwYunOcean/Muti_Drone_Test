#ifndef GVF_ISMC_PATH_FOLLOWING__THREE_LEAF_GVF_HPP_
#define GVF_ISMC_PATH_FOLLOWING__THREE_LEAF_GVF_HPP_

#include <Eigen/Core>

namespace gvf_ismc_path_following
{

struct ThreeLeafGvfParameters
{
  double target_height_m{1.2};
  double base_radius_m{2.0};
  double lobe_amplitude_m{0.5};
  double lateral_gain{1.0};
  double vertical_gain{1.0};
  double max_speed_mps{0.5};
  double min_planar_speed_for_yaw_mps{0.05};
  double yaw_alpha{0.1};
};

struct CircularGvfParameters
{
  double target_height_m{1.2};
  double radius_m{2.0};
  double lateral_gain{1.0};
  double vertical_gain{1.0};
  double max_speed_mps{0.5};
  double min_planar_speed_for_yaw_mps{0.05};
  double yaw_alpha{0.1};
};

struct GvfResult
{
  double phi1{0.0};
  double phi2{0.0};
  Eigen::Vector3d desired_velocity{Eigen::Vector3d::Zero()};
  double desired_yaw{0.0};
};

using ThreeLeafGvfResult = GvfResult;
using CircularGvfResult = GvfResult;

ThreeLeafGvfResult evaluate_three_leaf_gvf(
  const Eigen::Vector3d & position,
  double previous_yaw,
  const ThreeLeafGvfParameters & parameters);

CircularGvfResult evaluate_circular_gvf(
  const Eigen::Vector3d & position,
  double previous_yaw,
  const CircularGvfParameters & parameters);

}  // namespace gvf_ismc_path_following

#endif  // GVF_ISMC_PATH_FOLLOWING__THREE_LEAF_GVF_HPP_
