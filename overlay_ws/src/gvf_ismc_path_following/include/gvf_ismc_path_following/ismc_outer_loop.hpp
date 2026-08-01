#ifndef GVF_ISMC_PATH_FOLLOWING__ISMC_OUTER_LOOP_HPP_
#define GVF_ISMC_PATH_FOLLOWING__ISMC_OUTER_LOOP_HPP_

#include <Eigen/Core>

namespace gvf_ismc_path_following
{

struct IsmcOuterLoopParameters
{
  double lambda{2.0};
  double k{1.0};
  double gamma{0.0};
  double c1{0.05};
  double epsilon{0.01};
  double adaptation_gain{0.0};
  double max_acceleration_mps2{3.0};
  bool enable_adaptation{false};
};

struct IsmcOuterLoopState
{
  Eigen::Vector3d previous_velocity_error{Eigen::Vector3d::Zero()};
  Eigen::Vector3d adaptive_bias{Eigen::Vector3d::Zero()};
  bool initialized{false};
};

struct IsmcOuterLoopOutput
{
  Eigen::Vector3d velocity_error{Eigen::Vector3d::Zero()};
  Eigen::Vector3d error_rate{Eigen::Vector3d::Zero()};
  Eigen::Vector3d sliding_surface{Eigen::Vector3d::Zero()};
  Eigen::Vector3d adaptive_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration_command{Eigen::Vector3d::Zero()};
};

IsmcOuterLoopOutput step_ismc_outer_loop(
  const Eigen::Vector3d & desired_velocity,
  const Eigen::Vector3d & current_velocity,
  double dt_seconds,
  const IsmcOuterLoopParameters & parameters,
  IsmcOuterLoopState * state);

}  // namespace gvf_ismc_path_following

#endif  // GVF_ISMC_PATH_FOLLOWING__ISMC_OUTER_LOOP_HPP_
