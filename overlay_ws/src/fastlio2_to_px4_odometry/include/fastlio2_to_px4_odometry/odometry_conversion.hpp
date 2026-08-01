#pragma once

#include <array>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "fastlio2_to_px4_odometry/frame_mapping.hpp"

namespace fastlio2_to_px4_odometry
{

struct ConvertedOdometry
{
  std::array<float, 3> position{};
  std::array<float, 3> velocity{};
  std::array<float, 4> q{};
};

ConvertedOdometry convert_ros_world_to_px4_ned(
  const Eigen::Vector3d & ros_position,
  const Eigen::Vector3d & ros_velocity,
  const Eigen::Quaterniond & ros_orientation,
  const FrameMapping & mapping,
  const Eigen::Vector3d & body_extrinsic_translation = Eigen::Vector3d::Zero(),
  const Eigen::Quaterniond & body_extrinsic_rotation = Eigen::Quaterniond::Identity());

}  // namespace fastlio2_to_px4_odometry
