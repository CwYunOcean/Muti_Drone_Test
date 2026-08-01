#pragma once

#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace fastlio2_to_ego_swarm_leveling {

class LeveledFrameTransform {
 public:
  explicit LeveledFrameTransform(const Eigen::Vector3d& level_rpy_rad);

  LeveledFrameTransform(const Eigen::Vector3d& level_rpy_rad,
                        const Eigen::Vector3d& world_origin_xyz,
                        double world_origin_yaw_rad);

  nav_msgs::msg::Odometry transform_odometry(
      const nav_msgs::msg::Odometry& input,
      const std::string& output_frame_id,
      const std::string& output_child_frame_id) const;

  sensor_msgs::msg::PointCloud2 transform_cloud(
      const sensor_msgs::msg::PointCloud2& input,
      const std::string& output_frame_id) const;

 private:
  Eigen::Matrix3d rotation_matrix_;
  Eigen::Quaterniond rotation_quaternion_;
  Eigen::Quaterniond body_mount_correction_;
  Eigen::Vector3d world_origin_xyz_;
};

}  // namespace fastlio2_to_ego_swarm_leveling
