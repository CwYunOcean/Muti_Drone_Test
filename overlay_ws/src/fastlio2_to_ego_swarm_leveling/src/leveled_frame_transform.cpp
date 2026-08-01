#include "fastlio2_to_ego_swarm_leveling/leveled_frame_transform.hpp"

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace fastlio2_to_ego_swarm_leveling {

namespace {

Eigen::Quaterniond quaternionFromRpy(const Eigen::Vector3d& rpy_rad) {
  const Eigen::AngleAxisd roll_angle(rpy_rad.x(), Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch_angle(rpy_rad.y(), Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw_angle(rpy_rad.z(), Eigen::Vector3d::UnitZ());
  return yaw_angle * pitch_angle * roll_angle;
}

}  // namespace

LeveledFrameTransform::LeveledFrameTransform(const Eigen::Vector3d& level_rpy_rad)
    : LeveledFrameTransform(level_rpy_rad, Eigen::Vector3d::Zero(), 0.0) {}

LeveledFrameTransform::LeveledFrameTransform(
    const Eigen::Vector3d& level_rpy_rad,
    const Eigen::Vector3d& world_origin_xyz,
    double world_origin_yaw_rad)
    : rotation_quaternion_(
          Eigen::AngleAxisd(world_origin_yaw_rad, Eigen::Vector3d::UnitZ()) *
          quaternionFromRpy(level_rpy_rad)),
      body_mount_correction_(quaternionFromRpy(level_rpy_rad).inverse()),
      world_origin_xyz_(world_origin_xyz) {
  rotation_matrix_ = rotation_quaternion_.toRotationMatrix();
}

nav_msgs::msg::Odometry LeveledFrameTransform::transform_odometry(
    const nav_msgs::msg::Odometry& input,
    const std::string& output_frame_id,
    const std::string& output_child_frame_id) const {
  nav_msgs::msg::Odometry output = input;
  output.header.frame_id = output_frame_id;
  output.child_frame_id = output_child_frame_id;

  const Eigen::Vector3d position(
      input.pose.pose.position.x,
      input.pose.pose.position.y,
      input.pose.pose.position.z);
  const Eigen::Vector3d linear_velocity(
      input.twist.twist.linear.x,
      input.twist.twist.linear.y,
      input.twist.twist.linear.z);
  const Eigen::Vector3d angular_velocity(
      input.twist.twist.angular.x,
      input.twist.twist.angular.y,
      input.twist.twist.angular.z);
  const Eigen::Quaterniond input_quaternion(
      input.pose.pose.orientation.w,
      input.pose.pose.orientation.x,
      input.pose.pose.orientation.y,
      input.pose.pose.orientation.z);

  const Eigen::Vector3d rotated_position =
      rotation_matrix_ * position + world_origin_xyz_;
  const Eigen::Vector3d rotated_linear_velocity =
      rotation_matrix_ * linear_velocity;
  const Eigen::Vector3d rotated_angular_velocity =
      rotation_matrix_ * angular_velocity;
  const Eigen::Quaterniond rotated_quaternion =
      rotation_quaternion_ * input_quaternion * body_mount_correction_;

  output.pose.pose.position.x = rotated_position.x();
  output.pose.pose.position.y = rotated_position.y();
  output.pose.pose.position.z = rotated_position.z();
  output.pose.pose.orientation.w = rotated_quaternion.w();
  output.pose.pose.orientation.x = rotated_quaternion.x();
  output.pose.pose.orientation.y = rotated_quaternion.y();
  output.pose.pose.orientation.z = rotated_quaternion.z();
  output.twist.twist.linear.x = rotated_linear_velocity.x();
  output.twist.twist.linear.y = rotated_linear_velocity.y();
  output.twist.twist.linear.z = rotated_linear_velocity.z();
  output.twist.twist.angular.x = rotated_angular_velocity.x();
  output.twist.twist.angular.y = rotated_angular_velocity.y();
  output.twist.twist.angular.z = rotated_angular_velocity.z();
  return output;
}

sensor_msgs::msg::PointCloud2 LeveledFrameTransform::transform_cloud(
    const sensor_msgs::msg::PointCloud2& input,
    const std::string& output_frame_id) const {
  pcl::PointCloud<pcl::PointXYZI> cloud_in;
  pcl::fromROSMsg(input, cloud_in);

  pcl::PointCloud<pcl::PointXYZI> cloud_out;
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = rotation_matrix_.cast<float>();
  transform.block<3, 1>(0, 3) = world_origin_xyz_.cast<float>();
  pcl::transformPointCloud(
      cloud_in,
      cloud_out,
      transform);

  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(cloud_out, output);
  output.header = input.header;
  output.header.frame_id = output_frame_id;
  return output;
}

}  // namespace fastlio2_to_ego_swarm_leveling
