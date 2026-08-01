#include "fastlivo_to_px4_odometry/odometry_conversion.hpp"

namespace fastlivo_to_px4_odometry
{

ConvertedOdometry convert_ros_world_to_px4_ned(
  const Eigen::Vector3d & ros_position,
  const Eigen::Vector3d & ros_velocity,
  const Eigen::Quaterniond & ros_orientation,
  const FrameMapping & mapping,
  const Eigen::Vector3d & body_extrinsic_translation,
  const Eigen::Quaterniond & body_extrinsic_rotation)
{
  Eigen::Quaterniond normalized_ros_orientation = ros_orientation;
  normalized_ros_orientation.normalize();
  Eigen::Quaterniond normalized_body_extrinsic_rotation = body_extrinsic_rotation;
  normalized_body_extrinsic_rotation.normalize();

  const Eigen::Matrix3d world_transform = mapping.world_transform();
  const Eigen::Matrix3d body_transform = mapping.body_transform();
  const Eigen::Matrix3d ros_rotation = normalized_ros_orientation.toRotationMatrix();
  const Eigen::Matrix3d ros_body_rotation =
    ros_rotation * normalized_body_extrinsic_rotation.toRotationMatrix();
  const Eigen::Vector3d ros_body_position = ros_position + ros_rotation * body_extrinsic_translation;

  const Eigen::Vector3d px4_position = world_transform * ros_body_position;
  const Eigen::Vector3d px4_velocity = world_transform * ros_velocity;
  const Eigen::Matrix3d px4_rotation =
    world_transform * ros_body_rotation * body_transform.transpose();

  Eigen::Quaterniond px4_q(px4_rotation);
  px4_q.normalize();

  ConvertedOdometry converted{};
  converted.position = {
    static_cast<float>(px4_position.x()),
    static_cast<float>(px4_position.y()),
    static_cast<float>(px4_position.z())};
  converted.velocity = {
    static_cast<float>(px4_velocity.x()),
    static_cast<float>(px4_velocity.y()),
    static_cast<float>(px4_velocity.z())};
  converted.q = {
    static_cast<float>(px4_q.w()),
    static_cast<float>(px4_q.x()),
    static_cast<float>(px4_q.y()),
    static_cast<float>(px4_q.z())};

  return converted;
}

}  // namespace fastlivo_to_px4_odometry
