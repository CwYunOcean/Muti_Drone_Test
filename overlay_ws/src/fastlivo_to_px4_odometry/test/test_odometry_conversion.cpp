#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include "fastlivo_to_px4_odometry/frame_mapping.hpp"
#include "fastlivo_to_px4_odometry/odometry_conversion.hpp"

using fastlivo_to_px4_odometry::ConvertedOdometry;
using fastlivo_to_px4_odometry::FrameMapping;
using fastlivo_to_px4_odometry::convert_ros_world_to_px4_ned;

TEST(OdometryConversion, ConvertsWorldPositionFromEnuToNedDefaultContract)
{
  const FrameMapping mapping = FrameMapping::default_enu_world_flu_body_to_ned_world_frd_body();
  const Eigen::Vector3d ros_position(1.0, 2.0, 3.0);
  const Eigen::Vector3d ros_velocity(0.5, -0.25, 0.75);
  const Eigen::Quaterniond ros_orientation = Eigen::Quaterniond::Identity();

  const ConvertedOdometry converted =
    convert_ros_world_to_px4_ned(ros_position, ros_velocity, ros_orientation, mapping);

  EXPECT_FLOAT_EQ(converted.position[0], 2.0f);
  EXPECT_FLOAT_EQ(converted.position[1], 1.0f);
  EXPECT_FLOAT_EQ(converted.position[2], -3.0f);

  EXPECT_FLOAT_EQ(converted.velocity[0], -0.25f);
  EXPECT_FLOAT_EQ(converted.velocity[1], 0.5f);
  EXPECT_FLOAT_EQ(converted.velocity[2], -0.75f);
}

TEST(OdometryConversion, ConvertsIdentityQuaternionFromFluBodyToFrdBodyContract)
{
  const FrameMapping mapping = FrameMapping::default_enu_world_flu_body_to_ned_world_frd_body();
  const ConvertedOdometry converted =
    convert_ros_world_to_px4_ned(
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      Eigen::Quaterniond::Identity(),
      mapping);

  const Eigen::Quaternionf q_px4(
    converted.q[0], converted.q[1], converted.q[2], converted.q[3]);
  const Eigen::Matrix3f rotation = q_px4.toRotationMatrix();

  EXPECT_NEAR(rotation(0, 1), -1.0f, 1e-5f);
  EXPECT_NEAR(rotation(1, 0), 1.0f, 1e-5f);
  EXPECT_NEAR(rotation(2, 2), 1.0f, 1e-5f);
}

TEST(OdometryConversion, NormalizesInputQuaternionInternally)
{
  const FrameMapping mapping = FrameMapping::default_enu_world_flu_body_to_ned_world_frd_body();
  const Eigen::Quaterniond normalized_ros_q(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitX()));
  Eigen::Quaterniond scaled_ros_q = normalized_ros_q;
  scaled_ros_q.coeffs() *= 2.0;

  const ConvertedOdometry converted_from_normalized =
    convert_ros_world_to_px4_ned(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), normalized_ros_q, mapping);
  const ConvertedOdometry converted_from_scaled =
    convert_ros_world_to_px4_ned(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), scaled_ros_q, mapping);

  const Eigen::Quaternionf q_normalized(
    converted_from_normalized.q[0],
    converted_from_normalized.q[1],
    converted_from_normalized.q[2],
    converted_from_normalized.q[3]);
  const Eigen::Quaternionf q_scaled(
    converted_from_scaled.q[0],
    converted_from_scaled.q[1],
    converted_from_scaled.q[2],
    converted_from_scaled.q[3]);

  const Eigen::Matrix3f r_normalized = q_normalized.toRotationMatrix();
  const Eigen::Matrix3f r_scaled = q_scaled.toRotationMatrix();

  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      EXPECT_NEAR(r_scaled(row, col), r_normalized(row, col), 1e-5f);
    }
  }
}

TEST(OdometryConversion, AppliesBodyExtrinsicPitchCompensationBeforePx4Conversion)
{
  const FrameMapping mapping = FrameMapping::default_enu_world_flu_body_to_ned_world_frd_body();
  const Eigen::Quaterniond ros_orientation = Eigen::Quaterniond::Identity();
  const Eigen::Vector3d ros_position = Eigen::Vector3d::Zero();
  const Eigen::Vector3d ros_velocity = Eigen::Vector3d::Zero();
  const Eigen::Vector3d body_extrinsic_translation = Eigen::Vector3d::Zero();
  const Eigen::Quaterniond body_extrinsic_rotation(
    Eigen::AngleAxisd(-0.519, Eigen::Vector3d::UnitY()));

  const ConvertedOdometry converted =
    convert_ros_world_to_px4_ned(
      ros_position,
      ros_velocity,
      ros_orientation,
      mapping,
      body_extrinsic_translation,
      body_extrinsic_rotation);

  const Eigen::Quaterniond px4_q(
    converted.q[0], converted.q[1], converted.q[2], converted.q[3]);
  const Eigen::Matrix3d px4_rotation = px4_q.toRotationMatrix();
  const Eigen::Matrix3d expected_rotation =
    mapping.world_transform() *
    body_extrinsic_rotation.toRotationMatrix() *
    mapping.body_transform().transpose();

  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      EXPECT_NEAR(px4_rotation(row, col), expected_rotation(row, col), 1e-6);
    }
  }
}
