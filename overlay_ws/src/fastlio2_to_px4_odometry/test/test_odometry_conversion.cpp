#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include "fastlio2_to_px4_odometry/frame_mapping.hpp"
#include "fastlio2_to_px4_odometry/odometry_conversion.hpp"

using fastlio2_to_px4_odometry::ConvertedOdometry;
using fastlio2_to_px4_odometry::FrameMapping;
using fastlio2_to_px4_odometry::convert_ros_world_to_px4_ned;

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

TEST(OdometryConversion, CancelsLeveledWorldPitchWithSensorToBodyExtrinsic)
{
  constexpr double kMountPitchRad = 0.5235987755982988;
  const FrameMapping mapping = FrameMapping::default_enu_world_flu_body_to_ned_world_frd_body();
  const Eigen::Quaterniond leveled_orientation(
    Eigen::AngleAxisd(kMountPitchRad, Eigen::Vector3d::UnitY()));
  const Eigen::Quaterniond sensor_to_body_rotation(
    Eigen::AngleAxisd(-kMountPitchRad, Eigen::Vector3d::UnitY()));

  const ConvertedOdometry converted = convert_ros_world_to_px4_ned(
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(),
    leveled_orientation,
    mapping,
    Eigen::Vector3d::Zero(),
    sensor_to_body_rotation);

  const Eigen::Quaterniond px4_orientation(
    converted.q[0], converted.q[1], converted.q[2], converted.q[3]);
  const Eigen::Matrix3d actual_rotation = px4_orientation.toRotationMatrix();
  Eigen::Matrix3d expected_rotation;
  expected_rotation <<
    0.0, -1.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 0.0, 1.0;

  EXPECT_TRUE(actual_rotation.isApprox(expected_rotation, 1e-6));
}
