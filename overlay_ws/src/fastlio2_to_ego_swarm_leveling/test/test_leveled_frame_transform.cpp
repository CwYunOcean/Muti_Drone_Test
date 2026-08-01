#include "fastlio2_to_ego_swarm_leveling/leveled_frame_transform.hpp"

#include <gtest/gtest.h>

#include <cmath>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

TEST(LeveledFrameTransform, RotatesOdometryPoseAndTwist) {
  fastlio2_to_ego_swarm_leveling::LeveledFrameTransform transform(
      Eigen::Vector3d(0.0, M_PI_2, 0.0));

  nav_msgs::msg::Odometry input;
  input.header.frame_id = "camera_init";
  input.child_frame_id = "body";
  input.pose.pose.position.x = 1.0;
  input.pose.pose.orientation.w = 1.0;
  input.twist.twist.linear.x = 1.0;
  input.twist.twist.angular.z = 1.0;

  const auto output =
      transform.transform_odometry(input, "camera_init_level", "body_level");

  EXPECT_EQ(output.header.frame_id, "camera_init_level");
  EXPECT_EQ(output.child_frame_id, "body_level");
  EXPECT_NEAR(output.pose.pose.position.x, 0.0, 1e-9);
  EXPECT_NEAR(output.pose.pose.position.y, 0.0, 1e-9);
  EXPECT_NEAR(output.pose.pose.position.z, -1.0, 1e-9);
  EXPECT_NEAR(output.twist.twist.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(output.twist.twist.linear.y, 0.0, 1e-9);
  EXPECT_NEAR(output.twist.twist.linear.z, -1.0, 1e-9);
  EXPECT_NEAR(output.twist.twist.angular.x, 1.0, 1e-9);
  EXPECT_NEAR(output.twist.twist.angular.y, 0.0, 1e-9);
  EXPECT_NEAR(output.twist.twist.angular.z, 0.0, 1e-9);
}

TEST(LeveledFrameTransform, RotatesPointCloudCoordinatesAndFrameId) {
  fastlio2_to_ego_swarm_leveling::LeveledFrameTransform transform(
      Eigen::Vector3d(0.0, M_PI_2, 0.0));

  pcl::PointCloud<pcl::PointXYZI> input_cloud;
  input_cloud.push_back(pcl::PointXYZI{1.0f, 0.0f, 0.0f, 5.0f});
  input_cloud.push_back(pcl::PointXYZI{0.0f, 0.0f, 1.0f, 7.0f});

  sensor_msgs::msg::PointCloud2 input;
  pcl::toROSMsg(input_cloud, input);
  input.header.frame_id = "camera_init";

  const auto output = transform.transform_cloud(input, "camera_init_level");

  pcl::PointCloud<pcl::PointXYZI> output_cloud;
  pcl::fromROSMsg(output, output_cloud);

  ASSERT_EQ(output_cloud.size(), 2u);
  EXPECT_EQ(output.header.frame_id, "camera_init_level");
  EXPECT_NEAR(output_cloud[0].x, 0.0, 1e-6);
  EXPECT_NEAR(output_cloud[0].y, 0.0, 1e-6);
  EXPECT_NEAR(output_cloud[0].z, -1.0, 1e-6);
  EXPECT_NEAR(output_cloud[0].intensity, 5.0, 1e-6);
  EXPECT_NEAR(output_cloud[1].x, 1.0, 1e-6);
  EXPECT_NEAR(output_cloud[1].y, 0.0, 1e-6);
  EXPECT_NEAR(output_cloud[1].z, 0.0, 1e-6);
  EXPECT_NEAR(output_cloud[1].intensity, 7.0, 1e-6);
}

TEST(LeveledFrameTransform, AppliesWorldOriginYawAndTranslationToOdometry) {
  fastlio2_to_ego_swarm_leveling::LeveledFrameTransform transform(
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 2.0, 3.0),
      M_PI_2);

  nav_msgs::msg::Odometry input;
  input.header.frame_id = "camera_init";
  input.child_frame_id = "body";
  input.pose.pose.position.x = 1.0;
  input.pose.pose.orientation.w = 1.0;
  input.twist.twist.linear.x = 1.0;

  const auto output =
      transform.transform_odometry(input, "camera_init_level", "body_level");

  EXPECT_NEAR(output.pose.pose.position.x, 1.0, 1e-9);
  EXPECT_NEAR(output.pose.pose.position.y, 3.0, 1e-9);
  EXPECT_NEAR(output.pose.pose.position.z, 3.0, 1e-9);
  EXPECT_NEAR(output.twist.twist.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(output.twist.twist.linear.y, 1.0, 1e-9);
  const Eigen::Quaterniond expected_orientation(
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
  EXPECT_NEAR(output.pose.pose.orientation.w, expected_orientation.w(), 1e-9);
  EXPECT_NEAR(output.pose.pose.orientation.z, expected_orientation.z(), 1e-9);
}

TEST(LeveledFrameTransform, AppliesWorldOriginToPointCloud) {
  fastlio2_to_ego_swarm_leveling::LeveledFrameTransform transform(
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 2.0, 3.0),
      M_PI_2);

  pcl::PointCloud<pcl::PointXYZI> input_cloud;
  input_cloud.push_back(pcl::PointXYZI{1.0f, 0.0f, 0.0f, 5.0f});

  sensor_msgs::msg::PointCloud2 input;
  pcl::toROSMsg(input_cloud, input);
  input.header.frame_id = "camera_init";

  const auto output = transform.transform_cloud(input, "camera_init_level");

  pcl::PointCloud<pcl::PointXYZI> output_cloud;
  pcl::fromROSMsg(output, output_cloud);

  ASSERT_EQ(output_cloud.size(), 1u);
  EXPECT_NEAR(output_cloud[0].x, 1.0, 1e-6);
  EXPECT_NEAR(output_cloud[0].y, 3.0, 1e-6);
  EXPECT_NEAR(output_cloud[0].z, 3.0, 1e-6);
}

TEST(LeveledFrameTransform, DefaultWorldOriginKeepsLegacyBehaviour) {
  fastlio2_to_ego_swarm_leveling::LeveledFrameTransform transform(
      Eigen::Vector3d(0.0, M_PI_2, 0.0));

  nav_msgs::msg::Odometry input;
  input.pose.pose.position.x = 1.0;
  input.pose.pose.orientation.w = 1.0;

  const auto output =
      transform.transform_odometry(input, "camera_init_level", "body_level");
  EXPECT_NEAR(output.pose.pose.position.z, -1.0, 1e-9);
}

TEST(LeveledFrameTransform, LevelsBodyAttitudeSoRestingTiltedImuReadsLevel) {
  // A MID360 mounted pitched 30 deg reports identity orientation at rest
  // (its body frame coincides with its tilted init frame). The leveled
  // output must present a level airframe attitude, i.e. identity.
  const double tilt = 0.5235987755982988;
  fastlio2_to_ego_swarm_leveling::LeveledFrameTransform transform(
      Eigen::Vector3d(0.0, tilt, 0.0));

  nav_msgs::msg::Odometry input;
  input.pose.pose.orientation.w = 1.0;

  const auto output =
      transform.transform_odometry(input, "camera_init_level", "body_level");

  EXPECT_NEAR(output.pose.pose.orientation.w, 1.0, 1e-9);
  EXPECT_NEAR(output.pose.pose.orientation.x, 0.0, 1e-9);
  EXPECT_NEAR(output.pose.pose.orientation.y, 0.0, 1e-9);
  EXPECT_NEAR(output.pose.pose.orientation.z, 0.0, 1e-9);
}

TEST(LeveledFrameTransform, MatchesLegacyLevelingPlusPx4BodyExtrinsicChain) {
  // Equivalence with the retired two-step chain: leveling rotated only the
  // world side, then fastlio2_to_px4_odometry post-multiplied the inverse
  // mount rotation as body_extrinsic_rpy_rad.
  const double tilt = 0.5235987755982988;
  const double yaw = 0.4;
  const Eigen::Vector3d origin(1.0, -2.0, 0.5);
  fastlio2_to_ego_swarm_leveling::LeveledFrameTransform transform(
      Eigen::Vector3d(0.0, tilt, 0.0), origin, yaw);

  const Eigen::Quaterniond q_in =
      Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX());

  nav_msgs::msg::Odometry input;
  input.pose.pose.orientation.w = q_in.w();
  input.pose.pose.orientation.x = q_in.x();
  input.pose.pose.orientation.y = q_in.y();
  input.pose.pose.orientation.z = q_in.z();

  const auto output =
      transform.transform_odometry(input, "camera_init_level", "body_level");

  const Eigen::Quaterniond mount(
      Eigen::AngleAxisd(tilt, Eigen::Vector3d::UnitY()));
  const Eigen::Quaterniond legacy =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * mount * q_in *
      mount.inverse();

  EXPECT_NEAR(output.pose.pose.orientation.w, legacy.w(), 1e-9);
  EXPECT_NEAR(output.pose.pose.orientation.x, legacy.x(), 1e-9);
  EXPECT_NEAR(output.pose.pose.orientation.y, legacy.y(), 1e-9);
  EXPECT_NEAR(output.pose.pose.orientation.z, legacy.z(), 1e-9);
}
