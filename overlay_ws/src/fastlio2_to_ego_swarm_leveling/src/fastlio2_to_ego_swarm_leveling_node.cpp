#include "fastlio2_to_ego_swarm_leveling/leveled_frame_transform.hpp"

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace {

Eigen::Quaterniond quaternionFromRpy(const Eigen::Vector3d& rpy_rad) {
  const Eigen::AngleAxisd roll_angle(rpy_rad.x(), Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch_angle(rpy_rad.y(), Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw_angle(rpy_rad.z(), Eigen::Vector3d::UnitZ());
  return yaw_angle * pitch_angle * roll_angle;
}

}  // namespace

class Fastlio2ToEgoSwarmLevelingNode : public rclcpp::Node {
 public:
  Fastlio2ToEgoSwarmLevelingNode()
      : Node("fastlio2_to_ego_swarm_leveling"),
        level_rpy_rad_(declareLevelRpy()),
        world_origin_xyz_(declareWorldOriginXyz()),
        world_origin_yaw_rad_(
            declare_parameter<double>("world_origin_yaw_rad", 0.0)),
        raw_from_leveled_quaternion_(
            (Eigen::AngleAxisd(world_origin_yaw_rad_, Eigen::Vector3d::UnitZ()) *
             quaternionFromRpy(level_rpy_rad_))
                .inverse()),
        transform_(level_rpy_rad_, world_origin_xyz_, world_origin_yaw_rad_),
        tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this)),
        static_tf_broadcaster_(
            std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this)) {
    input_odom_topic_ = declare_parameter<std::string>(
        "input_odom_topic", "/drone_1/aft_mapped_to_init");
    input_cloud_topic_ = declare_parameter<std::string>(
        "input_cloud_topic", "/drone_1/cloud_registered");
    output_odom_topic_ = declare_parameter<std::string>(
        "output_odom_topic", "/drone_1/aft_mapped_to_init_level");
    output_cloud_topic_ = declare_parameter<std::string>(
        "output_cloud_topic", "/drone_1/cloud_registered_level");
    world_frame_id_ = declare_parameter<std::string>(
        "world_frame_id", "world");
    output_frame_id_ = declare_parameter<std::string>(
        "output_frame_id", "camera_init_level");
    output_child_frame_id_ = declare_parameter<std::string>(
        "output_child_frame_id", "body_level");

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        output_odom_topic_, 20);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        output_cloud_topic_, 20);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        input_odom_topic_,
        20,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          const auto leveled_odom = transform_.transform_odometry(
              *msg, output_frame_id_, output_child_frame_id_);
          publishStaticLevelFrames(*msg);
          publishDynamicBodyFrame(leveled_odom);
          odom_pub_->publish(leveled_odom);
        });

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_cloud_topic_,
        20,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          cloud_pub_->publish(
              transform_.transform_cloud(*msg, output_frame_id_));
        });
  }

 private:
  Eigen::Vector3d declareLevelRpy() {
    const auto rpy = declare_parameter<std::vector<double>>(
        "level_rpy_rad",
        std::vector<double>{0.0, 0.5235987755982988, 0.0});
    return Eigen::Vector3d(rpy.at(0), rpy.at(1), rpy.at(2));
  }

  Eigen::Vector3d declareWorldOriginXyz() {
    const auto xyz = declare_parameter<std::vector<double>>(
        "world_origin_xyz", std::vector<double>{0.0, 0.0, 0.0});
    return Eigen::Vector3d(xyz.at(0), xyz.at(1), xyz.at(2));
  }

  void publishStaticLevelFrames(const nav_msgs::msg::Odometry& input_odom) {
    if (static_level_frame_published_ ||
        input_odom.header.frame_id.empty() ||
        input_odom.header.frame_id == output_frame_id_) {
      return;
    }

    geometry_msgs::msg::TransformStamped world_to_leveled;
    world_to_leveled.header.stamp = now();
    world_to_leveled.header.frame_id = world_frame_id_;
    world_to_leveled.child_frame_id = output_frame_id_;
    world_to_leveled.transform.translation.x = 0.0;
    world_to_leveled.transform.translation.y = 0.0;
    world_to_leveled.transform.translation.z = 0.0;
    world_to_leveled.transform.rotation.w = 1.0;
    world_to_leveled.transform.rotation.x = 0.0;
    world_to_leveled.transform.rotation.y = 0.0;
    world_to_leveled.transform.rotation.z = 0.0;

    geometry_msgs::msg::TransformStamped leveled_to_raw;
    leveled_to_raw.header.stamp = world_to_leveled.header.stamp;
    leveled_to_raw.header.frame_id = output_frame_id_;
    leveled_to_raw.child_frame_id = input_odom.header.frame_id;
    const Eigen::Vector3d leveled_to_raw_translation =
        raw_from_leveled_quaternion_ * (-world_origin_xyz_);
    leveled_to_raw.transform.translation.x = leveled_to_raw_translation.x();
    leveled_to_raw.transform.translation.y = leveled_to_raw_translation.y();
    leveled_to_raw.transform.translation.z = leveled_to_raw_translation.z();
    leveled_to_raw.transform.rotation.w = raw_from_leveled_quaternion_.w();
    leveled_to_raw.transform.rotation.x = raw_from_leveled_quaternion_.x();
    leveled_to_raw.transform.rotation.y = raw_from_leveled_quaternion_.y();
    leveled_to_raw.transform.rotation.z = raw_from_leveled_quaternion_.z();

    static_tf_broadcaster_->sendTransform(world_to_leveled);
    static_tf_broadcaster_->sendTransform(leveled_to_raw);
    static_level_frame_published_ = true;
  }

  void publishDynamicBodyFrame(const nav_msgs::msg::Odometry& leveled_odom) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = leveled_odom.header.stamp;
    transform.header.frame_id = leveled_odom.header.frame_id;
    transform.child_frame_id = leveled_odom.child_frame_id;
    transform.transform.translation.x = leveled_odom.pose.pose.position.x;
    transform.transform.translation.y = leveled_odom.pose.pose.position.y;
    transform.transform.translation.z = leveled_odom.pose.pose.position.z;
    transform.transform.rotation = leveled_odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  Eigen::Vector3d level_rpy_rad_;
  Eigen::Vector3d world_origin_xyz_;
  double world_origin_yaw_rad_;
  Eigen::Quaterniond raw_from_leveled_quaternion_;
  fastlio2_to_ego_swarm_leveling::LeveledFrameTransform transform_;
  std::string input_odom_topic_;
  std::string input_cloud_topic_;
  std::string output_odom_topic_;
  std::string output_cloud_topic_;
  std::string world_frame_id_;
  std::string output_frame_id_;
  std::string output_child_frame_id_;
  bool static_level_frame_published_{false};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Fastlio2ToEgoSwarmLevelingNode>());
  rclcpp::shutdown();
  return 0;
}
