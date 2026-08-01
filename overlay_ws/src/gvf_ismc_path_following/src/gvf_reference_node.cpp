#include <functional>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <gvf_path_following_msgs/msg/gvf_reference.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "gvf_ismc_path_following/three_leaf_gvf.hpp"

namespace gvf_ismc_path_following
{

class GvfReferenceNode final : public rclcpp::Node
{
public:
  GvfReferenceNode()
  : Node("gvf_reference_node")
  {
    odom_topic_ =
      declare_parameter<std::string>("odom_topic", "/drone_1/aft_mapped_to_init_level");
    reference_topic_ =
      declare_parameter<std::string>("reference_topic", "/gvf/reference");
    reference_path_topic_ =
      declare_parameter<std::string>("reference_path_topic", "/gvf/reference_path");
    reference_marker_topic_ =
      declare_parameter<std::string>("reference_marker_topic", "/gvf/reference_marker");
    actual_path_topic_ =
      declare_parameter<std::string>("actual_path_topic", "/gvf/actual_path");
    reference_frame_id_ =
      declare_parameter<std::string>("reference_frame_id", "camera_init_level");
    reference_sample_count_ =
      declare_parameter<int>("reference_sample_count", 240);
    actual_path_max_length_ =
      declare_parameter<int>("actual_path_max_length", 4000);

    trajectory_type_ =
      declare_parameter<std::string>("trajectory_type", "circular");

    common_params_.target_height_m = declare_parameter<double>("target_height_m", 1.2);
    common_params_.lateral_gain = declare_parameter<double>("lateral_gain", 1.0);
    common_params_.vertical_gain = declare_parameter<double>("vertical_gain", 1.0);
    common_params_.max_speed_mps = declare_parameter<double>("max_speed_mps", 0.5);
    common_params_.min_planar_speed_for_yaw_mps =
      declare_parameter<double>("min_planar_speed_for_yaw_mps", 0.05);
    common_params_.yaw_alpha = declare_parameter<double>("yaw_alpha", 0.1);

    if (trajectory_type_ == "three_leaf") {
      three_leaf_params_.target_height_m = common_params_.target_height_m;
      three_leaf_params_.base_radius_m = declare_parameter<double>("base_radius_m", 2.0);
      three_leaf_params_.lobe_amplitude_m =
        declare_parameter<double>("lobe_amplitude_m", 0.5);
      three_leaf_params_.lateral_gain = common_params_.lateral_gain;
      three_leaf_params_.vertical_gain = common_params_.vertical_gain;
      three_leaf_params_.max_speed_mps = common_params_.max_speed_mps;
      three_leaf_params_.min_planar_speed_for_yaw_mps =
        common_params_.min_planar_speed_for_yaw_mps;
      three_leaf_params_.yaw_alpha = common_params_.yaw_alpha;
    } else {
      circular_params_.target_height_m = common_params_.target_height_m;
      circular_params_.radius_m = declare_parameter<double>("radius_m", 2.0);
      circular_params_.lateral_gain = common_params_.lateral_gain;
      circular_params_.vertical_gain = common_params_.vertical_gain;
      circular_params_.max_speed_mps = common_params_.max_speed_mps;
      circular_params_.min_planar_speed_for_yaw_mps =
        common_params_.min_planar_speed_for_yaw_mps;
      circular_params_.yaw_alpha = common_params_.yaw_alpha;
    }

    reference_pub_ = create_publisher<gvf_path_following_msgs::msg::GVFReference>(
      reference_topic_, 10);
    reference_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      reference_path_topic_, 10);
    reference_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      reference_marker_topic_, 10);
    actual_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      actual_path_topic_, 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&GvfReferenceNode::odometry_callback, this, std::placeholders::_1));

    build_reference_visualization();
    visualization_timer_ = create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&GvfReferenceNode::publish_reference_visualization, this));
  }

private:
  void build_reference_visualization()
  {
    const int sample_count = std::max(reference_sample_count_, 12);
    const double target_height = common_params_.target_height_m;

    reference_path_.header.frame_id = reference_frame_id_;
    reference_path_.poses.clear();
    reference_path_.poses.reserve(static_cast<std::size_t>(sample_count) + 1U);

    reference_marker_.header.frame_id = reference_frame_id_;
    reference_marker_.ns = "gvf_reference";
    reference_marker_.id = 0;
    reference_marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    reference_marker_.action = visualization_msgs::msg::Marker::ADD;
    reference_marker_.pose.orientation.w = 1.0;
    reference_marker_.scale.x = 0.05;
    reference_marker_.color.r = 0.1F;
    reference_marker_.color.g = 0.95F;
    reference_marker_.color.b = 0.95F;
    reference_marker_.color.a = 1.0F;
    reference_marker_.points.clear();
    reference_marker_.points.reserve(static_cast<std::size_t>(sample_count) + 1U);

    for (int i = 0; i <= sample_count; ++i) {
      const double alpha = static_cast<double>(i) / static_cast<double>(sample_count);
      const double theta = alpha * 2.0 * M_PI;

      double radius;
      if (trajectory_type_ == "three_leaf") {
        radius = three_leaf_params_.base_radius_m +
          three_leaf_params_.lobe_amplitude_m * std::cos(3.0 * theta);
      } else {
        radius = circular_params_.radius_m;
      }

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = reference_frame_id_;
      pose.pose.position.x = radius * std::cos(theta);
      pose.pose.position.y = radius * std::sin(theta);
      pose.pose.position.z = target_height;
      pose.pose.orientation.w = 1.0;
      reference_path_.poses.push_back(pose);

      geometry_msgs::msg::Point point;
      point.x = pose.pose.position.x;
      point.y = pose.pose.position.y;
      point.z = pose.pose.position.z;
      reference_marker_.points.push_back(point);
    }
  }

  void publish_reference_visualization()
  {
    const auto stamp = now();
    reference_path_.header.stamp = stamp;
    reference_marker_.header.stamp = stamp;
    for (auto & pose : reference_path_.poses) {
      pose.header.stamp = stamp;
    }
    reference_path_pub_->publish(reference_path_);
    reference_marker_pub_->publish(reference_marker_);
  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const Eigen::Vector3d position(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z);

    if (actual_path_.header.frame_id.empty()) {
      actual_path_.header.frame_id = msg->header.frame_id;
    }
    actual_path_.header.stamp = msg->header.stamp;
    geometry_msgs::msg::PoseStamped actual_pose;
    actual_pose.header = msg->header;
    actual_pose.pose = msg->pose.pose;
    actual_path_.poses.push_back(actual_pose);
    const std::size_t max_length = static_cast<std::size_t>(std::max(actual_path_max_length_, 2));
    if (actual_path_.poses.size() > max_length) {
      actual_path_.poses.erase(actual_path_.poses.begin());
    }
    actual_path_pub_->publish(actual_path_);

    GvfResult result;
    if (trajectory_type_ == "three_leaf") {
      result = evaluate_three_leaf_gvf(position, previous_yaw_, three_leaf_params_);
    } else {
      result = evaluate_circular_gvf(position, previous_yaw_, circular_params_);
    }
    previous_yaw_ = result.desired_yaw;

    gvf_path_following_msgs::msg::GVFReference reference{};
    reference.header = msg->header;
    reference.desired_velocity.x = result.desired_velocity.x();
    reference.desired_velocity.y = result.desired_velocity.y();
    reference.desired_velocity.z = result.desired_velocity.z();
    reference.desired_yaw = result.desired_yaw;
    reference.desired_yaw_rate = 0.0;
    reference.phi1 = result.phi1;
    reference.phi2 = result.phi2;
    reference_pub_->publish(reference);
  }

  struct CommonGvfParameters
  {
    double target_height_m{1.2};
    double lateral_gain{1.0};
    double vertical_gain{1.0};
    double max_speed_mps{0.5};
    double min_planar_speed_for_yaw_mps{0.05};
    double yaw_alpha{0.1};
  };

  std::string odom_topic_;
  std::string reference_topic_;
  std::string reference_path_topic_;
  std::string reference_marker_topic_;
  std::string actual_path_topic_;
  std::string reference_frame_id_;
  std::string trajectory_type_;
  int reference_sample_count_{240};
  int actual_path_max_length_{4000};
  CommonGvfParameters common_params_{};
  ThreeLeafGvfParameters three_leaf_params_{};
  CircularGvfParameters circular_params_{};
  double previous_yaw_{0.0};
  nav_msgs::msg::Path reference_path_{};
  nav_msgs::msg::Path actual_path_{};
  visualization_msgs::msg::Marker reference_marker_{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<gvf_path_following_msgs::msg::GVFReference>::SharedPtr reference_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr reference_marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_pub_;
  rclcpp::TimerBase::SharedPtr visualization_timer_;
};

}  // namespace gvf_ismc_path_following

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gvf_ismc_path_following::GvfReferenceNode>());
  rclcpp::shutdown();
  return 0;
}
