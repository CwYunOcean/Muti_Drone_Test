#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include <Eigen/Core>

#include <gvf_path_following_msgs/msg/gvf_reference.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>

#include "gvf_ismc_path_following/ismc_outer_loop.hpp"

namespace gvf_ismc_path_following
{

class IsmcVelocityTrackerNode final : public rclcpp::Node
{
public:
  IsmcVelocityTrackerNode()
  : Node("ismc_velocity_tracker_node")
  {
    odom_topic_ =
      declare_parameter<std::string>("odom_topic", "/drone_1/aft_mapped_to_init_level");
    reference_topic_ =
      declare_parameter<std::string>("reference_topic", "/gvf/reference");
    command_topic_ = declare_parameter<std::string>("command_topic", "/drone_1_planning/pos_cmd");
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    use_px4_position_hold_for_z_ =
      declare_parameter<bool>("use_px4_position_hold_for_z", false);
    target_height_m_ = declare_parameter<double>("target_height_m", 1.2);
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 50.0);
    input_timeout_sec_ = declare_parameter<double>("input_timeout_sec", 0.5);
    publish_position_nan_ = declare_parameter<bool>("publish_position_nan", true);
    outer_loop_parameters_.lambda = declare_parameter<double>("lambda", 2.0);
    outer_loop_parameters_.k = declare_parameter<double>("k", 1.0);
    outer_loop_parameters_.gamma = declare_parameter<double>("gamma", 0.0);
    outer_loop_parameters_.c1 = declare_parameter<double>("c1", 0.05);
    outer_loop_parameters_.epsilon = declare_parameter<double>("epsilon", 0.01);
    outer_loop_parameters_.adaptation_gain = declare_parameter<double>("adaptation_gain", 0.0);
    outer_loop_parameters_.max_acceleration_mps2 =
      declare_parameter<double>("max_acceleration_mps2", 3.0);
    outer_loop_parameters_.enable_adaptation =
      declare_parameter<bool>("enable_adaptation", false);

    command_pub_ = create_publisher<quadrotor_msgs::msg::PositionCommand>(
      command_topic_, 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&IsmcVelocityTrackerNode::odometry_callback, this, std::placeholders::_1));
    reference_sub_ = create_subscription<gvf_path_following_msgs::msg::GVFReference>(
      reference_topic_, 10,
      std::bind(&IsmcVelocityTrackerNode::reference_callback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(
      1.0 / ((control_rate_hz_ > 1e-6) ? control_rate_hz_ : 50.0));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&IsmcVelocityTrackerNode::publish_command, this));
  }

private:
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_ = *msg;
    last_odom_received_time_ = now();
    has_odom_ = true;
  }

  void reference_callback(const gvf_path_following_msgs::msg::GVFReference::SharedPtr msg)
  {
    last_reference_ = *msg;
    last_reference_received_time_ = now();
    has_reference_ = true;
  }

  bool odom_is_fresh(const rclcpp::Time & now_time) const
  {
    if (!has_odom_) {
      return false;
    }
    return (now_time - last_odom_received_time_).seconds() <= input_timeout_sec_;
  }

  bool reference_is_fresh(const rclcpp::Time & now_time) const
  {
    if (!has_reference_) {
      return false;
    }
    return (now_time - last_reference_received_time_).seconds() <= input_timeout_sec_;
  }

  void publish_command()
  {
    const rclcpp::Time now_time = now();
    if (!odom_is_fresh(now_time) || !reference_is_fresh(now_time)) {
      return;
    }

    double dt_seconds = 1.0 / ((control_rate_hz_ > 1e-6) ? control_rate_hz_ : 50.0);
    if (previous_command_time_.nanoseconds() > 0) {
      dt_seconds = (now_time - previous_command_time_).seconds();
    }
    previous_command_time_ = now_time;

    const Eigen::Vector3d desired_velocity_xy(
      last_reference_.desired_velocity.x,
      last_reference_.desired_velocity.y,
      0.0);
    const Eigen::Vector3d current_velocity_xy(
      last_odom_.twist.twist.linear.x,
      last_odom_.twist.twist.linear.y,
      0.0);

    const auto output = step_ismc_outer_loop(
      desired_velocity_xy,
      current_velocity_xy,
      dt_seconds,
      outer_loop_parameters_,
      &outer_loop_state_);

    quadrotor_msgs::msg::PositionCommand command{};
    command.header.stamp = now_time;
    command.header.frame_id = frame_id_;

    if (publish_position_nan_) {
      command.position.x = std::numeric_limits<double>::quiet_NaN();
      command.position.y = std::numeric_limits<double>::quiet_NaN();
      command.position.z = std::numeric_limits<double>::quiet_NaN();
    } else {
      command.position = last_odom_.pose.pose.position;
    }

    command.velocity.x = last_reference_.desired_velocity.x;
    command.velocity.y = last_reference_.desired_velocity.y;
    command.velocity.z = last_reference_.desired_velocity.z;
    command.acceleration.x = output.acceleration_command.x();
    command.acceleration.y = output.acceleration_command.y();
    command.acceleration.z = 0.0;

    command.yaw = last_reference_.desired_yaw;
    command.yaw_dot = last_reference_.desired_yaw_rate;
    command_pub_->publish(command);
  }

  std::string odom_topic_;
  std::string reference_topic_;
  std::string command_topic_;
  std::string frame_id_;
  bool use_px4_position_hold_for_z_{false};
  double target_height_m_{1.2};
  double control_rate_hz_{50.0};
  double input_timeout_sec_{0.5};
  bool publish_position_nan_{true};
  IsmcOuterLoopParameters outer_loop_parameters_{};
  IsmcOuterLoopState outer_loop_state_{};
  bool has_odom_{false};
  bool has_reference_{false};
  nav_msgs::msg::Odometry last_odom_{};
  gvf_path_following_msgs::msg::GVFReference last_reference_{};
  rclcpp::Time previous_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_received_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_reference_received_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<gvf_path_following_msgs::msg::GVFReference>::SharedPtr reference_sub_;
  rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace gvf_ismc_path_following

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gvf_ismc_path_following::IsmcVelocityTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
