#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fastlivo_to_px4_odometry/frame_mapping.hpp"
#include "fastlivo_to_px4_odometry/odometry_conversion.hpp"

namespace fastlivo_to_px4_odometry
{

class FastLivoToPx4OdometryNode : public rclcpp::Node
{
public:
  FastLivoToPx4OdometryNode()
  : Node("fastlivo_to_px4_odometry")
  {
    const std::string input_topic = this->declare_parameter<std::string>(
      "input_topic", "/aft_mapped_to_init");
    const std::string output_topic = this->declare_parameter<std::string>(
      "output_topic", "/fmu/in/vehicle_visual_odometry");
    const std::vector<int64_t> world_axis_param = this->declare_parameter<std::vector<int64_t>>(
      "world_axis", {1, 0, 2});
    const std::vector<double> world_sign_param = this->declare_parameter<std::vector<double>>(
      "world_sign", {1.0, 1.0, -1.0});
    const std::vector<int64_t> body_axis_param = this->declare_parameter<std::vector<int64_t>>(
      "body_axis", {0, 1, 2});
    const std::vector<double> body_sign_param = this->declare_parameter<std::vector<double>>(
      "body_sign", {1.0, -1.0, -1.0});
    const std::vector<double> body_extrinsic_translation_param =
      this->declare_parameter<std::vector<double>>(
      "body_extrinsic_translation_m", {0.0, 0.0, 0.0});
    const std::vector<double> body_extrinsic_rpy_param =
      this->declare_parameter<std::vector<double>>(
      "body_extrinsic_rpy_rad", {0.0, 0.0, 0.0});
    const int quality_param = this->declare_parameter<int>("quality", 100);

    mapping_ = FrameMapping::default_enu_world_flu_body_to_ned_world_frd_body();
    if (!load_frame_mapping(
        world_axis_param, world_sign_param, body_axis_param, body_sign_param, &mapping_))
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid frame mapping parameters provided, using default ENU/FLU to NED/FRD mapping.");
    }

    body_extrinsic_translation_ =
      parse_vector3(body_extrinsic_translation_param, Eigen::Vector3d::Zero());
    body_extrinsic_rotation_ =
      quaternion_from_rpy(parse_vector3(body_extrinsic_rpy_param, Eigen::Vector3d::Zero()));

    quality_ = clamp_quality(quality_param);
    if (quality_ != quality_param) {
      RCLCPP_WARN(
        this->get_logger(),
        "Parameter quality=%d is out of range [-1, 100]; clamped to %d.",
        quality_param,
        quality_);
    }

    publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(output_topic, 10);
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      input_topic,
      20,
      std::bind(&FastLivoToPx4OdometryNode::handle_odometry, this, std::placeholders::_1));
  }

private:
  static int8_t clamp_quality(int quality)
  {
    if (quality < -1) {
      return static_cast<int8_t>(-1);
    }
    if (quality > 100) {
      return static_cast<int8_t>(100);
    }
    return static_cast<int8_t>(quality);
  }

  static bool parse_axis(
    const std::vector<int64_t> & values,
    std::array<int, 3> * axis_out)
  {
    if (values.size() != 3 || axis_out == nullptr) {
      return false;
    }

    std::array<bool, 3> used{false, false, false};
    for (std::size_t i = 0; i < 3; ++i) {
      const int axis = static_cast<int>(values[i]);
      if (axis < 0 || axis > 2 || used[static_cast<std::size_t>(axis)]) {
        return false;
      }
      used[static_cast<std::size_t>(axis)] = true;
      (*axis_out)[i] = axis;
    }
    return true;
  }

  static bool parse_sign(
    const std::vector<double> & values,
    std::array<double, 3> * sign_out)
  {
    if (values.size() != 3 || sign_out == nullptr) {
      return false;
    }

    for (std::size_t i = 0; i < 3; ++i) {
      const double sign = values[i];
      if (!std::isfinite(sign) || (sign != -1.0 && sign != 1.0)) {
        return false;
      }
      (*sign_out)[i] = sign;
    }
    return true;
  }

  static bool load_frame_mapping(
    const std::vector<int64_t> & world_axis_values,
    const std::vector<double> & world_sign_values,
    const std::vector<int64_t> & body_axis_values,
    const std::vector<double> & body_sign_values,
    FrameMapping * mapping_out)
  {
    if (mapping_out == nullptr) {
      return false;
    }

    FrameMapping mapping = *mapping_out;
    const bool ok = parse_axis(world_axis_values, &mapping.world_axis) &&
      parse_sign(world_sign_values, &mapping.world_sign) &&
      parse_axis(body_axis_values, &mapping.body_axis) &&
      parse_sign(body_sign_values, &mapping.body_sign);

    if (!ok) {
      return false;
    }

    *mapping_out = mapping;
    return true;
  }

  static Eigen::Vector3d parse_vector3(
    const std::vector<double> & values,
    const Eigen::Vector3d & fallback)
  {
    if (values.size() != 3) {
      return fallback;
    }

    for (double value : values) {
      if (!std::isfinite(value)) {
        return fallback;
      }
    }

    return Eigen::Vector3d(values[0], values[1], values[2]);
  }

  static Eigen::Quaterniond quaternion_from_rpy(const Eigen::Vector3d & rpy)
  {
    return Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX());
  }

  static uint64_t stamp_to_micros(const builtin_interfaces::msg::Time & stamp)
  {
    const int64_t sec = static_cast<int64_t>(stamp.sec);
    if (sec < 0) {
      return 0;
    }

    return static_cast<uint64_t>(sec) * 1000000ULL + static_cast<uint64_t>(stamp.nanosec) / 1000ULL;
  }

  static bool is_finite_odometry(const nav_msgs::msg::Odometry & msg)
  {
    const auto & p = msg.pose.pose.position;
    const auto & q = msg.pose.pose.orientation;
    const auto & v = msg.twist.twist.linear;

    const bool position_finite =
      std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
    const bool velocity_finite =
      std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    const bool quaternion_finite =
      std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z);
    if (!(position_finite && velocity_finite && quaternion_finite)) {
      return false;
    }

    const double q_norm = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    return q_norm > 1e-12;
  }

  void handle_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!is_finite_odometry(*msg)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Dropping odometry message with non-finite or invalid pose/twist values.");
      return;
    }

    const auto & p = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;
    const auto & v = msg->twist.twist.linear;

    const Eigen::Vector3d ros_position(p.x, p.y, p.z);
    const Eigen::Vector3d ros_velocity(v.x, v.y, v.z);
    Eigen::Quaterniond ros_orientation(q.w, q.x, q.y, q.z);
    ros_orientation.normalize();

    const ConvertedOdometry converted =
      convert_ros_world_to_px4_ned(
      ros_position,
      ros_velocity,
      ros_orientation,
      mapping_,
      body_extrinsic_translation_,
      body_extrinsic_rotation_);

    px4_msgs::msg::VehicleOdometry out{};
    out.timestamp = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000LL);
    out.timestamp_sample = stamp_to_micros(msg->header.stamp);
    out.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    out.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
    out.position = converted.position;
    out.q = converted.q;
    out.velocity = converted.velocity;
    out.reset_counter = 0;
    out.quality = quality_;

    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (std::size_t i = 0; i < 3; ++i) {
      out.angular_velocity[i] = nan;
      out.position_variance[i] = nan;
      out.orientation_variance[i] = nan;
      out.velocity_variance[i] = nan;
    }

    publisher_->publish(out);
  }

  FrameMapping mapping_{};
  Eigen::Vector3d body_extrinsic_translation_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond body_extrinsic_rotation_{Eigen::Quaterniond::Identity()};
  int8_t quality_{100};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr publisher_;
};

}  // namespace fastlivo_to_px4_odometry

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fastlivo_to_px4_odometry::FastLivoToPx4OdometryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
