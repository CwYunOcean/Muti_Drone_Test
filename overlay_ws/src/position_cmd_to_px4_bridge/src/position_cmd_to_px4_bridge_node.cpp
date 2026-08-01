#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>

#include "position_cmd_to_px4_bridge/bridge_input_builder.hpp"
#include "position_cmd_to_px4_bridge/bridge_state_machine.hpp"
#include "position_cmd_to_px4_bridge/frame_mapping.hpp"
#include "position_cmd_to_px4_bridge/offboard_control_mode_builder.hpp"
#include "position_cmd_to_px4_bridge/px4_qos_profiles.hpp"
#include "position_cmd_to_px4_bridge/trajectory_setpoint_conversion.hpp"

namespace position_cmd_to_px4_bridge
{

namespace
{

const char * bridge_state_name(const BridgeState state)
{
  switch (state) {
    case BridgeState::IDLE:
      return "IDLE";
    case BridgeState::WAIT_FASTLIVO:
      return "WAIT_FASTLIVO";
    case BridgeState::WAIT_PX4:
      return "WAIT_PX4";
    case BridgeState::STREAM_SETPOINT:
      return "STREAM_SETPOINT";
    case BridgeState::ENTER_OFFBOARD:
      return "ENTER_OFFBOARD";
    case BridgeState::ARM:
      return "ARM";
    case BridgeState::ACTIVE:
      return "ACTIVE";
    case BridgeState::FAILSAFE:
      return "FAILSAFE";
    default:
      return "UNKNOWN";
  }
}

}  // namespace

class PositionCmdToPx4BridgeNode final : public rclcpp::Node
{
public:
  PositionCmdToPx4BridgeNode()
  : Node("position_cmd_to_px4_bridge")
  {
    const int warmup_cycles = declare_parameter<int>("warmup_cycles", 10);
    auto_request_offboard_and_arm_ =
      declare_parameter<bool>("auto_request_offboard_and_arm", false);
    state_machine_ = BridgeStateMachine(warmup_cycles, auto_request_offboard_and_arm_);
    command_topic_ =
      declare_parameter<std::string>("command_topic", "/drone_1_planning/pos_cmd");
    offboard_mode_topic_ = declare_parameter<std::string>(
      "offboard_mode_topic", "/drone_1/fmu/in/offboard_control_mode");
    trajectory_setpoint_topic_ = declare_parameter<std::string>(
      "trajectory_setpoint_topic", "/drone_1/fmu/in/trajectory_setpoint");
    vehicle_command_topic_ = declare_parameter<std::string>(
      "vehicle_command_topic", "/drone_1/fmu/in/vehicle_command");
    vehicle_odometry_topic_ = declare_parameter<std::string>(
      "vehicle_odometry_topic", "/drone_1/fmu/out/vehicle_odometry");
    vehicle_status_topic_ =
      declare_parameter<std::string>("vehicle_status_topic", "");
    fastlivo_odom_topic_ =
      declare_parameter<std::string>("fastlivo_odom_topic", "/drone_1/aft_mapped_to_init_level");
    target_system_ =
      declare_parameter<int>("target_system", 2);
    enabled_ = declare_parameter<bool>("enabled", true);
    command_timeout_ms_ = clamp_nonnegative(declare_parameter<int>("command_timeout_ms", 300));
    status_timeout_ms_ = clamp_nonnegative(declare_parameter<int>("status_timeout_ms", 1000));
    odom_timeout_ms_ = clamp_nonnegative(declare_parameter<int>("odom_timeout_ms", 300));
    position_control_enabled_ =
      declare_parameter<bool>("position_control_enabled", true);
    velocity_control_enabled_ =
      declare_parameter<bool>("velocity_control_enabled", true);
    use_acceleration_feedforward_ =
      declare_parameter<bool>("use_acceleration_feedforward", false);
    world_axis_ = declare_parameter<std::vector<int64_t>>("world_axis", {1, 0, 2});
    world_sign_ = declare_parameter<std::vector<double>>("world_sign", {1.0, 1.0, -1.0});

    mapping_ = FrameMapping::default_enu_world_to_ned_world();
    if (!load_mapping(world_axis_, world_sign_, &mapping_)) {
      RCLCPP_WARN(
        get_logger(),
        "Invalid world_axis/world_sign provided. Falling back to default ENU->NED mapping.");
    }

    command_sub_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
      command_topic_, 10,
      std::bind(&PositionCmdToPx4BridgeNode::command_callback, this, std::placeholders::_1));
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      fastlivo_odom_topic_, 10,
      std::bind(&PositionCmdToPx4BridgeNode::odometry_callback, this, std::placeholders::_1));
    auto vehicle_status_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    vehicle_status_qos.best_effort();
    vehicle_status_qos.transient_local();

    vehicle_odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      vehicle_odometry_topic_, vehicle_status_qos,
      std::bind(
        &PositionCmdToPx4BridgeNode::vehicle_odometry_callback,
        this,
        std::placeholders::_1));
    if (!vehicle_status_topic_.empty()) {
      vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
        vehicle_status_topic_, vehicle_status_qos,
        std::bind(
          &PositionCmdToPx4BridgeNode::vehicle_status_callback,
          this,
          std::placeholders::_1));
    }

    const auto px4_input_qos = make_px4_input_qos();
    offboard_mode_pub_ =
      create_publisher<px4_msgs::msg::OffboardControlMode>(offboard_mode_topic_, px4_input_qos);
    trajectory_setpoint_pub_ =
      create_publisher<px4_msgs::msg::TrajectorySetpoint>(trajectory_setpoint_topic_, px4_input_qos);
    vehicle_command_pub_ =
      create_publisher<px4_msgs::msg::VehicleCommand>(vehicle_command_topic_, px4_input_qos);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&PositionCmdToPx4BridgeNode::tick, this));
  }

private:
  static int clamp_nonnegative(int value)
  {
    return value < 0 ? 0 : value;
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

  static bool load_mapping(
    const std::vector<int64_t> & world_axis_values,
    const std::vector<double> & world_sign_values,
    FrameMapping * mapping_out)
  {
    if (mapping_out == nullptr) {
      return false;
    }

    FrameMapping mapping = *mapping_out;
    const bool ok = parse_axis(world_axis_values, &mapping.world_axis) &&
      parse_sign(world_sign_values, &mapping.world_sign);
    if (!ok) {
      return false;
    }

    *mapping_out = mapping;
    return true;
  }

  uint64_t now_micros() const
  {
    return static_cast<uint64_t>(now().nanoseconds() / 1000LL);
  }

  void command_callback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
  {
    last_command_ = *msg;
    last_command_time_ = now();
    has_command_ = true;
    RCLCPP_INFO_ONCE(
      get_logger(),
      "received first PositionCommand trajectory_id=%d flag=%d",
      msg->trajectory_id,
      msg->trajectory_flag);
  }

  void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
  {
    last_vehicle_status_ = *msg;
    last_status_time_ = now();
    has_vehicle_status_ = true;
  }

  void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr)
  {
    last_px4_heartbeat_time_ = now();
    has_px4_heartbeat_ = true;
    RCLCPP_INFO_ONCE(get_logger(), "received first VehicleOdometry heartbeat");
  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr)
  {
    last_odom_time_ = now();
    has_odom_ = true;
    RCLCPP_INFO_ONCE(get_logger(), "received first FAST-LIVO odometry");
  }

  bool command_fresh() const
  {
    if (!has_command_) {
      return false;
    }
    return (now() - last_command_time_).nanoseconds() <
      static_cast<int64_t>(command_timeout_ms_) * 1000000LL;
  }

  bool px4_heartbeat_fresh() const
  {
    if (!has_px4_heartbeat_) {
      return false;
    }
    return (now() - last_px4_heartbeat_time_).nanoseconds() <
      static_cast<int64_t>(status_timeout_ms_) * 1000000LL;
  }

  bool fastlivo_fresh() const
  {
    if (!has_odom_) {
      return false;
    }
    return (now() - last_odom_time_).nanoseconds() <
      static_cast<int64_t>(odom_timeout_ms_) * 1000000LL;
  }

  void publish_offboard_control_mode() const
  {
    const OffboardControlModeSelection selection =
      resolve_offboard_control_mode_selection(
      position_control_enabled_,
      velocity_control_enabled_);

    if (!position_control_enabled_ && !velocity_control_enabled_) {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "Both position_control_enabled and velocity_control_enabled are false; "
        "falling back to velocity control for OffboardControlMode.");
    }

    const px4_msgs::msg::OffboardControlMode msg = build_offboard_control_mode(
      now_micros(),
      selection.position,
      selection.velocity,
      use_acceleration_feedforward_);
    offboard_mode_pub_->publish(msg);
  }

  void publish_vehicle_command(
    uint16_t command,
    float param1 = 0.0f,
    float param2 = 0.0f) const
  {
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = now_micros();
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = static_cast<uint8_t>(target_system_);
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    vehicle_command_pub_->publish(msg);
  }

  void publish_trajectory_setpoint() const
  {
    const ConvertedTrajectorySetpoint converted =
      convert_position_command(last_command_, mapping_);

    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = now_micros();
    msg.position = converted.position;
    msg.velocity = converted.velocity;
    if (use_acceleration_feedforward_) {
      msg.acceleration = converted.acceleration;
    } else {
      msg.acceleration = {
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN()};
    }
    msg.yaw = converted.yaw;
    msg.yawspeed = converted.yaw_rate;
    trajectory_setpoint_pub_->publish(msg);
  }

  void tick()
  {
    const bool fastlivo_fresh_now = fastlivo_fresh();
    const bool px4_heartbeat_fresh_now = px4_heartbeat_fresh();
    const bool command_fresh_now = command_fresh();
    VehicleStatusSnapshot vehicle_status{};
    vehicle_status.available = has_vehicle_status_;
    vehicle_status.nav_state = last_vehicle_status_.nav_state;
    vehicle_status.arming_state = last_vehicle_status_.arming_state;
    const BridgeInputs inputs = build_bridge_inputs(
      enabled_,
      fastlivo_fresh_now,
      px4_heartbeat_fresh_now,
      command_fresh_now,
      vehicle_status);

    const BridgeOutputs outputs = state_machine_.tick(inputs);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "state=%s enabled=%d fastlivo=%d heartbeat=%d command=%d stream=%d offboard_req=%d arm_req=%d "
      "has_cmd=%d has_odom=%d has_px4_heartbeat=%d",
      bridge_state_name(outputs.state),
      enabled_ ? 1 : 0,
      fastlivo_fresh_now ? 1 : 0,
      px4_heartbeat_fresh_now ? 1 : 0,
      command_fresh_now ? 1 : 0,
      outputs.stream_setpoint ? 1 : 0,
      outputs.request_offboard ? 1 : 0,
      outputs.request_arm ? 1 : 0,
      has_command_ ? 1 : 0,
      has_odom_ ? 1 : 0,
      has_px4_heartbeat_ ? 1 : 0);

    if (outputs.stream_setpoint && has_command_) {
      publish_offboard_control_mode();
      publish_trajectory_setpoint();
    }

    if (outputs.request_offboard) {
      publish_vehicle_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
        1.0f,
        6.0f);
    }

    if (outputs.request_arm) {
      publish_vehicle_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        1.0f,
        0.0f);
    }

    if (outputs.request_land) {
      publish_vehicle_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND,
        0.0f,
        0.0f);
    }
  }

  std::string command_topic_;
  std::string offboard_mode_topic_;
  std::string trajectory_setpoint_topic_;
  std::string vehicle_command_topic_;
  std::string vehicle_odometry_topic_;
  std::string vehicle_status_topic_;
  std::string fastlivo_odom_topic_;
  bool enabled_{true};
  bool auto_request_offboard_and_arm_{false};
  int target_system_{2};
  int command_timeout_ms_{300};
  int status_timeout_ms_{1000};
  int odom_timeout_ms_{300};
  bool position_control_enabled_{true};
  bool velocity_control_enabled_{true};
  bool use_acceleration_feedforward_{false};
  std::vector<int64_t> world_axis_;
  std::vector<double> world_sign_;
  FrameMapping mapping_{};
  BridgeStateMachine state_machine_{10, false};
  bool has_command_{false};
  bool has_px4_heartbeat_{false};
  bool has_vehicle_status_{false};
  bool has_odom_{false};
  quadrotor_msgs::msg::PositionCommand last_command_{};
  px4_msgs::msg::VehicleStatus last_vehicle_status_{};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_px4_heartbeat_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
};

}  // namespace position_cmd_to_px4_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<position_cmd_to_px4_bridge::PositionCmdToPx4BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
