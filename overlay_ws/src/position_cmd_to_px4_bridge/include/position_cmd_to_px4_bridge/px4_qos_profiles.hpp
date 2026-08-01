#pragma once

#include <rclcpp/rclcpp.hpp>

namespace position_cmd_to_px4_bridge
{

inline rclcpp::QoS make_px4_input_qos()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.best_effort();
  qos.durability_volatile();
  return qos;
}

}  // namespace position_cmd_to_px4_bridge
