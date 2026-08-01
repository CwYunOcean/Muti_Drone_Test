#include <chrono>
#include <memory>
#include <string>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class SyncNodeRos2 : public rclcpp::Node {
public:
  SyncNodeRos2()
  : Node("sync_node_ros2"),
    image_topic_(declare_parameter<std::string>("image_topic", "/camera/camera/infra1/image_rect_raw")),
    lidar_topic_(declare_parameter<std::string>("lidar_topic", "/livox/lidar")),
    synced_image_topic_(declare_parameter<std::string>("synced_image_topic", "/synced_image")),
    synced_lidar_topic_(declare_parameter<std::string>("synced_lidar_topic", "/synced_lidar")),
    publish_rate_(declare_parameter<double>("publish_rate", 10.0)),
    image_sub_(this, image_topic_),
    lidar_sub_(this, lidar_topic_),
    sync_(SyncPolicy(10), image_sub_, lidar_sub_) {
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(synced_image_topic_, 10);
    lidar_pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(synced_lidar_topic_, 10);

    sync_.registerCallback(std::bind(&SyncNodeRos2::callback, this, _1, _2));

    auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SyncNodeRos2::timer_callback, this));
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    livox_ros_driver2::msg::CustomMsg>;

  void callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & lidar_msg) {
    last_image_msg_ = std::make_shared<sensor_msgs::msg::Image>(*image_msg);
    last_lidar_msg_ = std::make_shared<livox_ros_driver2::msg::CustomMsg>(*lidar_msg);
    new_image_received_ = true;
    new_lidar_received_ = true;
  }

  void timer_callback() {
    if (!new_image_received_ || !new_lidar_received_) {
      return;
    }

    image_pub_->publish(*last_image_msg_);
    lidar_pub_->publish(*last_lidar_msg_);
    new_image_received_ = false;
    new_lidar_received_ = false;
  }

  std::string image_topic_;
  std::string lidar_topic_;
  std::string synced_image_topic_;
  std::string synced_lidar_topic_;
  double publish_rate_;

  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
  message_filters::Subscriber<livox_ros_driver2::msg::CustomMsg> lidar_sub_;
  message_filters::Synchronizer<SyncPolicy> sync_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::Image::SharedPtr last_image_msg_;
  livox_ros_driver2::msg::CustomMsg::SharedPtr last_lidar_msg_;
  bool new_image_received_ = false;
  bool new_lidar_received_ = false;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SyncNodeRos2>());
  rclcpp::shutdown();
  return 0;
}
