#pragma once
#include "curve_base.hpp"
#include <cmath>
#include <vector>
#include <Eigen/Dense>

// [新增] 引入 ROS 相关头文件
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon.hpp>

namespace distribute_control
{

    class BsplineCurve : public CurveBase
    {
    public:
        BsplineCurve() = default;

        // 1. 必须实现的接口 (多态)
        // 对于 B 样条，这个函数可以留空，或者用来给个默认形状
        void load_params(const std::vector<double> &params) override;

        Eigen::Vector3d get_position(double w, double height) override;
        Eigen::Vector3d get_tangent(double w) override;
        std::string get_type() const override { return "bspline"; }

        // 2. [新增] 专属接口：启动 ROS 订阅
        // 传入 node 指针，让这个类寄生在 OffboardNode 上进行通信
        void start_subscription(rclcpp::Node *node);
        void fit_waypoints_and_update(const std::vector<Eigen::Vector2d> &waypoints);

    private:
        std::vector<Eigen::Vector2d> control_points_;
        int num_points_ = 0;
        bool valid_ = false;

        // [新增] 订阅者句柄
        rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr sub_waypoints_;

        // 辅助函数
        Eigen::Vector2d get_pt(int idx) const;
    };

} // namespace distribute_control