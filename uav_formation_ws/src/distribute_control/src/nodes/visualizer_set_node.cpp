#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <cmath>
#include <vector>
#include <mutex>

// 引入曲线库
#include "distribute_control/curves/ellipse_curve.hpp"
#include "distribute_control/curves/eight_curve.hpp"
#include "distribute_control/curves/squircle_curve.hpp"
#include "distribute_control/curves/bspline_curve.hpp"
#include "distribute_control/core/topic_names.hpp"
#include "distribute_control/core/world_transform.hpp"

using namespace std::chrono_literals;

static constexpr double EARTH_RADIUS = 6378137.0;
// [交互设置] 闭合判定距离 (米)
static constexpr double CLOSE_LOOP_THRESHOLD = 3.0;

class VisualizerSetNode : public rclcpp::Node
{
public:
    VisualizerSetNode() : Node("visualizer_set_node")
    {
        // ===========================
        // 1. 参数加载 (适配新 YAML 结构)
        // ===========================
        this->declare_parameter("system.uav_id", 1);
        this->declare_parameter("system.viz_rate", 1.0);
        this->declare_parameter("system.location_mode", "indoor");
        this->declare_parameter("system.is_sim", true);
        this->declare_parameter("world.origin_x", 0.0);
        this->declare_parameter("world.origin_y", 0.0);
        this->declare_parameter("world.origin_z", 0.0);
        this->declare_parameter("world.origin_yaw_rad", 0.0);

        this->declare_parameter("mission.takeoff_height", 1.0);
        this->declare_parameter("mission.curve_id", 0);
        this->declare_parameter("mission.curve_params", std::vector<double>{5.0, 5.0, 0.0});

        this->declare_parameter("offset_x", 0.0);
        this->declare_parameter("offset_y", 0.0);

        // 获取参数
        uav_id_ = this->get_parameter("system.uav_id").as_int();
        mode_ = this->get_parameter("system.location_mode").as_string();
        is_sim_ = this->get_parameter("system.is_sim").as_bool();

        takeoff_height_ = this->get_parameter("mission.takeoff_height").as_double();

        offset_x_ = this->get_parameter("offset_x").as_double();
        offset_y_ = this->get_parameter("offset_y").as_double();
        world_transform_ = distribute_control::WorldTransform(
            Eigen::Vector3d(this->get_parameter("world.origin_x").as_double(),
                            this->get_parameter("world.origin_y").as_double(),
                            this->get_parameter("world.origin_z").as_double()),
            this->get_parameter("world.origin_yaw_rad").as_double());

        RCLCPP_INFO(this->get_logger(), "Visualizer UAV %d Started. Mode: %s", uav_id_, mode_.c_str());

        // ===========================
        // 2. 通信初始化
        // ===========================
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        const std::string topic_prefix = distribute_control::dronePrefix(uav_id_);

        sub_local_pos_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            topic_prefix + "/fmu/out/vehicle_local_position", qos,
            std::bind(&VisualizerSetNode::local_pos_cb, this, std::placeholders::_1));

        if (mode_ == "outdoor")
        {
            sub_global_pos_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
                topic_prefix + "/fmu/out/vehicle_global_position", qos,
                std::bind(&VisualizerSetNode::global_pos_cb, this, std::placeholders::_1));
            sub_origin_gps_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
                distribute_control::dronePrefix(1) + "/fmu/out/vehicle_global_position", qos,
                std::bind(&VisualizerSetNode::origin_gps_cb, this, std::placeholders::_1));
        }

        // [交互逻辑] 仅在 UAV1 启用
        if (uav_id_ == 1)
        {
            // (A) 订阅 "Publish Point" 工具 (无快捷键，点击工具栏)
            sub_clicked_point_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
                "/clicked_point", 10,
                std::bind(&VisualizerSetNode::clicked_point_cb, this, std::placeholders::_1));

            // (B) [新增] 订阅 "2D Goal Pose" 工具 (快捷键 'g')
            // 这样你按 g 也可以生成点了！
            sub_goal_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/goal_pose", 10,
                std::bind(&VisualizerSetNode::goal_pose_cb, this, std::placeholders::_1));

            pub_waypoints_list_ = this->create_publisher<geometry_msgs::msg::Polygon>(
                "/global/curve_points", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

            pub_click_markers_ = this->create_publisher<visualization_msgs::msg::Marker>("/global_viz/click_markers", 10);

            RCLCPP_INFO(this->get_logger(), "\033[1;32m[Interactive Mode] READY!\033[0m");
            RCLCPP_INFO(this->get_logger(), "  Option 1: Click 'Publish Point' in toolbar.");
            RCLCPP_INFO(this->get_logger(), "  Option 2: Press 'g' on keyboard and click.");
            RCLCPP_INFO(this->get_logger(), "  Action: Click near START point to CLOSE LOOP.");
        }

        // 常规发布
        pub_drone_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_prefix + "/viz/drone_model", 10);
        pub_path_ = this->create_publisher<nav_msgs::msg::Path>(topic_prefix + "/viz/actual_path", 10);
        pub_velocity_ = this->create_publisher<visualization_msgs::msg::Marker>(topic_prefix + "/viz/velocity_arrow", 10);
        pub_global_path_ = this->create_publisher<nav_msgs::msg::Path>("/global_viz/desired_path", 10);
        pub_map_boundary_ = this->create_publisher<visualization_msgs::msg::Marker>("/global_viz/map_boundary", 10);

        // 定时器
        double viz_rate = this->get_parameter("system.viz_rate").as_double();
        auto period = std::chrono::duration<double>(1.0 / viz_rate);
        timer_viz_ = this->create_wall_timer(period, std::bind(&VisualizerSetNode::timer_callback, this));
    }

private:
    int uav_id_;
    double takeoff_height_;
    double offset_x_, offset_y_;
    std::string mode_;
    bool is_sim_;
    distribute_control::WorldTransform world_transform_;
    bool origin_set_ = false;
    double origin_lat_ = 0.0, origin_lon_ = 0.0;
    double cur_x_ = 0.0, cur_y_ = 0.0, cur_z_ = 0.0;

    std::shared_ptr<distribute_control::BsplineCurve> bspline_curve_;

    // [修复1] 默认为 false，启动时不画任何线
    bool curve_generated_ = false;

    std::vector<Eigen::Vector2d> temp_points_;
    std::mutex mutex_;

    // ROS 句柄
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_pos_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr sub_global_pos_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr sub_origin_gps_;

    // 交互订阅
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_clicked_point_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pose_; // [新增]

    rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr pub_waypoints_list_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_click_markers_;

    // 可视化发布
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_drone_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_velocity_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_global_path_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_map_boundary_;

    rclcpp::TimerBase::SharedPtr timer_viz_;
    nav_msgs::msg::Path path_msg_;

    // ===========================
    // [核心] 统一输入处理函数
    // ===========================
    void handle_input_point(double x, double y)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        Eigen::Vector2d new_pt(x, y);

        // 1. 检查是否闭合回路 (距离起点很近，且已有至少3个点)
        if (temp_points_.size() >= 3)
        {
            double dist = (new_pt - temp_points_[0]).norm();
            if (dist < CLOSE_LOOP_THRESHOLD)
            {
                RCLCPP_INFO(this->get_logger(), "\033[1;32mLoop Closed! Generating B-Spline...\033[0m");

                // A. 拟合曲线
                bspline_curve_->fit_waypoints_and_update(temp_points_);
                curve_generated_ = true; // [关键] 只有这里设为 true 后，Timer 才会开始画红线

                // B. 广播给 OffboardNode
                geometry_msgs::msg::Polygon poly_msg;
                for (const auto &pt : temp_points_)
                {
                    geometry_msgs::msg::Point32 p;
                    p.x = pt.x();
                    p.y = pt.y();
                    p.z = 0.0;
                    poly_msg.points.push_back(p);
                }
                pub_waypoints_list_->publish(poly_msg);

                // C. 立即刷新一次显示
                publish_desired_curve();

                // D. 清理交互状态，允许重新绘制
                temp_points_.clear();
                publish_click_markers();
                return;
            }
        }

        // 2. 如果没闭合，加入暂存点
        temp_points_.push_back(new_pt);
        RCLCPP_INFO(this->get_logger(), "Waypoint %d Added: [%.2f, %.2f]", (int)temp_points_.size(), x, y);

        // 3. 刷新黄点显示
        publish_click_markers();
    }

    // 回调 1: /clicked_point (Publish Point 工具)
    void clicked_point_cb(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        handle_input_point(msg->point.x, msg->point.y);
    }

    // 回调 2: /goal_pose (按 'g' 键的工具)
    void goal_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        handle_input_point(msg->pose.position.x, msg->pose.position.y);
    }

    // ===========================
    // 可视化逻辑
    // ===========================
    void publish_click_markers()
    {
        visualization_msgs::msg::Marker points_marker;
        points_marker.header.frame_id = "map";
        points_marker.header.stamp = this->now();
        points_marker.ns = "clicks";
        points_marker.id = 0;
        points_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;

        if (temp_points_.empty())
        {
            points_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        }
        else
        {
            points_marker.action = visualization_msgs::msg::Marker::ADD;
        }

        points_marker.scale.x = 0.5;
        points_marker.scale.y = 0.5;
        points_marker.scale.z = 0.5;
        points_marker.color.r = 1.0;
        points_marker.color.g = 1.0;
        points_marker.color.b = 0.0;
        points_marker.color.a = 1.0;

        for (const auto &pt : temp_points_)
        {
            geometry_msgs::msg::Point p;
            p.x = pt.x();
            p.y = pt.y();
            p.z = 0.0;
            points_marker.points.push_back(p);
        }
        pub_click_markers_->publish(points_marker);
    }

    void timer_callback()
    {
        if (uav_id_ == 1)
        {
            publish_desired_curve();
            publish_map_boundary();
        }
    }

    void publish_desired_curve()
    {
        // [修复1] 初始状态直接返回，不画任何东西
        if (!curve_generated_)
            return;

        nav_msgs::msg::Path desired_path;
        desired_path.header.frame_id = "map";
        desired_path.header.stamp = this->now();

        int samples = 300;
        for (int i = 0; i <= samples; i++)
        {
            double w = 2.0 * M_PI * i / samples;
            Eigen::Vector3d pos = bspline_curve_->get_position(w, takeoff_height_);
            geometry_msgs::msg::PoseStamped p;
            p.pose.position.x = pos.x();
            p.pose.position.y = pos.y();
            p.pose.position.z = pos.z();
            desired_path.poses.push_back(p);
        }
        pub_global_path_->publish(desired_path);
    }

    // ... (以下辅助函数保持不变) ...
    void publish_map_boundary()
    {
        if (mode_ == "outdoor")
            return;
        visualization_msgs::msg::Marker wall;
        wall.header.frame_id = "map";
        wall.header.stamp = this->now();
        wall.ns = "map_boundary";
        wall.id = 0;
        wall.type = visualization_msgs::msg::Marker::LINE_LIST;
        wall.action = visualization_msgs::msg::Marker::ADD;
        wall.scale.x = 0.05;
        wall.color.r = 0;
        wall.color.g = 0;
        wall.color.b = 0;
        wall.color.a = 0.6;
        double xl = 20, yl = 10, h = 5, xh = xl / 2, yh = yl / 2;
        for (double z = 0; z <= h + 0.01; z += 1)
        {
            geometry_msgs::msg::Point p1, p2, p3, p4;
            p1.x = xh;
            p1.y = yh;
            p1.z = z;
            p2.x = -xh;
            p2.y = yh;
            p2.z = z;
            p3.x = -xh;
            p3.y = -yh;
            p3.z = z;
            p4.x = xh;
            p4.y = -yh;
            p4.z = z;
            wall.points.push_back(p1);
            wall.points.push_back(p2);
            wall.points.push_back(p2);
            wall.points.push_back(p3);
            wall.points.push_back(p3);
            wall.points.push_back(p4);
            wall.points.push_back(p4);
            wall.points.push_back(p1);
        }
        std::vector<std::pair<double, double>> cols = {{xh, yh}, {-xh, yh}, {-xh, -yh}, {xh, -yh}};
        for (double x = -xh + 5; x < xh; x += 5)
        {
            cols.push_back({x, yh});
            cols.push_back({x, -yh});
        }
        for (auto &c : cols)
        {
            geometry_msgs::msg::Point s, e;
            s.x = c.first;
            s.y = c.second;
            s.z = 0;
            e.x = c.first;
            e.y = c.second;
            e.z = h;
            wall.points.push_back(s);
            wall.points.push_back(e);
        }
        pub_map_boundary_->publish(wall);
    }
    void origin_gps_cb(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
    {
        if (!origin_set_)
        {
            origin_lat_ = msg->lat;
            origin_lon_ = msg->lon;
            origin_set_ = true;
        }
    }
    void global_pos_cb(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
    {
        if (!origin_set_)
            return;
        double d_lat = msg->lat - origin_lat_, d_lon = msg->lon - origin_lon_;
        double rl = EARTH_RADIUS * M_PI / 180.0, rlon = EARTH_RADIUS * cos(origin_lat_ * M_PI / 180.0) * M_PI / 180.0;
        cur_x_ = d_lon * rlon;
        cur_y_ = d_lat * rl;
        update_visualization(0, 0, 0);
    }
    void local_pos_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        const Eigen::Vector3d local_position = world_transform_.positionFromPx4Ned(
            Eigen::Vector3d(msg->x, msg->y, msg->z));
        const Eigen::Vector3d local_velocity = world_transform_.velocityFromPx4Ned(
            Eigen::Vector3d(msg->vx, msg->vy, msg->vz));
        if (mode_ == "indoor")
        {
            cur_x_ = local_position.x();
            cur_y_ = local_position.y();
            if (is_sim_)
            {
                cur_x_ += offset_x_;
                cur_y_ += offset_y_;
            }
        }
        cur_z_ = local_position.z();
        update_visualization(local_velocity.x(), local_velocity.y(), local_velocity.z());
    }
    void update_visualization(double vx, double vy, double vz)
    {
        publish_drone_model(cur_x_, cur_y_, cur_z_);
        publish_actual_path(cur_x_, cur_y_, cur_z_);
        publish_velocity_arrow(cur_x_, cur_y_, cur_z_, vx, vy, vz);
    }
    void publish_velocity_arrow(double x, double y, double z, double vx, double vy, double vz)
    {
        visualization_msgs::msg::Marker a;
        a.header.frame_id = "map";
        a.header.stamp = this->now();
        a.ns = "v_" + std::to_string(uav_id_);
        a.id = 0;
        a.type = 0;
        a.action = 0;
        geometry_msgs::msg::Point s, e;
        s.x = x;
        s.y = y;
        s.z = z;
        e.x = x + vx;
        e.y = y + vy;
        e.z = z + vz;
        a.points.push_back(s);
        a.points.push_back(e);
        a.scale.x = 0.05;
        a.scale.y = 0.1;
        a.scale.z = 0.1;
        a.color.r = 0.5;
        a.color.b = 0.5;
        a.color.a = 1;
        pub_velocity_->publish(a);
    }
    void publish_actual_path(double x, double y, double z)
    {
        path_msg_.header.frame_id = "map";
        path_msg_.header.stamp = this->now();
        geometry_msgs::msg::PoseStamped p;
        p.header = path_msg_.header;
        p.pose.position.x = x;
        p.pose.position.y = y;
        p.pose.position.z = z;
        path_msg_.poses.push_back(p);
        if (path_msg_.poses.size() > 2000)
            path_msg_.poses.erase(path_msg_.poses.begin());
        pub_path_->publish(path_msg_);
    }
    void publish_drone_model(double x, double y, double z)
    {
        visualization_msgs::msg::MarkerArray markers;
        auto now = this->now();
        float arm_len = 0.25;
        float prop_r = 0.1;
        float r = 0.0, g = 0.0, b = 0.0;
        if (uav_id_ == 1)
        {
            r = 1.0;
            g = 0.2;
            b = 0.2;
        }
        else if (uav_id_ == 2)
        {
            r = 0.2;
            g = 1.0;
            b = 0.2;
        }
        else if (uav_id_ == 3)
        {
            r = 0.2;
            g = 0.2;
            b = 1.0;
        }
        else
        {
            r = 1.0;
            g = 1.0;
            b = 0.0;
        }

        visualization_msgs::msg::Marker body;
        body.header.frame_id = "map";
        body.header.stamp = now;
        body.ns = "uav_" + std::to_string(uav_id_);
        body.id = 0;
        body.type = visualization_msgs::msg::Marker::CUBE;
        body.action = visualization_msgs::msg::Marker::ADD;
        body.pose.position.x = x;
        body.pose.position.y = y;
        body.pose.position.z = z;
        body.pose.orientation.w = 1.0;
        body.scale.x = 0.1;
        body.scale.y = 0.1;
        body.scale.z = 0.05;
        body.color.r = r;
        body.color.g = g;
        body.color.b = b;
        body.color.a = 1.0;
        markers.markers.push_back(body);

        visualization_msgs::msg::Marker arm1 = body;
        arm1.id = 1;
        arm1.type = visualization_msgs::msg::Marker::CUBE;
        arm1.scale.x = arm_len * 2;
        arm1.scale.y = 0.02;
        arm1.scale.z = 0.02;
        arm1.color.r = 0.8;
        arm1.color.g = 0.8;
        arm1.color.b = 0.8;
        markers.markers.push_back(arm1);

        visualization_msgs::msg::Marker arm2 = arm1;
        arm2.id = 2;
        arm2.scale.x = 0.02;
        arm2.scale.y = arm_len * 2;
        markers.markers.push_back(arm2);

        for (int i = 0; i < 4; i++)
        {
            visualization_msgs::msg::Marker prop = body;
            prop.id = 3 + i;
            prop.type = visualization_msgs::msg::Marker::CYLINDER;
            prop.scale.x = prop_r * 2;
            prop.scale.y = prop_r * 2;
            prop.scale.z = 0.01;
            prop.color.r = 0.1;
            prop.color.g = 0.1;
            prop.color.b = 0.1;
            prop.color.a = 0.8;

            double dx = (i == 0) ? arm_len : (i == 1) ? -arm_len
                                                      : 0;
            double dy = (i == 2) ? arm_len : (i == 3) ? -arm_len
                                                      : 0;
            prop.pose.position.x += dx;
            prop.pose.position.y += dy;
            prop.pose.position.z += 0.02;
            markers.markers.push_back(prop);
        }
        pub_drone_->publish(markers);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualizerSetNode>());
    rclcpp::shutdown();
    return 0;
}
