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
#include <iomanip> // 用于格式化距离文字

#include "distribute_control/curves/ellipse_curve.hpp"
#include "distribute_control/curves/eight_curve.hpp"
#include "distribute_control/curves/squircle_curve.hpp"
#include "distribute_control/curves/bspline_curve.hpp"
#include "distribute_control/core/topic_names.hpp"
#include "distribute_control/core/world_transform.hpp"
#include "distribute_control/curves/eight_curve_2.hpp"
using namespace std::chrono_literals;

static constexpr double EARTH_RADIUS = 6378137.0;

enum class CurveID
{
    ELLIPSE = 0,
    EIGHT = 1,
    SQUIRCLE = 2,
    BSPLINE = 3
};

class VisualizerNode : public rclcpp::Node
{
public:
    VisualizerNode() : Node("visualizer_node")
    {
        // ===========================
        // 1. 参数加载
        // ===========================
        this->declare_parameter("system.uav_id", 1);
        this->declare_parameter("system.total_uavs", 3);  // [关键] 必须知道总共有多少架
        this->declare_parameter("system.viz_rate", 20.0); // 提高刷新率以便距离显示流畅
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
        this->declare_parameter("orca.safe_dist", 1.0);

        uav_id_ = this->get_parameter("system.uav_id").as_int();
        total_uavs_ = this->get_parameter("system.total_uavs").as_int();
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
        safe_dist_ = this->get_parameter("orca.safe_dist").as_double();

        // [逻辑修正] 计算邻居 ID (1->2, 2->3, ..., n->1)
        if (uav_id_ < total_uavs_)
        {
            neighbor_id_ = uav_id_ + 1;
        }
        else
        {
            neighbor_id_ = 1; // 闭环
        }

        RCLCPP_INFO(this->get_logger(), "Visualizer UAV %d Started. Offset: (%.1f, %.1f). Neighbor: UAV %d",
                    uav_id_, offset_x_, offset_y_, neighbor_id_);

        // ===========================
        // 2. 通信初始化
        // ===========================
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        const std::string topic_prefix = distribute_control::dronePrefix(uav_id_);

        // 2.1 订阅自己的 Local Pos
        sub_local_pos_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            topic_prefix + "/fmu/out/vehicle_local_position", qos,
            std::bind(&VisualizerNode::local_pos_cb, this, std::placeholders::_1));

        // [关键修正] 订阅邻居的 Viz Marker，而不是 Local Pos
        // 这样可以自动获取邻居已经加上 Offset 后的正确世界坐标
        const std::string neighbor_viz_topic = distribute_control::dronePrefix(neighbor_id_) + "/viz/drone_model";
        sub_neighbor_viz_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            neighbor_viz_topic, 10,
            std::bind(&VisualizerNode::neighbor_viz_cb, this, std::placeholders::_1));

        // 2.2 发布器
        pub_heading_ = this->create_publisher<visualization_msgs::msg::Marker>(topic_prefix + "/viz/heading_arrow", 10);
        pub_safe_radius_ = this->create_publisher<visualization_msgs::msg::Marker>(topic_prefix + "/viz/safe_radius", 10);

        // [新增] 距离连线和距离文字
        pub_link_line_ = this->create_publisher<visualization_msgs::msg::Marker>(topic_prefix + "/viz/link_line", 10);
        pub_dist_text_ = this->create_publisher<visualization_msgs::msg::Marker>(topic_prefix + "/viz/dist_text", 10);

        if (mode_ == "outdoor")
        {
            sub_global_pos_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
                topic_prefix + "/fmu/out/vehicle_global_position", qos,
                std::bind(&VisualizerNode::global_pos_cb, this, std::placeholders::_1));

            sub_origin_gps_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
                distribute_control::dronePrefix(1) + "/fmu/out/vehicle_global_position", qos,
                std::bind(&VisualizerNode::origin_gps_cb, this, std::placeholders::_1));
        }

        pub_drone_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_prefix + "/viz/drone_model", 10);
        pub_path_ = this->create_publisher<nav_msgs::msg::Path>(topic_prefix + "/viz/actual_path", 10);
        pub_velocity_ = this->create_publisher<visualization_msgs::msg::Marker>(topic_prefix + "/viz/velocity_arrow", 10);

        if (uav_id_ == 1)
        {
            pub_global_path_ = this->create_publisher<nav_msgs::msg::Path>("/global_viz/desired_path", 10);
            pub_map_boundary_ = this->create_publisher<visualization_msgs::msg::Marker>("/global_viz/map_boundary", 10);
        }

        // 定时器用于一些低频的全局绘制，主要的绘制在 local_pos_cb 中触发
        double viz_rate = this->get_parameter("system.viz_rate").as_double();
        timer_viz_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / viz_rate),
                                             std::bind(&VisualizerNode::timer_callback, this));
    }

private:
    int uav_id_;
    int total_uavs_;
    int neighbor_id_;

    // [新增] 邻居数据
    bool has_neighbor_ = false;
    geometry_msgs::msg::Point neighbor_pos_; // 邻居在 Map 系下的坐标

    double takeoff_height_;
    double offset_x_, offset_y_;
    std::string mode_;
    bool is_sim_;
    distribute_control::WorldTransform world_transform_;
    double cur_yaw_ = 0.0;
    double safe_dist_ = 1.0;

    bool origin_set_ = false;
    double origin_lat_ = 0.0, origin_lon_ = 0.0;
    double cur_x_ = 0.0, cur_y_ = 0.0, cur_z_ = 0.0;

    // Publishers & Subscribers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_heading_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_safe_radius_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_drone_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_velocity_;

    // [新增] 距离相关
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_link_line_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_dist_text_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_neighbor_viz_;

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_pos_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr sub_global_pos_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr sub_origin_gps_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_global_path_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_map_boundary_;

    rclcpp::TimerBase::SharedPtr timer_viz_;
    nav_msgs::msg::Path path_msg_;

    // ===========================
    // 定时器回调
    // ===========================
    void timer_callback()
    {
        if (uav_id_ == 1)
        {
            publish_desired_curve();
            publish_map_boundary();
        }
    }

    // ===========================
    // [核心修正] 邻居 Viz 回调
    // ===========================
    void neighbor_viz_cb(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        // 邻居发布的 MarkerArray 中，Marker[0] 通常是机身中心
        if (!msg->markers.empty())
        {
            const auto &body_marker = msg->markers[0];
            // 获取邻居在 Map 系下的位置（邻居节点已经加上了它的 Offset）
            neighbor_pos_ = body_marker.pose.position;
            has_neighbor_ = true;
        }
    }

    void local_pos_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        const Eigen::Vector3d local_position = world_transform_.positionFromPx4Ned(
            Eigen::Vector3d(msg->x, msg->y, msg->z));

        // 计算当前无人机在 Map 系下的坐标
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
        // 室外模式 GPS 回调会更新 cur_x/y，这里只更新 z
        cur_z_ = local_position.z();

        double heading_ned = msg->heading;
        if (!std::isfinite(heading_ned))
            heading_ned = 0.0;
        cur_yaw_ = -heading_ned + M_PI_2;

        double vx = msg->vy;
        double vy = msg->vx;
        double vz = -msg->vz;

        // 更新所有可视化
        update_visualization(vx, vy, vz);
    }

    void update_visualization(double vx, double vy, double vz)
    {
        // 1. 基础模型
        publish_drone_model(cur_x_, cur_y_, cur_z_, cur_yaw_);
        publish_actual_path(cur_x_, cur_y_, cur_z_);
        publish_velocity_arrow(cur_x_, cur_y_, cur_z_, vx, vy, vz);
        publish_heading_arrow(cur_x_, cur_y_, cur_z_, cur_yaw_);
        publish_safe_radius(cur_x_, cur_y_, cur_z_);

        // 2. [新增] 邻居连线与距离
        if (has_neighbor_)
        {
            publish_neighbor_link_and_dist();
        }
    }

    // ===========================
    // [核心功能] 绘制虚线和距离文字
    // ===========================
    void publish_neighbor_link_and_dist()
    {
        // 计算两点之间的向量
        double dx = neighbor_pos_.x - cur_x_;
        double dy = neighbor_pos_.y - cur_y_;
        double dz = neighbor_pos_.z - cur_z_;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        // --- A. 绘制虚线 ---
        visualization_msgs::msg::Marker line;
        line.header.frame_id = "map";
        line.header.stamp = this->now();
        line.ns = "conn_line_" + std::to_string(uav_id_);
        line.id = 0;
        line.type = visualization_msgs::msg::Marker::LINE_LIST;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.03; // 线条粗细
        // 亮青色
        line.color.r = 0.0;
        line.color.g = 1.0;
        line.color.b = 1.0;
        line.color.a = 0.8;

        // 手动切分虚线: 0.2m 实线, 0.2m 间隔
        double dash_len = 0.2;
        double gap_len = 0.2;
        double current_len = 0.0;

        // 单位向量
        double ux = dx / dist;
        double uy = dy / dist;
        double uz = dz / dist;

        while (current_len < dist)
        {
            geometry_msgs::msg::Point p1, p2;
            p1.x = cur_x_ + ux * current_len;
            p1.y = cur_y_ + uy * current_len;
            p1.z = cur_z_ + uz * current_len;

            double next_len = std::min(current_len + dash_len, dist);
            p2.x = cur_x_ + ux * next_len;
            p2.y = cur_y_ + uy * next_len;
            p2.z = cur_z_ + uz * next_len;

            line.points.push_back(p1);
            line.points.push_back(p2);

            current_len += dash_len + gap_len;
        }
        pub_link_line_->publish(line);

        // --- B. 绘制距离数值 ---
        visualization_msgs::msg::Marker text;
        text.header.frame_id = "map";
        text.header.stamp = this->now();
        text.ns = "dist_text_" + std::to_string(uav_id_);
        text.id = 0;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;

        // 放在连线中点
        text.pose.position.x = cur_x_ + dx * 0.5;
        text.pose.position.y = cur_y_ + dy * 0.5;
        // 稍微往上抬一点，防止和线重合
        text.pose.position.z = cur_z_ + dz * 0.5 + 0.3;

        text.scale.z = 0.35; // 字体大小
        // 鲜艳的黄色
        text.color.r = 0.0;
        text.color.g = 1.0;
        text.color.b = 0.0;
        text.color.a = 1.0;

        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << dist << "m";
        text.text = ss.str();

        pub_dist_text_->publish(text);
    }

    // ===========================
    // 其他绘图函数 (保持不变)
    // ===========================
    void publish_safe_radius(double x, double y, double z)
    {
        visualization_msgs::msg::Marker disc;
        disc.header.frame_id = "map";
        disc.header.stamp = this->now();
        disc.ns = "safe_radius_" + std::to_string(uav_id_);
        disc.id = 0;
        disc.type = visualization_msgs::msg::Marker::CYLINDER;
        disc.action = visualization_msgs::msg::Marker::ADD;
        disc.pose.position.x = x;
        disc.pose.position.y = y;
        disc.pose.position.z = z;
        disc.pose.orientation.w = 1.0;
        disc.scale.x = safe_dist_ * 2.0;
        disc.scale.y = safe_dist_ * 2.0;
        disc.scale.z = 0.02;
        disc.color.r = 1.0;
        disc.color.g = 0.2;
        disc.color.b = 0.2;
        disc.color.a = 0.15;
        pub_safe_radius_->publish(disc);
    }

    void publish_desired_curve()
    {
        int curve_id_int = this->get_parameter("mission.curve_id").as_int();
        std::vector<double> c_params = this->get_parameter("mission.curve_params").as_double_array();
        std::shared_ptr<distribute_control::CurveBase> curve;
        switch (static_cast<CurveID>(curve_id_int))
        {
        case CurveID::ELLIPSE:
            curve = std::make_shared<distribute_control::EllipseCurve>();
            break;
        case CurveID::EIGHT:
            curve = std::make_shared<distribute_control::EightCurve>();
            break;
        case CurveID::SQUIRCLE:
            curve = std::make_shared<distribute_control::SquircleCurve>();
            break;
        case CurveID::BSPLINE:
        {
            auto b = std::make_shared<distribute_control::BsplineCurve>();
            curve = b;
        }
        break;
        default:
            curve = std::make_shared<distribute_control::EllipseCurve>();
            break;
        }
        curve->load_params(c_params);
        nav_msgs::msg::Path desired_path;
        desired_path.header.frame_id = "map";
        desired_path.header.stamp = this->now();
        for (int i = 0; i <= 200; i++)
        {
            double w = 2.0 * M_PI * i / 200;
            Eigen::Vector3d pos = curve->get_position(w, takeoff_height_);
            geometry_msgs::msg::PoseStamped p;
            p.pose.position.x = pos.x();
            p.pose.position.y = pos.y();
            p.pose.position.z = pos.z();
            desired_path.poses.push_back(p);
        }
        pub_global_path_->publish(desired_path);
    }

    void publish_heading_arrow(double x, double y, double z, double yaw)
    {
        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = "map";
        arrow.header.stamp = this->now();
        arrow.ns = "heading_" + std::to_string(uav_id_);
        arrow.id = 0;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        geometry_msgs::msg::Point start, end;
        start.x = x;
        start.y = y;
        start.z = z;
        end.x = x + 0.6 * cos(yaw);
        end.y = y + 0.6 * sin(yaw);
        end.z = z;
        arrow.points.push_back(start);
        arrow.points.push_back(end);
        arrow.scale.x = 0.03;
        arrow.scale.y = 0.06;
        arrow.scale.z = 0.06;
        arrow.color.r = 1.0;
        arrow.color.g = 0.8;
        arrow.color.b = 0.0;
        arrow.color.a = 1.0;
        pub_heading_->publish(arrow);
    }

    void publish_velocity_arrow(double x, double y, double z, double vx, double vy, double vz)
    {
        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = "map";
        arrow.header.stamp = this->now();
        arrow.ns = "velocity_" + std::to_string(uav_id_);
        arrow.id = 0;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        geometry_msgs::msg::Point start, end;
        start.x = x;
        start.y = y;
        start.z = z;
        end.x = x + vx;
        end.y = y + vy;
        end.z = z + vz;
        arrow.points.push_back(start);
        arrow.points.push_back(end);
        arrow.scale.x = 0.05;
        arrow.scale.y = 0.1;
        arrow.scale.z = 0.1;
        arrow.color.r = 0.5;
        arrow.color.g = 0.0;
        arrow.color.b = 0.5;
        arrow.color.a = 1.0;
        pub_velocity_->publish(arrow);
    }

    void publish_actual_path(double x, double y, double z)
    {
        path_msg_.header.frame_id = "map";
        path_msg_.header.stamp = this->now();
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg_.header;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        path_msg_.poses.push_back(pose);
        if (path_msg_.poses.size() > 2000)
            path_msg_.poses.erase(path_msg_.poses.begin());
        pub_path_->publish(path_msg_);
    }

    void publish_drone_model(double x, double y, double z, double yaw)
    {
        visualization_msgs::msg::MarkerArray markers;
        auto now = this->now();
        double qw = cos(yaw / 2.0);
        double qz = sin(yaw / 2.0);
        float arm_len = 0.205;
        float prop_r = 0.12;
        float r = 0, g = 0, b = 0;
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
        body.pose.orientation.w = qw;
        body.pose.orientation.x = 0.0;
        body.pose.orientation.y = 0.0;
        body.pose.orientation.z = qz;
        body.scale.x = 0.1;
        body.scale.y = 0.1;
        body.scale.z = 0.05;
        body.color.r = r;
        body.color.g = g;
        body.color.b = b;
        body.color.a = 1.0;
        markers.markers.push_back(body);

        // ... (保持后面的机臂和螺旋桨绘制代码不变) ...
        visualization_msgs::msg::Marker arm1 = body;
        arm1.id = 1;
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
            double dx_l = (i == 0) ? arm_len : (i == 1) ? -arm_len
                                                        : 0;
            double dy_l = (i == 2) ? arm_len : (i == 3) ? -arm_len
                                                        : 0;
            double dx_world = dx_l * cos(yaw) - dy_l * sin(yaw);
            double dy_world = dx_l * sin(yaw) + dy_l * cos(yaw);
            prop.pose.position.x = x + dx_world;
            prop.pose.position.y = y + dy_world;
            prop.pose.position.z = z + 0.02;
            prop.pose.orientation = body.pose.orientation;
            markers.markers.push_back(prop);
        }
        pub_drone_->publish(markers);
    }

    // ===========================
    // 地图边界 & GPS 回调 (保持不变)
    // ===========================
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
        wall.color.r = 0.0;
        wall.color.g = 0.0;
        wall.color.b = 0.0;
        wall.color.a = 0.6;
        double x_len = 20.0, y_len = 10.0, height = 5.0;
        double x_half = x_len / 2.0, y_half = y_len / 2.0;
        for (double h = 0; h <= height + 0.01; h += 1.0)
        {
            geometry_msgs::msg::Point p1, p2, p3, p4;
            p1.x = x_half;
            p1.y = y_half;
            p1.z = h;
            p2.x = -x_half;
            p2.y = y_half;
            p2.z = h;
            p3.x = -x_half;
            p3.y = -y_half;
            p3.z = h;
            p4.x = x_half;
            p4.y = -y_half;
            p4.z = h;
            wall.points.push_back(p1);
            wall.points.push_back(p2);
            wall.points.push_back(p2);
            wall.points.push_back(p3);
            wall.points.push_back(p3);
            wall.points.push_back(p4);
            wall.points.push_back(p4);
            wall.points.push_back(p1);
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
        double d_lat = msg->lat - origin_lat_;
        double d_lon = msg->lon - origin_lon_;
        double r_lat = EARTH_RADIUS * M_PI / 180.0;
        double r_lon = EARTH_RADIUS * cos(origin_lat_ * M_PI / 180.0) * M_PI / 180.0;
        cur_x_ = d_lon * r_lon;
        cur_y_ = d_lat * r_lat;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualizerNode>());
    rclcpp::shutdown();
    return 0;
}
