#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <iostream>
#include <cmath>
#include <map>
#include <vector>

#include "distribute_control/core/controller.hpp"
#include "distribute_control/core/topology.hpp"
#include "distribute_control/curves/ellipse_curve.hpp"
#include "distribute_control/curves/eight_curve.hpp"
#include "distribute_control/curves/squircle_curve.hpp"
#include "distribute_control/curves/bspline_curve.hpp"
#include "distribute_control/core/orca_barrier.hpp"
#include "distribute_control/core/topic_names.hpp"
#include "distribute_control/core/world_transform.hpp"

using namespace std::chrono_literals;

static constexpr double EARTH_RADIUS = 6378137.0;

enum class CurveID
{
    ELLIPSE = 0,
    EIGHT = 1,
    SQUIRCLE = 2,
    BSPLINE = 3
};

class OffboardNode : public rclcpp::Node
{
public:
    OffboardNode() : Node("offboard_node")
    {
        // =======================================================
        // 1. 参数加载 (Group: System)
        // =======================================================
        this->declare_parameter("system.uav_id", 1);
        this->declare_parameter("system.total_uavs", 3);
        this->declare_parameter("system.loop_rate", 50.0);
        this->declare_parameter("system.location_mode", "indoor");
        this->declare_parameter("system.is_sim", true);
        this->declare_parameter("system.target_system", 0);
        this->declare_parameter("control.auto_arm", false);
        this->declare_parameter("control.odom_timeout", 0.5);
        this->declare_parameter("control.neighbor_timeout", 0.5);

        uav_id_ = this->get_parameter("system.uav_id").as_int();
        int total_uavs = this->get_parameter("system.total_uavs").as_int();
        double loop_rate = this->get_parameter("system.loop_rate").as_double();
        mode_ = this->get_parameter("system.location_mode").as_string();
        is_sim_ = this->get_parameter("system.is_sim").as_bool();
        const int configured_target_system = this->get_parameter("system.target_system").as_int();
        target_system_ = configured_target_system > 0 ? configured_target_system : uav_id_ + 1;
        auto_arm_ = this->get_parameter("control.auto_arm").as_bool();
        odom_timeout_ = this->get_parameter("control.odom_timeout").as_double();
        neighbor_timeout_ = this->get_parameter("control.neighbor_timeout").as_double();
        dt_ = 1.0 / loop_rate;

        // =======================================================
        // 2. 任务配置 (Group: Mission)
        // =======================================================
        this->declare_parameter("mission.takeoff_height", 1.0);
        this->declare_parameter("mission.formation_spacing", 2.0);
        this->declare_parameter("mission.curve_id", 0); // 默认为 0 (Ellipse)
        this->declare_parameter("mission.curve_params", std::vector<double>{5.0, 5.0, 0.0});
        this->declare_parameter("mission.yaw_mode", 1); // 0:Fixed, 1:Tangent

        takeoff_height_ = this->get_parameter("mission.takeoff_height").as_double();
        double formation_spacing = this->get_parameter("mission.formation_spacing").as_double();
        int curve_id_int = this->get_parameter("mission.curve_id").as_int();
        std::vector<double> c_params = this->get_parameter("mission.curve_params").as_double_array();

        // 外部传入的偏移量 (Launch文件覆盖)
        this->declare_parameter("offset_x", 0.0);
        this->declare_parameter("offset_y", 0.0);
        offset_x_ = this->get_parameter("offset_x").as_double();
        offset_y_ = this->get_parameter("offset_y").as_double();
        this->declare_parameter("world.origin_x", 0.0);
        this->declare_parameter("world.origin_y", 0.0);
        this->declare_parameter("world.origin_z", 0.0);
        this->declare_parameter("world.origin_yaw_rad", 0.0);
        world_transform_ = distribute_control::WorldTransform(
            Eigen::Vector3d(this->get_parameter("world.origin_x").as_double(),
                            this->get_parameter("world.origin_y").as_double(),
                            this->get_parameter("world.origin_z").as_double()),
            this->get_parameter("world.origin_yaw_rad").as_double());

        RCLCPP_INFO(this->get_logger(), ">>> UAV %d Init. Mode: %s, CurveID: %d, Spacing: %.1f",
                    uav_id_, mode_.c_str(), curve_id_int, formation_spacing);

        // =======================================================
        // 3. 控制器增益 (Group: Gains)
        // =======================================================
        this->declare_parameter("gains.k_pos", 5.0);
        this->declare_parameter("gains.k_c", 5.0);
        this->declare_parameter("gains.look_ahead", 0.2);
        this->declare_parameter("gains.max_speed", 1.0);

        distribute_control::ControlParams cp;
        cp.k_pos = this->get_parameter("gains.k_pos").as_double();
        cp.k_c = this->get_parameter("gains.k_c").as_double();
        cp.look_ahead = this->get_parameter("gains.look_ahead").as_double();
        cp.max_speed = this->get_parameter("gains.max_speed").as_double();

        // =======================================================
        // 4. 避障设置 (Group: ORCA)
        // =======================================================
        this->declare_parameter("orca.enabled", true);
        this->declare_parameter("orca.safe_dist", 0.5);
        this->declare_parameter("orca.time_horizon", 2.0);
        this->declare_parameter("orca.max_speed", 2.0);

        orca_enabled_ = this->get_parameter("orca.enabled").as_bool();

        // 初始化 ORCA
        orca_barrier_ = std::make_unique<distribute_control::ORCABarrier>();
        distribute_control::ORCAParams orca_p;
        orca_p.safe_dist = this->get_parameter("orca.safe_dist").as_double();
        orca_p.time_horizon = this->get_parameter("orca.time_horizon").as_double();
        orca_p.max_speed = this->get_parameter("orca.max_speed").as_double();
        orca_barrier_->set_params(orca_p);

        // =======================================================
        // 5. 核心模块初始化 (Curve Selection via Switch)
        // =======================================================
        topology_ = std::make_unique<distribute_control::Topology>(total_uavs, uav_id_);

        std::shared_ptr<distribute_control::CurveBase> curve;

        // [优化] 使用 switch-case 替代 string 比较，更规范
        switch (static_cast<CurveID>(curve_id_int))
        {
        case CurveID::ELLIPSE:
            curve = std::make_shared<distribute_control::EllipseCurve>();
            RCLCPP_INFO(this->get_logger(), "Curve: Ellipse Selected");
            break;
        case CurveID::EIGHT:
            curve = std::make_shared<distribute_control::EightCurve>();
            RCLCPP_INFO(this->get_logger(), "Curve: Eight Selected");
            break;
        case CurveID::SQUIRCLE:
            curve = std::make_shared<distribute_control::SquircleCurve>();
            RCLCPP_INFO(this->get_logger(), "Curve: Squircle Selected");
            break;
        case CurveID::BSPLINE:
        {
            auto bspline = std::make_shared<distribute_control::BsplineCurve>();
            bspline->start_subscription(this);
            curve = bspline;
            RCLCPP_INFO(this->get_logger(), "Curve: B-Spline Selected");
        }
        break;
        default:
            curve = std::make_shared<distribute_control::EllipseCurve>();
            RCLCPP_WARN(this->get_logger(), "Unknown Curve ID %d! Defaulting to Ellipse.", curve_id_int);
            break;
        }

        curve->load_params(c_params);
        controller_ = std::make_unique<distribute_control::DistributeController>();
        controller_->set_curve(curve);
        controller_->update_params(cp);

        // =======================================================
        // 6. 通信订阅
        // =======================================================
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        const std::string ns = distribute_control::dronePrefix(uav_id_);

        sub_local_pos_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            ns + "/fmu/out/vehicle_local_position", qos,
            std::bind(&OffboardNode::local_pos_cb, this, std::placeholders::_1));

        // 邻居订阅
        for (int i = 1; i <= total_uavs; ++i)
        {
            if (i == uav_id_)
                continue;
            const std::string other_ns = distribute_control::dronePrefix(i);

            auto sub = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                other_ns + "/fmu/out/vehicle_local_position", qos,
                [this, i, total_uavs, formation_spacing](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
                {
                    const Eigen::Vector3d local_position = world_transform_.positionFromPx4Ned(
                        Eigen::Vector3d(msg->x, msg->y, msg->z));
                    const Eigen::Vector3d local_velocity = world_transform_.velocityFromPx4Ned(
                        Eigen::Vector3d(msg->vx, msg->vy, msg->vz));

                    // 计算偏移量 (使用参数化的 formation_spacing)
                    double n_offset_x = 0.0;
                    double n_offset_y = 0.0;

                    if (mode_ == "indoor" && is_sim_)
                    {
                        double spacing = formation_spacing;
                        int cols = std::ceil(std::sqrt(total_uavs));
                        int rows = std::ceil((double)total_uavs / cols);
                        double grid_width_x = (cols - 1) * spacing;
                        double grid_width_y = (rows - 1) * spacing;

                        int idx = i - 1;
                        int r_id = idx / cols;
                        int c_id = idx % cols;

                        n_offset_x = (c_id * spacing) - (grid_width_x / 2.0);
                        n_offset_y = (r_id * spacing) - (grid_width_y / 2.0);
                    }

                    neighbor_map_[i] = local_position + Eigen::Vector3d(n_offset_x, n_offset_y, 0.0);
                    neighbor_vel_map_[i] = local_velocity;
                    neighbor_last_seen_[i] = this->now();
                });
            neighbor_subs_.push_back(sub);
        }

        // Publisher 初始化
        pub_offboard_mode_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(ns + "/fmu/in/offboard_control_mode", qos);
        pub_traj_setpoint_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(ns + "/fmu/in/trajectory_setpoint", qos);
        pub_vehicle_cmd_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(ns + "/fmu/in/vehicle_command", qos);

        // Debug Publisher
        pub_debug_phi_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(ns + "/phi", qos);
        pub_w_self_ = this->create_publisher<std_msgs::msg::Float32>(ns + "/virtual_w", qos);

        // 虚拟量 w 通信
        int prev_id = topology_->get_prev_id();
        int next_id = topology_->get_next_id();
        sub_w_prev_ = this->create_subscription<std_msgs::msg::Float32>(
            distribute_control::dronePrefix(prev_id) + "/virtual_w", qos,
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            { w_prev_ = msg->data; });
        sub_w_next_ = this->create_subscription<std_msgs::msg::Float32>(
            distribute_control::dronePrefix(next_id) + "/virtual_w", qos,
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            { w_next_ = msg->data; });

        // 控制指令订阅
        sub_start_cmd_ = this->create_subscription<std_msgs::msg::Int32>(
            "/start_and_stop", qos, [this](const std_msgs::msg::Int32::SharedPtr msg)
            { flag_start_ = msg->data; });
        sub_algo_cmd_ = this->create_subscription<std_msgs::msg::Int32>(
            "/start_algorithm", qos, [this](const std_msgs::msg::Int32::SharedPtr msg)
            { flag_algo_ = msg->data; });
        sub_vehicle_status_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            ns + "/fmu/out/vehicle_status", qos,
            [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg)
            {
                vehicle_status_received_ = true;
                armed_ = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
                offboard_ready_ = msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
            });

        // 定时器
        timer_ = this->create_wall_timer(std::chrono::duration<double>(dt_), std::bind(&OffboardNode::timer_callback, this));
    }

private:
    int uav_id_;
    uint8_t target_system_ = 1;
    double dt_;                 // 保存时间步长
    bool orca_enabled_ = false; // 默认关闭
    std::string mode_;
    bool is_sim_;
    double takeoff_height_;
    double offset_x_, offset_y_;
    bool auto_arm_ = false;
    double odom_timeout_ = 0.5;
    double neighbor_timeout_ = 0.5;
    bool current_position_received_ = false;
    rclcpp::Time last_local_position_time_{0, 0, RCL_ROS_TIME};
    bool vehicle_status_received_ = false;
    bool armed_ = false;
    bool offboard_ready_ = false;
    bool offboard_requested_ = false;
    distribute_control::WorldTransform world_transform_;
    bool origin_set_ = false;
    double origin_lat_ = 0.0;
    double origin_lon_ = 0.0;

    // [Fix] 增加当前速度成员变量
    Eigen::Vector3d current_pos_{0, 0, 0};
    Eigen::Vector3d current_vel_{0, 0, 0};
    // [Fix] 增加邻居速度 Map
    std::map<int, Eigen::Vector3d> neighbor_map_;
    std::map<int, Eigen::Vector3d> neighbor_vel_map_;
    std::map<int, rclcpp::Time> neighbor_last_seen_;

    // 用于起飞防飘移
    bool home_pos_recorded_ = false;
    Eigen::Vector3d home_pos_{0, 0, 0};

    double w_self_ = 0.0;
    double w_prev_ = 0.0;
    double w_next_ = 0.0;
    int flag_start_ = 0;
    int flag_algo_ = 0;
    int offboard_setpoint_counter_ = 0;

    std::unique_ptr<distribute_control::Topology> topology_;
    std::unique_ptr<distribute_control::DistributeController> controller_;

    // [新增] ORCA 对象
    std::unique_ptr<distribute_control::ORCABarrier> orca_barrier_;

    std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr> neighbor_subs_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_pos_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr sub_global_pos_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr sub_origin_gps_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_vehicle_status_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offboard_mode_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_traj_setpoint_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_vehicle_cmd_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_w_prev_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_w_next_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_w_self_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_debug_phi_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_start_cmd_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_algo_cmd_;

    void origin_gps_cb(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
    {
        if (!origin_set_)
        {
            origin_lat_ = msg->lat;
            origin_lon_ = msg->lon;
            origin_set_ = true;
            RCLCPP_INFO(this->get_logger(), "GPS Origin Set to UAV1: [%.7f, %.7f]", origin_lat_, origin_lon_);
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
        current_pos_.x() = d_lon * r_lon; // East
        current_pos_.y() = d_lat * r_lat; // North
        // Log throttle...
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "UAV %d Global Pos: [%.2f, %.2f]", uav_id_, current_pos_.x(), current_pos_.y());
    }

    void local_pos_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        const Eigen::Vector3d local_position = world_transform_.positionFromPx4Ned(
            Eigen::Vector3d(msg->x, msg->y, msg->z));
        const Eigen::Vector3d local_velocity = world_transform_.velocityFromPx4Ned(
            Eigen::Vector3d(msg->vx, msg->vy, msg->vz));

        if (mode_ == "indoor")
        {
            current_pos_ = local_position;
            if (is_sim_)
                current_pos_ += Eigen::Vector3d(offset_x_, offset_y_, 0.0);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "UAV %d Local Pos: [%.2f, %.2f]", uav_id_, current_pos_.x(), current_pos_.y());
        }
        current_pos_.z() = local_position.z();
        current_vel_ = local_velocity;
        current_position_received_ = true;
        last_local_position_time_ = this->now();
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = target_system_;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        pub_vehicle_cmd_->publish(msg);
    }

    void publish_offboard_control_mode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = false;
        msg.velocity = true;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        pub_offboard_mode_->publish(msg);
    }

    void timer_callback()
    {
        if (flag_start_ == 1)
        {
            publish_offboard_control_mode();

            if (!local_position_fresh())
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "UAV %d has no fresh local position; holding zero setpoint", uav_id_);
                offboard_setpoint_counter_ = 0;
                publish_zero_velocity_setpoint();
                return;
            }

            if (offboard_setpoint_counter_ == 10 && !offboard_requested_)
            {
                this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                offboard_requested_ = true;
            }
            if (offboard_setpoint_counter_ < 11)
                offboard_setpoint_counter_++;

            if (offboard_setpoint_counter_ >= 11)
            {
                if (offboard_ready_ && auto_arm_ && !armed_)
                {
                    this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
                }
                if (!vehicle_status_received_ || !offboard_ready_)
                {
                    publish_zero_velocity_setpoint();
                    return;
                }
                px4_msgs::msg::TrajectorySetpoint msg{};
                msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
                msg.position = {NAN, NAN, NAN};
                msg.yaw = NAN;

                if (mode_ == "outdoor" && !origin_set_)
                {
                    msg.velocity = {0.0, 0.0, 0.0};
                    pub_traj_setpoint_->publish(msg);
                    return;
                }

                Eigen::Vector3d velocity_enu(0, 0, 0);
                float target_yaw = NAN; // [Fix] 必须在这里初始化

                if (flag_algo_ == 0)
                {
                    // === 阶段A: 起飞 (Position Hold) ===

                    // 1. 如果还没记录起飞原点，记录下来
                    if (!home_pos_recorded_)
                    {
                        home_pos_ = current_pos_;
                        home_pos_recorded_ = true;
                        RCLCPP_INFO(this->get_logger(), "Takeoff Home Recorded: [%.2f, %.2f]", home_pos_.x(), home_pos_.y());
                    }

                    // 2. 计算 XYZ 三轴误差
                    float err_x = home_pos_.x() - current_pos_.x();
                    float err_y = home_pos_.y() - current_pos_.y();
                    float err_z = takeoff_height_ - current_pos_.z();

                    // 3. P控制 (Kp = 1.0~1.5 比较柔和)
                    float k_takeoff = 1.5f;

                    // 4. 计算速度并限幅 (防止飘太远猛拉回来)
                    float vx = std::max(std::min(1.0f, k_takeoff * err_x), -1.0f);
                    float vy = std::max(std::min(1.0f, k_takeoff * err_y), -1.0f);
                    float vz = std::max(std::min(0.5f, err_z), -0.5f);

                    velocity_enu = Eigen::Vector3d(vx, vy, vz);
                }
                else
                {
                    // === 阶段B: 协同算法 + 避障 ===

                    // 1. 计算协同控制律 (引力速度)
                    auto output = controller_->compute_control(
                        current_pos_, w_self_, w_prev_, w_next_,
                        topology_->get_delta_prev(), topology_->get_delta_next(),
                        takeoff_height_, dt_);
                    w_self_ = output.w_updated;
                    Eigen::Vector3d nominal_vel = output.velocity;

                    // 2. 准备邻居位置数据
                    std::vector<Eigen::Vector3d> neighbors_pos_list;
                    for (auto const &[id, pos] : neighbor_map_)
                    {
                        // 简单的技巧：如果邻居位置是 (0,0,0) (未收到数据)，则跳过，防止误判
                        if (neighbor_fresh(id))
                        {
                            neighbors_pos_list.push_back(pos);
                        }
                    }

                    if (orca_enabled_)
                    {
                        // 2. 准备 RVO 数据
                        std::vector<Eigen::Vector2d> n_pos, n_vel;
                        std::vector<double> n_yaw, n_radii;
                        double my_radius = this->get_parameter("orca.safe_dist").as_double();

                        for (auto const &[id, pos] : neighbor_map_)
                        {
                            if (neighbor_fresh(id))
                            {
                                n_pos.push_back(Eigen::Vector2d(pos.x(), pos.y()));
                                Eigen::Vector3d v3 = neighbor_vel_map_[id];
                                n_vel.push_back(Eigen::Vector2d(v3.x(), v3.y()));
                                n_yaw.push_back(0.0); // 标准RVO不需要邻居偏航
                                n_radii.push_back(my_radius);
                            }
                        }

                        // [核心修改] 构造一个合成目标点 (Synthetic Goal)
                        // 原理: RVO2 内部会计算 pref_vel = (Goal - Curr).normalized() * max_speed
                        // 为了让 RVO2 尊重我们的 nominal_vel (编队力)，我们构造 Goal = Curr + nominal_vel
                        // 这样 RVO2 内部计算出的 Preferred Velocity 就约等于我们的 nominal_vel
                        Eigen::Vector2d synthetic_goal_2d(
                            current_pos_.x() + nominal_vel.x(),
                            current_pos_.y() + nominal_vel.y());

                        // 3. 计算避障后的平面速度
                        Eigen::Vector3d safe_vel_2d = orca_barrier_->compute_safe_velocity(
                            Eigen::Vector2d(current_pos_.x(), current_pos_.y()),
                            Eigen::Vector2d(current_vel_.x(), current_vel_.y()),
                            0.0, // 本机偏航在标准RVO中未被使用 (假设全向)
                            synthetic_goal_2d,
                            n_pos, n_vel, n_yaw, my_radius, n_radii);

                        // 4. 融合垂直速度 (RVO只处理XY平面)
                        velocity_enu.x() = safe_vel_2d.x();
                        velocity_enu.y() = safe_vel_2d.y();
                        velocity_enu.z() = nominal_vel.z(); // 垂直方向不做避障，直接用控制器的
                    }
                    else
                    {
                        velocity_enu = nominal_vel;
                    }

                    // 5. 计算 Yaw
                    int yaw_mode = this->get_parameter("mission.yaw_mode").as_int();
                    if (yaw_mode == 1 && velocity_enu.norm() > 0.1)
                    {
                        float yaw_enu = std::atan2(velocity_enu.y(), velocity_enu.x());
                        target_yaw = -yaw_enu + M_PI_2; // ENU -> NED
                    }
                    else
                    {
                        target_yaw = 0.0f;
                    }
                    // 4. Debug 发布
                    std_msgs::msg::Float32MultiArray debug_msg;
                    debug_msg.data = {
                        (float)output.target_pos.x(), (float)output.target_pos.y(),
                        (float)current_pos_.x(), (float)current_pos_.y()};
                    pub_debug_phi_->publish(debug_msg);
                }

                // 发布速度指令 (注意 ENU -> NED 转换: x_ros -> y_px4, y_ros -> x_px4, z_ros -> -z_px4)
                msg.velocity = {(float)velocity_enu.y(), (float)velocity_enu.x(), (float)-velocity_enu.z()};
                msg.yaw = target_yaw; // [Fix] 发布 Yaw
                pub_traj_setpoint_->publish(msg);

                std_msgs::msg::Float32 w_msg;
                w_msg.data = w_self_;
                pub_w_self_->publish(w_msg);
            }
        }
        else
        {
            // 如果收到停止指令，重置状态
            offboard_setpoint_counter_ = 0;
            home_pos_recorded_ = false; // [重要] 允许下次起飞重新记录原点
            offboard_requested_ = false;
        }
    }

    bool local_position_fresh() const
    {
        return current_position_received_ &&
               (this->now() - last_local_position_time_).seconds() <= odom_timeout_;
    }

    bool neighbor_fresh(int id) const
    {
        const auto it = neighbor_last_seen_.find(id);
        return it != neighbor_last_seen_.end() &&
               (this->now() - it->second).seconds() <= neighbor_timeout_;
    }

    void publish_zero_velocity_setpoint()
    {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position = {NAN, NAN, NAN};
        msg.velocity = {0.0F, 0.0F, 0.0F};
        msg.yaw = NAN;
        pub_traj_setpoint_->publish(msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardNode>());
    rclcpp::shutdown();
    return 0;
}
