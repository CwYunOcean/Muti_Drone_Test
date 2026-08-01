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

// 引入库
#include "distribute_control/core/controller_2.hpp"
#include "distribute_control/core/topology.hpp" // 依然需要拓扑类来获取ID
#include "distribute_control/curves/eight_curve_2.hpp"
#include "distribute_control/core/orca_barrier.hpp"
#include "distribute_control/core/topic_names.hpp"
#include "distribute_control/core/world_transform.hpp"

using namespace std::chrono_literals;

static constexpr double EARTH_RADIUS = 6378137.0;

enum class CurveID
{
    EIGHT_2 = 1,
    // SQUIRCLE_2 = 2
};

class OffboardNode_2 : public rclcpp::Node
{
public:
    OffboardNode_2() : Node("offboard_node_2")
    {
        // =======================================================
        // 1. 参数加载
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
        total_uavs_ = this->get_parameter("system.total_uavs").as_int();
        double loop_rate = this->get_parameter("system.loop_rate").as_double();
        mode_ = this->get_parameter("system.location_mode").as_string();
        is_sim_ = this->get_parameter("system.is_sim").as_bool();
        const int configured_target_system = this->get_parameter("system.target_system").as_int();
        target_system_ = configured_target_system > 0 ? configured_target_system : uav_id_ + 1;
        auto_arm_ = this->get_parameter("control.auto_arm").as_bool();
        odom_timeout_ = this->get_parameter("control.odom_timeout").as_double();
        neighbor_timeout_ = this->get_parameter("control.neighbor_timeout").as_double();
        dt_ = 1.0 / loop_rate;

        // Mission
        this->declare_parameter("mission.takeoff_height", 1.0);
        this->declare_parameter("mission.formation_spacing", 2.0); // 这里实际可能用作 Delta 计算参考
        this->declare_parameter("mission.curve_id", 1);
        this->declare_parameter("mission.curve_params", std::vector<double>{5.0, 5.0, 2.0}); // a, b, radius
        this->declare_parameter("mission.yaw_mode", 1);

        takeoff_height_ = this->get_parameter("mission.takeoff_height").as_double();
        const double formation_spacing = this->get_parameter("mission.formation_spacing").as_double();
        int curve_id_int = this->get_parameter("mission.curve_id").as_int();
        std::vector<double> c_params = this->get_parameter("mission.curve_params").as_double_array();

        // Offset
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

        RCLCPP_INFO(this->get_logger(), ">>> UAV %d (Double-W) Init. CurveID: %d", uav_id_, curve_id_int);

        // =======================================================
        // 2. 控制器与曲线初始化
        // =======================================================
        // Gains
        this->declare_parameter("gains.k_pos", 3.0);
        this->declare_parameter("gains.k_c", 2.0);
        this->declare_parameter("gains.max_speed", 1.0);

        distribute_control::ControlParams_2 cp;
        cp.k_pos = this->get_parameter("gains.k_pos").as_double();
        cp.k_c = this->get_parameter("gains.k_c").as_double();
        cp.max_speed = this->get_parameter("gains.max_speed").as_double();

        // ORCA
        this->declare_parameter("orca.enabled", true);
        this->declare_parameter("orca.safe_dist", 0.5);
        this->declare_parameter("orca.time_horizon", 2.0);

        orca_enabled_ = this->get_parameter("orca.enabled").as_bool();
        if (orca_enabled_)
        {
            orca_barrier_ = std::make_unique<distribute_control::ORCABarrier>();
            distribute_control::ORCAParams orca_p;
            orca_p.safe_dist = this->get_parameter("orca.safe_dist").as_double();
            orca_p.time_horizon = this->get_parameter("orca.time_horizon").as_double();
            orca_p.max_speed = cp.max_speed;
            orca_barrier_->set_params(orca_p);
        }

        // 拓扑
        topology_ = std::make_unique<distribute_control::Topology>(total_uavs_, uav_id_);

        // 曲线
        std::shared_ptr<distribute_control::CurveBase_2> curve;
        // 默认使用 8字曲线 V2
        curve = std::make_shared<distribute_control::EightCurve_2>();
        curve->load_params(c_params);

        controller_2 = std::make_unique<distribute_control::DistributeController_2>();
        controller_2->set_curve(curve);
        controller_2->update_params(cp);

        // =======================================================
        // 3. 通信订阅
        // =======================================================
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        const std::string ns = distribute_control::dronePrefix(uav_id_);

        sub_local_pos_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            ns + "/fmu/out/vehicle_local_position", qos,
            std::bind(&OffboardNode_2::local_pos_cb, this, std::placeholders::_1));

        // 邻居物理位置订阅
        for (int i = 1; i <= total_uavs_; ++i)
        {
            if (i == uav_id_)
                continue;
            const std::string other_ns = distribute_control::dronePrefix(i);

            auto sub = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                other_ns + "/fmu/out/vehicle_local_position", qos,
                [this, i, formation_spacing](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
                {
                    const auto grid_offset = distribute_control::formationGridOffset(
                        i, total_uavs_, formation_spacing);
                    neighbor_map_[i] = world_transform_.positionFromPx4Ned(
                        Eigen::Vector3d(msg->x, msg->y, msg->z));
                    if (is_sim_ && mode_ == "indoor")
                    {
                        neighbor_map_[i] += Eigen::Vector3d(grid_offset.x, grid_offset.y, 0.0);
                    }
                    neighbor_vel_map_[i] = world_transform_.velocityFromPx4Ned(
                        Eigen::Vector3d(msg->vx, msg->vy, msg->vz));
                    neighbor_last_seen_[i] = this->now();
                });
            neighbor_subs_.push_back(sub);
        }

        // Publisher 初始化
        pub_offboard_mode_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(ns + "/fmu/in/offboard_control_mode", qos);
        pub_traj_setpoint_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(ns + "/fmu/in/trajectory_setpoint", qos);
        pub_vehicle_cmd_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(ns + "/fmu/in/vehicle_command", qos);

        // Debug & Virtual State Publisher
        pub_debug_phi_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(ns + "/phi", qos);

        // [修改] 发布两个虚拟变量 w1, w2 到不同话题
        pub_w1_self_ = this->create_publisher<std_msgs::msg::Float32>(ns + "/virtual_w1", qos);
        pub_w2_self_ = this->create_publisher<std_msgs::msg::Float32>(ns + "/virtual_w2", qos);

        // 虚拟量 w 通信 (Prev/Next)
        int prev_id = topology_->get_prev_id();
        int next_id = topology_->get_next_id();

        // [修改] 分别订阅 w1 和 w2
        // Prev Neighbor
        sub_w1_prev_ = this->create_subscription<std_msgs::msg::Float32>(
            distribute_control::dronePrefix(prev_id) + "/virtual_w1", qos,
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            { w1_prev_ = msg->data; });
        sub_w2_prev_ = this->create_subscription<std_msgs::msg::Float32>(
            distribute_control::dronePrefix(prev_id) + "/virtual_w2", qos,
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            { w2_prev_ = msg->data; });

        // Next Neighbor
        sub_w1_next_ = this->create_subscription<std_msgs::msg::Float32>(
            distribute_control::dronePrefix(next_id) + "/virtual_w1", qos,
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            { w1_next_ = msg->data; });
        sub_w2_next_ = this->create_subscription<std_msgs::msg::Float32>(
            distribute_control::dronePrefix(next_id) + "/virtual_w2", qos,
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            { w2_next_ = msg->data; });

        // Cmd
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

        timer_ = this->create_wall_timer(std::chrono::duration<double>(dt_), std::bind(&OffboardNode_2::timer_callback, this));
    }

private:
    int uav_id_;
    int total_uavs_;
    uint8_t target_system_ = 1;
    double dt_;
    bool orca_enabled_ = false;
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

    Eigen::Vector3d current_pos_{0, 0, 0};
    Eigen::Vector3d current_vel_{0, 0, 0};
    std::map<int, Eigen::Vector3d> neighbor_map_;
    std::map<int, Eigen::Vector3d> neighbor_vel_map_;
    std::map<int, rclcpp::Time> neighbor_last_seen_;

    bool home_pos_recorded_ = false;
    Eigen::Vector3d home_pos_{0, 0, 0};

    // [新增] 双 W 变量存储
    double w1_self_ = 0.0;
    double w2_self_ = 0.0;
    double w1_prev_ = 0.0;
    double w1_next_ = 0.0;
    double w2_prev_ = 0.0;
    double w2_next_ = 0.0;

    int flag_start_ = 0;
    int flag_algo_ = 0;
    int offboard_setpoint_counter_ = 0;

    std::unique_ptr<distribute_control::Topology> topology_;
    std::unique_ptr<distribute_control::DistributeController_2> controller_2; // 使用 Controller_2
    std::unique_ptr<distribute_control::ORCABarrier> orca_barrier_;

    std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr> neighbor_subs_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_pos_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_vehicle_status_;

    // Publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offboard_mode_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_traj_setpoint_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_vehicle_cmd_;

    // Virtual State Subs
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_w1_prev_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_w1_next_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_w2_prev_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_w2_next_;

    // Virtual State Pubs
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_w1_self_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_w2_self_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_debug_phi_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_start_cmd_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_algo_cmd_;

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

                Eigen::Vector3d velocity_enu(0, 0, 0);
                float target_yaw = NAN;

                if (flag_algo_ == 0) // Takeoff Phase
                {
                    if (!home_pos_recorded_)
                    {
                        home_pos_ = current_pos_;
                        home_pos_recorded_ = true;
                    }
                    float err_x = home_pos_.x() - current_pos_.x();
                    float err_y = home_pos_.y() - current_pos_.y();
                    float err_z = takeoff_height_ - current_pos_.z();
                    float k_takeoff = 1.5f;
                    velocity_enu = Eigen::Vector3d(
                        std::max(std::min(1.0f, k_takeoff * err_x), -1.0f),
                        std::max(std::min(1.0f, k_takeoff * err_y), -1.0f),
                        std::max(std::min(0.5f, err_z), -0.5f));
                    target_yaw = 0.0f;
                }
                else // Algorithm Phase (Double W)
                {
                    // 1. 设置 Delta (间距)
                    // 假设 w1 (主参数) 沿圆周均匀分布，w2 (副参数) 同步
                    double delta1_prev = 0.0;
                    double delta1_next = 0.0;
                    double delta2_prev = topology_->get_delta_prev();
                    double delta2_next = topology_->get_delta_next();

                    // 2. 计算控制
                    auto output = controller_2->compute_control(
                        current_pos_,
                        w1_self_, w2_self_,
                        w1_prev_, w2_prev_,
                        w1_next_, w2_next_,
                        delta1_prev, delta2_prev,
                        delta1_next, delta2_next,
                        takeoff_height_, // 使用起飞高度作为参考面高度
                        dt_);

                    // 3. 更新自身状态
                    w1_self_ = output.w1_updated;
                    w2_self_ = output.w2_updated;
                    Eigen::Vector3d nominal_vel = output.velocity;

                    // 4. 避障
                    if (orca_enabled_)
                    {
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
                                n_yaw.push_back(0.0);
                                n_radii.push_back(my_radius);
                            }
                        }

                        // 合成目标点 logic
                        Eigen::Vector2d synthetic_goal_2d(
                            current_pos_.x() + nominal_vel.x(),
                            current_pos_.y() + nominal_vel.y());
                        Eigen::Vector3d safe_vel_2d = orca_barrier_->compute_safe_velocity(
                            Eigen::Vector2d(current_pos_.x(), current_pos_.y()),
                            Eigen::Vector2d(current_vel_.x(), current_vel_.y()),
                            0.0, synthetic_goal_2d, n_pos, n_vel, n_yaw, my_radius, n_radii);

                        velocity_enu.x() = safe_vel_2d.x();
                        velocity_enu.y() = safe_vel_2d.y();
                        velocity_enu.z() = nominal_vel.z();
                    }
                    else
                    {
                        velocity_enu = nominal_vel;
                    }

                    // 5. Yaw
                    if (this->get_parameter("mission.yaw_mode").as_int() == 1 && velocity_enu.norm() > 0.1)
                    {
                        float yaw_enu = std::atan2(velocity_enu.y(), velocity_enu.x());
                        target_yaw = -yaw_enu + M_PI_2;
                    }
                    else
                    {
                        target_yaw = 0.0f;
                    }

                    // 6. Debug Data
                    std_msgs::msg::Float32MultiArray debug_msg;

                    // 将数据填充到数组中
                    // 布局定义: [0]=phi_x, [1]=phi_y, [2]=phi_z, [3]=target_x, [4]=target_y, [5]=curr_x, [6]=curr_y
                    debug_msg.data = {
                        (float)output.current_error.x(),                                                                             // phi x
                        (float)output.current_error.y(),                                                                             // phi y
                        (float)output.current_error.z(),                                                                             // phi z
                        (float)output.target_pos.x(),                                                                                // 期望位置 x
                        (float)output.target_pos.y(),                                                                                // 期望位置 y
                        (float)current_pos_.x(),                                                                                     // 当前位置 x
                        (float)current_pos_.y(),                                                                                     // 当前位置 y
                        (float)sqrt(velocity_enu.y() * (float)velocity_enu.y() + (float)velocity_enu.x() * (float)velocity_enu.x()), // 速度大小,
                        (float)(w1_self_ - w1_prev_ - delta1_prev),
                        (float)(w2_self_ - w2_prev_ - delta2_prev),
                    };

                    pub_debug_phi_->publish(debug_msg);
                }

                // 发布控制
                msg.velocity = {(float)velocity_enu.y(), (float)velocity_enu.x(), (float)-velocity_enu.z()};
                msg.yaw = target_yaw;
                pub_traj_setpoint_->publish(msg);

                // 发布虚拟状态
                std_msgs::msg::Float32 w1_msg;
                w1_msg.data = w1_self_;
                pub_w1_self_->publish(w1_msg);

                std_msgs::msg::Float32 w2_msg;
                w2_msg.data = w2_self_;
                pub_w2_self_->publish(w2_msg);
            }
        }
        else
        {
            offboard_setpoint_counter_ = 0;
            home_pos_recorded_ = false;
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
    rclcpp::spin(std::make_shared<OffboardNode_2>());
    rclcpp::shutdown();
    return 0;
}
