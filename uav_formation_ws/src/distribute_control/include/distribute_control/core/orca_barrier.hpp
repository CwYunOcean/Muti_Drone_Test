#ifndef DISTRIBUTE_CONTROL_ORCA_BARRIER_HPP
#define DISTRIBUTE_CONTROL_ORCA_BARRIER_HPP

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <limits>

namespace distribute_control
{

    struct ORCAParams
    {
        double safe_dist = 0.5;    // 碰撞半径缓冲 (radius buffer)
        double time_horizon = 2.0; // 避障时间窗 (tau)
        double max_speed = 2.0;    // 最大物理速度
    };

    // 定义半平面约束: point 是线上一点, direction 是线的方向向量
    struct Line
    {
        Eigen::Vector2d point;
        Eigen::Vector2d direction;
    };

    class ORCABarrier
    {
    public:
        ORCABarrier();
        ~ORCABarrier() = default;

        void set_params(const ORCAParams &params);

        /**
         * @brief 标准 RVO2 计算接口
         * * @param curr_pos 当前位置 (2D)
         * @param curr_vel 当前速度 (2D)
         * @param curr_yaw 当前偏航 (标准RVO2通常忽略此项，视作全向移动)
         * @param goal_pos 目标位置 (2D)
         * @param neighbors_pos 邻居位置列表
         * @param neighbors_vel 邻居速度列表
         * @param neighbors_yaw 邻居偏航列表 (标准RVO2忽略)
         * @param radius_i 自身半径
         * @param radius_neighbors 邻居半径列表
         * @return Eigen::Vector3d 输出的安全速度 (z轴通常为0)
         */
        Eigen::Vector3d compute_safe_velocity(
            const Eigen::Vector2d &curr_pos,
            const Eigen::Vector2d &curr_vel,
            double curr_yaw,
            const Eigen::Vector2d &goal_pos,
            const std::vector<Eigen::Vector2d> &neighbors_pos,
            const std::vector<Eigen::Vector2d> &neighbors_vel,
            const std::vector<double> &neighbors_yaw,
            double radius_i,
            const std::vector<double> &radius_neighbors);

    private:
        ORCAParams params_;

        // ==========================================
        // 标准 RVO2 线性规划求解器 (Linear Programming)
        // ==========================================

        // LP1: 在一条线的约束下优化速度
        bool linearProgram1(const std::vector<Line> &lines, size_t lineNo,
                            double radius, const Eigen::Vector2d &optVelocity,
                            bool directionOpt, Eigen::Vector2d &result);

        // LP2: 处理所有约束的主循环
        size_t linearProgram2(const std::vector<Line> &lines, double radius,
                              const Eigen::Vector2d &optVelocity, bool directionOpt,
                              Eigen::Vector2d &result);

        // LP3: 当无解时 (线性约束围成的区域为空)，寻找“最不坏”的速度
        void linearProgram3(const std::vector<Line> &lines, size_t numObstLines,
                            double radius, const Eigen::Vector2d &optVelocity,
                            bool directionOpt, Eigen::Vector2d &result);
    };

} // namespace distribute_control

#endif