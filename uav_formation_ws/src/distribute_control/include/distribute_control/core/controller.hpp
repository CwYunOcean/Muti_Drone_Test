#pragma once
#include <Eigen/Dense>
#include <memory>
#include "distribute_control/curves/curve_base.hpp"

namespace distribute_control {

// 控制参数结构体 (方便一次性传参)
struct ControlParams {
    double k_pos = 2.0;      // 位置误差增益
    double k_c = 1.0;        // 一致性增益
    double look_ahead = 0.5; // 前瞻距离
    double max_speed = 1.0;  // 最大速度
};

// 控制器输出结构体
struct ControlOutput {
    Eigen::Vector3d velocity;      // 速度指令
    double w_updated;              // 更新后的 w
    Eigen::Vector3d target_pos;    // 调试用：目标位置
    Eigen::Vector3d current_error; // 调试用：误差
};

class DistributeController {
public:
    DistributeController();

    // 设置使用的曲线 (策略模式)
    void set_curve(std::shared_ptr<CurveBase> curve);
    
    // 更新参数
    void update_params(const ControlParams& params);

    /**
     * @brief 核心计算函数 (每 10ms 调用一次)
     * * @param curr_pos 当前真实位置 (x, y, z)
     * @param w_self 当前虚拟参数 w
     * @param w_prev 前一个邻居的 w
     * @param w_next 后一个邻居的 w
     * @param delta_prev 与前一个的期望相位差 (来自 Topology)
     * @param delta_next 与后一个的期望相位差 (来自 Topology)
     * @param dt 时间步长 (0.01s)
     * @return ControlOutput 包含速度指令和新的 w
     */
    ControlOutput compute_control(
        const Eigen::Vector3d& curr_pos,
        double w_self,
        double w_prev, double w_next,
        double delta_prev, double delta_next,double target_z,
        double dt
    );

private:
    std::shared_ptr<CurveBase> curve_; // 当前使用的曲线
    ControlParams params_;
};

}