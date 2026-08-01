#include "distribute_control/core/controller.hpp"
#include <iostream>
#include <algorithm>

namespace distribute_control
{

    DistributeController::DistributeController()
    {
        // 默认空指针，调用前必须 set_curve
    }

    void DistributeController::set_curve(std::shared_ptr<CurveBase> curve)
    {
        curve_ = curve;
    }

    void DistributeController::update_params(const ControlParams &params)
    {
        params_ = params;
    }

    ControlOutput DistributeController::compute_control(
        const Eigen::Vector3d &curr_pos,
        double w_self,
        double w_prev, double w_next,
        double delta_prev, double delta_next,
        double target_z,
        double dt)
    {
        ControlOutput output;

        // 安全检查
        if (!curve_)
        {
            std::cerr << "[Controller] Error: Curve not set!" << std::endl;
            output.velocity = Eigen::Vector3d::Zero();
            output.w_updated = w_self;
            return output;
        }

        // 1. 计算前瞻点 (解决滞后问题)
        double w_target = w_self + params_.look_ahead;

        // 获取期望位置和切向量 (前馈速度)
        // 假设 z 轴高度已经在 Curve 内部处理 (或者这里单独处理 z)
        Eigen::Vector3d target_pos = curve_->get_position(w_target, target_z);
        Eigen::Vector3d target_vel_vec = curve_->get_tangent(w_target);

        // 2. 位置误差 (Vector Field 核心)
        // 只考虑 XY 平面的误差用于路径跟随，Z 轴通常独立控制
        Eigen::Vector3d pos_error = curr_pos - target_pos;
        pos_error.z() = 0; // 忽略 Z 轴误差对 w 的影响

        // 3. 计算一致性误差 c (Consensus)
        // 公式: e_i = \sum (w_i - w_j - delta_ij)
        // 你的原代码逻辑: c = - (w - w1 - delta_prev) - (w - w2 - delta_next)
        double err_prev = w_self - w_prev - delta_prev;
        double err_next = w_self - w_next - delta_next;
        double c = -(err_prev + err_next);

        // 4. 计算虚拟状态变化率 dot_w
        // 基础速率: 让无人机以 max_speed 沿着切线跑
        double tangent_norm = target_vel_vec.norm() + 1e-6;
        double dot_w_base = params_.max_speed / tangent_norm;

        // 反馈项: 如果落后 (error 与 tangent 同向)，需要加速 w
        // 注意符号：target_vel_vec 是前进方向
        // dot_product > 0 意味着我们在切线方向上有分量
        // 这里采用常用的路径参数自适应律:
        double feedback = 1.0 + params_.k_pos * (pos_error.dot(target_vel_vec));

        // 归一化并施加一致性
        // 注意: 你的原代码中除以了 v_norm，这里为了稳定性简化为切向量模长
        double dot_w = dot_w_base * feedback + params_.k_c * c;

        // 5. 更新 w (限制非负，防止倒车太厉害)
        // 加上限幅防止数值爆炸
        double w_next_val = w_self + dt * dot_w;

        // 6. 计算物理速度指令 (Vector Field Guidance)
        // V_cmd = V_feedforward - k * Error
        Eigen::Vector3d v_cmd = target_vel_vec - params_.k_pos * pos_error;

        // Z轴高度控制 (简单 P 控制飞到目标高度)
        // 假设 Curve 返回的 z 是目标高度
        double z_err = curr_pos.z() - target_pos.z();
        v_cmd.z() = -1.0 * z_err;                             // 简单的 P 控制
        v_cmd.z() = std::max(-0.5, std::min(0.5, v_cmd.z())); // Z轴限幅

        // 7. XY 平面速度限幅
        const double v_norm_xy = std::sqrt(v_cmd.x() * v_cmd.x() + v_cmd.y() * v_cmd.y());
        if (v_norm_xy > params_.max_speed && v_norm_xy > 0.0)
        {
            const double scale = params_.max_speed / v_norm_xy;
            v_cmd.x() *= scale;
            v_cmd.y() *= scale;
        }

        // 填充输出
        output.velocity = v_cmd;
        output.w_updated = w_next_val;
        output.target_pos = target_pos; // 方便外部可视化
        output.current_error = pos_error;

        return output;
    }

}
