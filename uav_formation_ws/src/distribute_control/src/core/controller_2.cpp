#include "distribute_control/core/controller_2.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>

namespace distribute_control
{

    DistributeController_2::DistributeController_2()
    {
    }

    void DistributeController_2::set_curve(std::shared_ptr<CurveBase_2> curve)
    {
        curve_ = curve;
    }

    void DistributeController_2::update_params(const ControlParams_2 &params)
    {
        params_ = params;
    }

    ControlOutput_2 DistributeController_2::compute_control(
        const Eigen::Vector3d &curr_pos,
        double w1_self, double w2_self,
        double w1_prev, double w2_prev,
        double w1_next, double w2_next,
        double delta1_prev, double delta2_prev,
        double delta1_next, double delta2_next,
        double target_z,
        double dt)
    {
        ControlOutput_2 output;

        // 安全检查
        if (!curve_)
        {
            std::cerr << "[Controller_2] Error: Curve not set!" << std::endl;
            output.velocity = Eigen::Vector3d::Zero();
            output.w1_updated = w1_self;
            output.w2_updated = w2_self;
            return output;
        }

        // =========================================================
        // 0. 参数定义 (对应 MATLAB 公式中的常数)
        // =========================================================
        double alpha = 1.0;

        // MATLAB 中的手动速度参数 manual_v = [0 0 0 1 1]
        // 但根据你的公式注释: v1=0, v2=1
        double v1 = 0.0;
        double v2 = 0.3; // 0.3

        // 增益 k
        double k_pos = params_.k_pos; // 对应 k1, k2
        double k_c = params_.k_c;     // 协同增益

        // =========================================================
        // 1. 几何信息查询 (Calculate f(w) and df(w))
        // =========================================================
        // 获取期望位置 f(w)
        // 注意：get_position 不再接受 height 参数，Z 轴逻辑后置处理
        Eigen::Vector3d f_w = curve_->get_position(w1_self, w2_self, target_z); // [修改点] 去掉 target_z

        // 处理 Z 轴: 如果曲线是平面的，强制使用 target_z
        // 这里为了计算 phi 准确，先把 f_w 的 z 设置为 target_z
        // 或者保留 f_w 原样，在 phi 计算时处理 z 轴误差
        if (std::abs(f_w.z()) < 0.01)
        {
            f_w.z() = target_z;
        }

        // 获取切向量 df (对应 MATLAB 中的 beta*df)
        // first -> df/dw1, second -> df/dw2
        auto tangents = curve_->get_tangents(w1_self, w2_self);
        Eigen::Vector3d df_dw1 = tangents.first;
        Eigen::Vector3d df_dw2 = tangents.second;

        // =========================================================
        // 2. 计算误差 Phi (Calculate Phi)
        // =========================================================
        // MATLAB: phi = alpha * (x - f1w)
        Eigen::Vector3d phi = alpha * (curr_pos - f_w);

        // =========================================================
        // 3. 计算路径跟随向量场 (PFVF) - 严格对应 MATLAB 公式
        // =========================================================

        // --- Part A: 位置控制部分 (dX/dt) ---
        // MATLAB:
        // term1 = v2*df1w1 - v1*df1w2 - k1 * phi1; (x轴)
        // term2 = v2*df2w1 - v1*df2w2 - k2 * phi2; (y轴)
        // 向量化表达: v_cmd = v2 * df_dw1 - v1 * df_dw2 - k * phi

        Eigen::Vector3d term_feedforward = v2 * df_dw1 - v1 * df_dw2;
        Eigen::Vector3d term_feedback = -k_pos * phi;

        Eigen::Vector3d v_cmd = term_feedforward + term_feedback;

        // --- Part B: 虚拟参数动态部分 (dw/dt) ---
        // MATLAB:
        // term3 (dw1) = v2 + k1 * phi * df_dw1
        // term4 (dw2) = -v1 + k2 * phi * df_dw2
        // 注意：MATLAB 代码里其实隐含了点乘 phi * df，因为 phi 和 df 都是向量

        // 针对 w1 的 PFVF
        double correction_w1 = k_pos * phi.dot(df_dw1);
        double dw1_pfvf = v2 + correction_w1;

        // 针对 w2 的 PFVF
        double correction_w2 = k_pos * phi.dot(df_dw2);
        double dw2_pfvf = -v1 + correction_w2;

        // =========================================================
        // 4. 计算协同向量场 (COVF: Consensus Vector Field)
        // =========================================================
        // MATLAB:
        // W1(j) = w1 + (v(3) + kc * c1(j)) * snap;
        // W2(j) = w2 + (v(4) + kc * c2(j)) * snap;
        // 这里 c1, c2 是协同误差

        double err1_prev = w1_self - w1_prev - delta1_prev;
        double err1_next = w1_self - w1_next - delta1_next;
        double covf_w1 = -(err1_prev + err1_next);

        double err2_prev = w2_self - w2_prev - delta2_prev;
        double err2_next = w2_self - w2_next - delta2_next;
        double covf_w2 = -(err2_prev + err2_next);

        // =========================================================
        // 5. 总输出 (Total Dynamics)
        // =========================================================
        // dx/dt 直接作为速度指令
        // dw/dt = pfvf + kc * covf

        double dot_w1 = dw1_pfvf + k_c * covf_w1;
        double dot_w2 = dw2_pfvf + k_c * covf_w2;

        // 积分更新 w
        double w1_next_val = w1_self + dt * dot_w1;
        double w2_next_val = w2_self + dt * dot_w2;

        // =========================================================
        // 6. 后处理 (Z轴修正与限幅)
        // =========================================================

        // Z轴额外修正: MATLAB 代码主要处理平面，Z轴需要单独拉回
        double z_err = curr_pos.z() - f_w.z();
        // 如果 target_z 不在 f_w 中，则需额外处理
        // 简单 P 控制维持高度
        v_cmd.z() = -1.0 * z_err;
        v_cmd.z() = std::max(-0.8, std::min(0.8, v_cmd.z()));

        // XY 速度限幅 (保持方向)
        double v_norm_xy = std::sqrt(v_cmd.x() * v_cmd.x() + v_cmd.y() * v_cmd.y()) + 1e-6;
        if (v_norm_xy > params_.max_speed)
        {
            double scale = params_.max_speed / v_norm_xy;
            v_cmd.x() *= scale;
            v_cmd.y() *= scale;
        }

        // =========================================================
        // 7. 填充输出结构体
        // =========================================================
        output.velocity = v_cmd;
        output.w1_updated = w1_next_val;
        output.w2_updated = w2_next_val;
        output.target_pos = f_w;
        output.current_error = phi;

        return output;
    }

} // namespace distribute_control