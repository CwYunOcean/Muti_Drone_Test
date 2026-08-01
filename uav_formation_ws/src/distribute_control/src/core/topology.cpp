#include "distribute_control/core/topology.hpp"

namespace distribute_control
{

    Topology::Topology(int total_uavs, int self_id)
        : N_(total_uavs), id_(self_id)
    {
        // 基础步长 delta = 2π / N
        phase_step_ = 2.0 * M_PI / static_cast<double>(N_);
    }

    int Topology::get_prev_id() const
    {
        // (i - 2 + N) % N + 1
        return (id_ - 2 + N_) % N_ + 1;
    }

    int Topology::get_next_id() const
    {
        // (i % N) + 1
        return (id_ % N_) + 1;
    }

    // 修正的核心：根据 ID 计算期望的相位差
    double Topology::get_delta_prev() const
    {
        // 1. 获取邻居 ID
        int prev_id = get_prev_id();

        // 2. 计算各自的全局期望相位 (Bias)
        // 你的逻辑中：Bias = (id - 1) * delta
        double bias_self = (id_ - 1) * phase_step_;
        double bias_prev = (prev_id - 1) * phase_step_;

        // 3. 返回差值 (Bias_self - Bias_prev)
        // 例如 UAV 1 (prev 3): 0 - 2*delta = -2*delta
        return bias_self - bias_prev;
    }

    double Topology::get_delta_next() const
    {
        int next_id = get_next_id();

        double bias_self = (id_ - 1) * phase_step_;
        double bias_next = (next_id - 1) * phase_step_;

        // 返回差值 (Bias_self - Bias_next)
        // 例如 UAV 1 (next 2): 0 - 1*delta = -delta
        return bias_self - bias_next;
    }

}