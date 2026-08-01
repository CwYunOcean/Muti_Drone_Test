#pragma once
#include <cmath>
#include <vector>

namespace distribute_control {

class Topology {
public:
    /**
     * @param total_uavs 无人机总数 N
     * @param self_id 当前无人机 ID (1 ~ N)
     */
    Topology(int total_uavs, int self_id);

    // 获取邻居 ID (用于订阅 ROS 话题)
    int get_prev_id() const;
    int get_next_id() const;

    // 获取期望的相位偏移 (Delta w)
    // 例如：我应该比前一个邻居落后多少弧度
    double get_delta_prev() const;
    
    // 我应该比后一个邻居超前多少弧度
    double get_delta_next() const;

private:
    int N_;
    int id_;
    double phase_step_; // 2 * PI / N
};

}