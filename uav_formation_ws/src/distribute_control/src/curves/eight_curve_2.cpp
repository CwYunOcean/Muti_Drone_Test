#include "distribute_control/curves/eight_curve_2.hpp"

namespace distribute_control
{

    void EightCurve_2::load_params(const std::vector<double> &params)
    {
        if (params.size() >= 3)
        {
            a_ = params[0];
            b_ = params[1];
            radius_ = params[2];
        }
    }

    Eigen::Vector3d EightCurve_2::get_position(double w1, double w2, double height)
    {
        // x = a * sin(w1) + radius * cos(w2)
        // y = b * sin(w1) * cos(w1) + radius * sin(w2)
        // return Eigen::Vector3d(a_ * std::sin(w1) + radius_ * std::cos(w2), b_ * std::sin(w1) * std::cos(w1) + radius_ * std::sin(w2), height);
        return Eigen::Vector3d(a_ * std::sin(w1) + radius_ * std::cos(w2), b_ * std::cos(w1) + radius_ * std::sin(w2), height);
    }

    // [Fix] 修复函数定义语法，并分开计算偏导数
    std::pair<Eigen::Vector3d, Eigen::Vector3d> EightCurve_2::get_tangents(double w1, double w2)
    {
        // 1. 对 w1 求偏导 (视为 w2 是常数)
        // d(sin(w1))/dw1 = cos(w1)
        // d(sin(w1)cos(w1))/dw1 = d(0.5*sin(2w1))/dw1 = cos(2w1)
        Eigen::Vector3d df_dw1(a_ * std::cos(w1), -b_ * std::sin(w1), 0.0);
        // 2. 对 w2 求偏导 (视为 w1 是常数)
        // d(cos(w2))/dw2 = -sin(w2)
        // d(sin(w2))/dw2 = cos(w2)
        double dx_dw2 = -radius_ * std::sin(w2);
        double dy_dw2 = radius_ * std::cos(w2);
        Eigen::Vector3d df_dw2(dx_dw2, dy_dw2, 0.0);

        return std::make_pair(df_dw1, df_dw2);
    }

}
