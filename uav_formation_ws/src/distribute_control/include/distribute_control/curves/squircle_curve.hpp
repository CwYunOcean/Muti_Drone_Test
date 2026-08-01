#pragma once
#include "curve_base.hpp"
#include <cmath>

namespace distribute_control {

class SquircleCurve : public CurveBase {
public:
    void load_params(const std::vector<double>& params) override;
    Eigen::Vector3d get_position(double w, double height) override;
    Eigen::Vector3d get_tangent(double w) override;
    std::string get_type() const override { return "squircle"; }

private:
    double a_ = 3.0; 
    double b_ = 3.0; 
    double n_ = 2.0; // n=2是圆, n=4是圆角矩形

    // 辅助函数: 带符号的幂运算 sgn(x) * |x|^p
    double sgn_pow(double val, double p);
};

}