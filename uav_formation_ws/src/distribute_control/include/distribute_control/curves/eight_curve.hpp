#pragma once
#include "curve_base.hpp"
#include <cmath>

namespace distribute_control {

class EightCurve : public CurveBase {
public:
    void load_params(const std::vector<double>& params) override;
    Eigen::Vector3d get_position(double w, double height) override;
    Eigen::Vector3d get_tangent(double w) override;
    std::string get_type() const override { return "eight"; }

private:
    double a_ = 3.0; // 宽度范围
    double b_ = 3.0; // 高度范围
};

}