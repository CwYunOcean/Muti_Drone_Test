#pragma once
#include "curve_base.hpp"
#include <cmath>

namespace distribute_control {

class EllipseCurve : public CurveBase {
public:
    void load_params(const std::vector<double>& params) override;
    Eigen::Vector3d get_position(double w, double height) override;
    Eigen::Vector3d get_tangent(double w) override;
    std::string get_type() const override { return "ellipse"; }

private:
    double a_ = 2.0; // 长轴 (x方向)
    double b_ = 2.0; // 短轴 (y方向)
};

}