#pragma once
#include "curve_base_2.hpp"
#include <cmath>

namespace distribute_control
{

    class EightCurve_2 : public CurveBase_2
    {
    public:
        void load_params(const std::vector<double> &params) override;
        Eigen::Vector3d get_position(double w1, double w2, double height) override;
        std::pair<Eigen::Vector3d, Eigen::Vector3d> get_tangents(double w1, double w2) override;
        std::string get_type() const override { return "eight"; }

    private:
        double a_ = 3.0;    // 宽度范围
        double b_ = 3.0;    // 高度范围
        double radius_ = 1; // 曲线半径
    };

}