#include "distribute_control/curves/ellipse_curve.hpp"

namespace distribute_control {

void EllipseCurve::load_params(const std::vector<double>& params) {
    if (params.size() >= 2) {
        a_ = params[0];
        b_ = params[1];
    }
}

Eigen::Vector3d EllipseCurve::get_position(double w, double height) {
    // x = a * cos(w)
    // y = b * sin(w)
    return Eigen::Vector3d(a_ * std::cos(w), b_ * std::sin(w), height);
}

Eigen::Vector3d EllipseCurve::get_tangent(double w) {
    // dx = -a * sin(w)
    // dy =  b * cos(w)
    return Eigen::Vector3d(-a_ * std::sin(w), b_ * std::cos(w), 0.0);
}

}