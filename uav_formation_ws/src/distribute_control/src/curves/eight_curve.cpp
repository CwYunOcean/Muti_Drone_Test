#include "distribute_control/curves/eight_curve.hpp"

namespace distribute_control
{

    void EightCurve::load_params(const std::vector<double> &params)
    {
        if (params.size() >= 2)
        {
            a_ = params[0];
            b_ = params[1];
        }
    }

    Eigen::Vector3d EightCurve::get_position(double w, double height)
    {
        // x = a * sin(w)
        // y = b * sin(w) * cos(w)
        return Eigen::Vector3d(a_ * std::sin(w), b_ * std::sin(w) * std::cos(w), height);
    }

    Eigen::Vector3d EightCurve::get_tangent(double w)
    {
        // dx = a * cos(w)
        // dy = b * (cos^2(w) - sin^2(w)) = b * cos(2w)
        double dx = a_ * std::cos(w);
        double dy = b_ * std::cos(2.0 * w);
        return Eigen::Vector3d(dx, dy, 0.0);
    }

}