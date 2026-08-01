#include "distribute_control/curves/squircle_curve.hpp"

namespace distribute_control
{

    void SquircleCurve::load_params(const std::vector<double> &params)
    {
        if (params.size() >= 3)
        {
            a_ = params[0];
            b_ = params[1];
            n_ = params[2];
        }
    }

    double SquircleCurve::sgn_pow(double val, double p)
    {
        if (val > 0)
            return std::pow(val, p);
        if (val < 0)
            return -std::pow(-val, p);
        return 0.0;
    }

    Eigen::Vector3d SquircleCurve::get_position(double w, double height)
    {
        // 超椭圆参数方程
        // x = a * |cos(w)|^(2/n) * sgn(cos(w))
        // y = b * |sin(w)|^(2/n) * sgn(sin(w))

        double p = 2.0 / n_;
        double x = a_ * sgn_pow(std::cos(w), p);
        double y = b_ * sgn_pow(std::sin(w), p);

        return Eigen::Vector3d(x, y, height);
    }

    Eigen::Vector3d SquircleCurve::get_tangent(double w)
    {
        // 链式法则求导
        // d/dw (sgn(cos w)|cos w|^p) = -p * |cos w|^(p-1) * sin w * sgn(cos w)

        double p = 2.0 / n_;
        double cos_w = std::cos(w);
        double sin_w = std::sin(w);

        // 防止除以0 (当 cos_w = 0 时)
        double eps = 1e-6;

        double term_x = std::pow(std::abs(cos_w) + eps, p - 1.0);
        double term_y = std::pow(std::abs(sin_w) + eps, p - 1.0);

        double sgn_cos = (cos_w >= 0) ? 1.0 : -1.0;
        double sgn_sin = (sin_w >= 0) ? 1.0 : -1.0;

        double dx = -a_ * p * term_x * sin_w * sgn_cos;
        double dy = b_ * p * term_y * cos_w * sgn_sin;

        return Eigen::Vector3d(dx, dy, 0.0);
    }

}