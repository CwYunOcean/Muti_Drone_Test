#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <iostream>
#include <utility> // [新增] 必须包含，用于 std::pair

namespace distribute_control
{

    /**
     * @brief 双参数曲线抽象基类 (Interface)
     * 适用于定义由 (w1, w2) 决定的曲面或复杂轨迹
     */
    class CurveBase_2
    {
    public:
        virtual ~CurveBase_2() = default;

        /**
         * @brief 加载参数
         * @param params [radius, beta, z_scale, ...]
         */
        virtual void load_params(const std::vector<double> &params) = 0;

        /**
         * @brief 计算期望位置 p(w1, w2)
         * 注意：对于3D曲线，Z轴由曲线方程决定；对于2D曲线，Z轴通常返回0
         * @param w1 参数1
         * @param w2 参数2
         * @return Eigen::Vector3d 期望位置 (x_d, y_d, z_d)
         */
        virtual Eigen::Vector3d get_position(double w1, double w2, double target_z) = 0;

        /**
         * @brief [核心修改] 计算切向量对 (偏导数)
         * 需要同时返回针对 w1 和 w2 的偏导数
         * first:  ∂p/∂w1 (对应 Controller 中的 df_dw1)
         * second: ∂p/∂w2 (对应 Controller 中的 df_dw2)
         */
        virtual std::pair<Eigen::Vector3d, Eigen::Vector3d> get_tangents(double w1, double w2) = 0;

        /**
         * @brief 获取曲线类型名称
         */
        virtual std::string get_type() const = 0;
    };

} // namespace distribute_control