#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <iostream>

namespace distribute_control {

/**
 * @brief 曲线抽象基类 (Interface)
 * 所有具体的轨迹形状（圆形、8字形、矩形）都必须继承此类
 */
class CurveBase {
public:
    // 虚析构函数，确保派生类能被正确销毁
    virtual ~CurveBase() = default;

    /**
     * @brief 加载参数
     * @param params 从 YAML 读取的参数列表 [a, b, n, ...]
     */
    virtual void load_params(const std::vector<double>& params) = 0;

    /**
     * @brief 计算期望位置
     * @param w 虚拟路径参数 (弧度或路径长度)
     * @param height 期望飞行高度
     * @return Eigen::Vector3d 期望位置 (x_d, y_d, z_d)
     */
    virtual Eigen::Vector3d get_position(double w, double height) = 0;

    /**
     * @brief 计算切向量 (速度前馈)
     * 即位置对 w 的导数: dP/dw
     * @param w 虚拟路径参数
     * @return Eigen::Vector3d 切向速度向量 (vx, vy, vz)
     */
    virtual Eigen::Vector3d get_tangent(double w) = 0;

    /**
     * @brief 获取曲线类型名称 (用于调试)
     */
    virtual std::string get_type() const = 0;
};

} // namespace distribute_control