#include "distribute_control/curves/bspline_curve.hpp"
#include <iostream>

namespace distribute_control
{

    // [新增] 启动订阅逻辑
    // 传入 Node 指针，让曲线对象能够“寄生”在节点上接收数据
    void BsplineCurve::start_subscription(rclcpp::Node *node)
    {
        // 使用 Transient Local QoS，确保能收到最后一次发布的地图 (哪怕发布者先发布，订阅者后启动)
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();

        sub_waypoints_ = node->create_subscription<geometry_msgs::msg::Polygon>(
            "/global/curve_points", qos,
            [this](const geometry_msgs::msg::Polygon::SharedPtr msg)
            {
                // 1. 解析消息: ROS msg -> Eigen Vector
                std::vector<Eigen::Vector2d> new_pts;
                for (const auto &p : msg->points)
                {
                    new_pts.push_back(Eigen::Vector2d(p.x, p.y));
                }

                // 2. 执行拟合计算
                if (new_pts.size() >= 3)
                {
                    // 这里我们收到的是航路点，需要反求控制点
                    this->fit_waypoints_and_update(new_pts);

                    // 打印调试信息 (可选)
                    // std::cout << "[BsplineCurve] Curve updated via Topic. Points: " << new_pts.size() << std::endl;
                }
            });
    }

    // [新增] 拟合算法 (解方程 A*P = 6*D)
    // 核心数学：已知曲线经过点 D (waypoints)，反求 B样条控制点 P (control_points_)
    void BsplineCurve::fit_waypoints_and_update(const std::vector<Eigen::Vector2d> &waypoints)
    {
        int n = waypoints.size();
        if (n < 3)
            return;

        // 1. 构建系数矩阵 A (n x n) - 闭合均匀 B 样条的系数矩阵是循环三对角矩阵
        // [4 1 0 ... 1]
        // [1 4 1 ... 0]
        // ...
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
        for (int i = 0; i < n; ++i)
        {
            A(i, i) = 4.0;
            A(i, (i - 1 + n) % n) = 1.0; // 左邻
            A(i, (i + 1) % n) = 1.0;     // 右邻
        }

        // 2. 构建常数项 B (n x 2)
        // 公式推导结果为: A * P = 6 * Waypoints
        Eigen::MatrixXd B(n, 2);
        for (int i = 0; i < n; ++i)
        {
            B(i, 0) = waypoints[i].x() * 6.0;
            B(i, 1) = waypoints[i].y() * 6.0;
        }

        // 3. 求解线性方程组 A * P = B
        // 使用 ColPivHouseholderQR 求解，比求逆更稳定
        Eigen::MatrixXd P = A.colPivHouseholderQr().solve(B);

        // 4. 更新内部参数
        control_points_.clear();
        for (int i = 0; i < n; ++i)
        {
            control_points_.push_back(Eigen::Vector2d(P(i, 0), P(i, 1)));
        }
        num_points_ = control_points_.size();
        valid_ = true;
    }

    // --- 接口实现 ---

    void BsplineCurve::load_params(const std::vector<double> &params)
    {
        // params 结构: [x0, y0, x1, y1, x2, y2, ...]
        if (params.size() >= 6 && params.size() % 2 == 0)
        {
            std::vector<Eigen::Vector2d> initial_waypoints;
            for (size_t i = 0; i < params.size(); i += 2)
            {
                initial_waypoints.push_back(Eigen::Vector2d(params[i], params[i + 1]));
            }

            // 为了保持逻辑一致性，我们将 YAML 里的参数也视为“必须经过的航点”
            // 因此同样调用拟合函数
            fit_waypoints_and_update(initial_waypoints);
        }
        else
        {
            // 如果参数无效，valid_ 保持 false，OffboardNode 会处理
            valid_ = false;
        }
    }

    // 辅助函数：安全获取闭环点 (处理负数和越界)
    Eigen::Vector2d BsplineCurve::get_pt(int idx) const
    {
        int n = num_points_;
        // ((idx % n) + n) % n 确保结果总是正数索引 [0, n-1]
        int true_idx = ((idx % n) + n) % n;
        return control_points_[true_idx];
    }

    // 计算曲线位置 P(w)
    Eigen::Vector3d BsplineCurve::get_position(double w, double height)
    {
        if (!valid_)
            return Eigen::Vector3d(0, 0, height);

        // 1. 将 w [0, 2pi] 映射到 索引空间 [0, N]
        double t_total = (w / (2.0 * M_PI)) * num_points_;
        int i = std::floor(t_total);
        double t = t_total - i; // 小数部分 t (0 <= t < 1)

        // 2. 获取相关的4个控制点 (i-1, i, i+1, i+2)
        Eigen::Vector2d p0 = get_pt(i - 1);
        Eigen::Vector2d p1 = get_pt(i);
        Eigen::Vector2d p2 = get_pt(i + 1);
        Eigen::Vector2d p3 = get_pt(i + 2);

        // 3. 均匀三次B样条基函数
        double t2 = t * t;
        double t3 = t2 * t;

        // 基函数系数
        double b0 = (-t3 + 3 * t2 - 3 * t + 1) / 6.0;
        double b1 = (3 * t3 - 6 * t2 + 4) / 6.0;
        double b2 = (-3 * t3 + 3 * t2 + 3 * t + 1) / 6.0;
        double b3 = t3 / 6.0;

        // 线性组合
        double x = b0 * p0.x() + b1 * p1.x() + b2 * p2.x() + b3 * p3.x();
        double y = b0 * p0.y() + b1 * p1.y() + b2 * p2.y() + b3 * p3.y();

        return Eigen::Vector3d(x, y, height);
    }

    // 计算曲线切线 (速度方向) dP/dw
    Eigen::Vector3d BsplineCurve::get_tangent(double w)
    {
        if (!valid_)
            return Eigen::Vector3d(0, 0, 0);

        double t_total = (w / (2.0 * M_PI)) * num_points_;
        int i = std::floor(t_total);
        double t = t_total - i;

        Eigen::Vector2d p0 = get_pt(i - 1);
        Eigen::Vector2d p1 = get_pt(i);
        Eigen::Vector2d p2 = get_pt(i + 1);
        Eigen::Vector2d p3 = get_pt(i + 2);

        double t2 = t * t;

        // 基函数对 t 的导数
        double db0 = (-3 * t2 + 6 * t - 3) / 6.0;
        double db1 = (9 * t2 - 12 * t) / 6.0;
        double db2 = (-9 * t2 + 6 * t + 3) / 6.0;
        double db3 = 3 * t2 / 6.0;

        // 计算 dP/dt
        double dx_dt = db0 * p0.x() + db1 * p1.x() + db2 * p2.x() + db3 * p3.x();
        double dy_dt = db0 * p0.y() + db1 * p1.y() + db2 * p2.y() + db3 * p3.y();

        // 链式法则：dP/dw = dP/dt * dt/dw
        // dt/dw = N / (2*pi)
        double dt_dw = (double)num_points_ / (2.0 * M_PI);

        return Eigen::Vector3d(dx_dt * dt_dw, dy_dt * dt_dw, 0.0);
    }

}