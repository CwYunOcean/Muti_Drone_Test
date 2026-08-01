#include "distribute_control/core/orca_barrier.hpp"
#include <iostream>

// 辅助函数：计算行列式
static double det(const Eigen::Vector2d &v1, const Eigen::Vector2d &v2)
{
    return v1.x() * v2.y() - v1.y() * v2.x();
}

namespace distribute_control
{

    ORCABarrier::ORCABarrier() {}

    void ORCABarrier::set_params(const ORCAParams &params)
    {
        params_ = params;
    }

    Eigen::Vector3d ORCABarrier::compute_safe_velocity(
        const Eigen::Vector2d &curr_pos,
        const Eigen::Vector2d &curr_vel,
        double curr_yaw, // 未使用 (Holonomic假设)
        const Eigen::Vector2d &goal_pos,
        const std::vector<Eigen::Vector2d> &neighbors_pos,
        const std::vector<Eigen::Vector2d> &neighbors_vel,
        const std::vector<double> &neighbors_yaw, // 未使用
        double radius_i,
        const std::vector<double> &radius_neighbors)
    {
        // 1. 计算偏好速度 (Pref Velocity)
        Eigen::Vector2d dist_vec = goal_pos - curr_pos;
        double dist = dist_vec.norm();
        Eigen::Vector2d pref_vel;

        if (dist > 1.0)
        {
            pref_vel = dist_vec.normalized() * params_.max_speed;
        }
        else
        {
            pref_vel = dist_vec / 1.0 * params_.max_speed; // 简单的比例减速
        }

        // 2. 构建 ORCA 约束线
        std::vector<Line> orca_lines;
        double invTimeHorizon = 1.0 / params_.time_horizon;

        size_t num_neighbors = neighbors_pos.size();

        for (size_t i = 0; i < num_neighbors; ++i)
        {
            // 相对状态
            Eigen::Vector2d relative_pos = neighbors_pos[i] - curr_pos;
            Eigen::Vector2d relative_vel = curr_vel - neighbors_vel[i];

            double distSq = relative_pos.squaredNorm();
            // 组合半径 (自身 + 邻居 + 安全缓冲)
            double combined_radius = radius_i + radius_neighbors[i] + params_.safe_dist;
            double combined_radius_sq = combined_radius * combined_radius;

            Line line;
            Eigen::Vector2d u; // 避让向量

            // 几何构建 VO
            if (distSq > combined_radius_sq)
            {
                // 情况 A: 尚未发生碰撞
                Eigen::Vector2d w = relative_vel - invTimeHorizon * relative_pos;
                double wLengthSq = w.squaredNorm();
                double dotProduct1 = w.dot(relative_pos);

                if (dotProduct1 < 0.0f && (dotProduct1 * dotProduct1) > combined_radius_sq * wLengthSq)
                {
                    // 已经在 Cutoff 圆内 (Project on cut-off circle)
                    double wLength = std::sqrt(wLengthSq);
                    Eigen::Vector2d unitW = w / wLength;

                    line.direction = Eigen::Vector2d(unitW.y(), -unitW.x());
                    u = (combined_radius * invTimeHorizon - wLength) * unitW;
                }
                else
                {
                    // 在 VO 锥体腿上 (Project on legs)
                    double leg = std::sqrt(distSq - combined_radius_sq);

                    if (det(relative_pos, w) > 0.0f) // Left leg
                    {
                        line.direction = Eigen::Vector2d(
                                             relative_pos.x() * leg - relative_pos.y() * combined_radius,
                                             relative_pos.x() * combined_radius + relative_pos.y() * leg) /
                                         distSq;
                    }
                    else // Right leg
                    {
                        line.direction = -Eigen::Vector2d(
                                             relative_pos.x() * leg + relative_pos.y() * combined_radius,
                                             -relative_pos.x() * combined_radius + relative_pos.y() * leg) /
                                         distSq;
                    }

                    double dotProduct2 = relative_vel.dot(line.direction);
                    u = dotProduct2 * line.direction - relative_vel;
                }
            }
            else
            {
                // 情况 B: 已经发生碰撞 (Collision)
                // 此时时间窗无限小，尽快逃逸
                double invTimeStep = 1.0 / 0.05; // 假设 dt = 0.05 或其他小数值
                Eigen::Vector2d w = relative_vel - invTimeStep * relative_pos;
                double wLength = w.norm();
                Eigen::Vector2d unitW = w / wLength;

                line.direction = Eigen::Vector2d(unitW.y(), -unitW.x());
                u = (combined_radius * invTimeStep - wLength) * unitW;
            }

            // 构建约束线: point 是这一侧半平面的边界点
            // Reciprocity: 承担一半的避让责任 (0.5 * u)
            line.point = curr_vel + 0.5f * u;
            orca_lines.push_back(line);
        }

        // 3. 线性规划求解
        Eigen::Vector2d new_vel = curr_vel;

        // linearProgram2 会在无解时调用 linearProgram3
        size_t lineFail = linearProgram2(orca_lines, params_.max_speed, pref_vel, false, new_vel);

        if (lineFail < orca_lines.size())
        {
            linearProgram3(orca_lines, lineFail, params_.max_speed, new_vel, true, new_vel);
        }

        return Eigen::Vector3d(new_vel.x(), new_vel.y(), 0.0);
    }

    // ==========================================================
    // 标准线性规划实现 (Reference: RVO2 Library)
    // ==========================================================

    bool ORCABarrier::linearProgram1(const std::vector<Line> &lines, size_t lineNo,
                                     double radius, const Eigen::Vector2d &optVelocity,
                                     bool directionOpt, Eigen::Vector2d &result)
    {
        const double dotProduct = lines[lineNo].point.dot(lines[lineNo].direction);
        const double discriminant = dotProduct * dotProduct + radius * radius - lines[lineNo].point.squaredNorm();

        if (discriminant < 0.0f)
        {
            return false; // Max speed check failed
        }

        const double sqrtDiscriminant = std::sqrt(discriminant);
        double tLeft = -dotProduct - sqrtDiscriminant;
        double tRight = -dotProduct + sqrtDiscriminant;

        for (size_t i = 0; i < lineNo; ++i)
        {
            const double denominator = det(lines[lineNo].direction, lines[i].direction);
            const double numerator = det(lines[i].direction, lines[lineNo].point - lines[i].point);

            if (std::abs(denominator) <= 1.0e-6) // Parallel lines
            {
                if (numerator < 0.0f)
                    return false;
            }
            else
            {
                const double t = numerator / denominator;
                if (denominator > 0.0f)
                    tRight = std::min(tRight, t);
                else
                    tLeft = std::max(tLeft, t);
            }

            if (tLeft > tRight)
                return false;
        }

        if (directionOpt)
        {
            // Optimize direction only
            if (optVelocity.dot(lines[lineNo].direction) > 0.0f)
            {
                result = lines[lineNo].point + tRight * lines[lineNo].direction;
            }
            else
            {
                result = lines[lineNo].point + tLeft * lines[lineNo].direction;
            }
        }
        else
        {
            // Optimize closest point
            const double t = lines[lineNo].direction.dot(optVelocity - lines[lineNo].point);
            if (t < tLeft)
                result = lines[lineNo].point + tLeft * lines[lineNo].direction;
            else if (t > tRight)
                result = lines[lineNo].point + tRight * lines[lineNo].direction;
            else
                result = lines[lineNo].point + t * lines[lineNo].direction;
        }

        return true;
    }

    size_t ORCABarrier::linearProgram2(const std::vector<Line> &lines, double radius,
                                       const Eigen::Vector2d &optVelocity, bool directionOpt,
                                       Eigen::Vector2d &result)
    {
        if (directionOpt)
        {
            result = optVelocity * radius;
        }
        else if (optVelocity.squaredNorm() > radius * radius)
        {
            result = optVelocity.normalized() * radius;
        }
        else
        {
            result = optVelocity;
        }

        for (size_t i = 0; i < lines.size(); ++i)
        {
            if (det(lines[i].direction, lines[i].point - result) > 0.0f)
            {
                // Result does not satisfy constraint i, compute new optimal result
                Eigen::Vector2d tempResult = result;
                if (!linearProgram1(lines, i, radius, optVelocity, directionOpt, result))
                {
                    result = tempResult;
                    return i; // Failed at line i
                }
            }
        }
        return lines.size();
    }

    void ORCABarrier::linearProgram3(const std::vector<Line> &lines, size_t numObstLines,
                                     double radius, const Eigen::Vector2d &optVelocity,
                                     bool directionOpt, Eigen::Vector2d &result)
    {
        double distance = 0.0f;

        for (size_t i = numObstLines; i < lines.size(); ++i)
        {
            if (det(lines[i].direction, lines[i].point - result) > distance)
            {
                // Result does not satisfy constraint of line i
                std::vector<Line> projLines;
                for (size_t j = 0; j < numObstLines; ++j)
                {
                    projLines.push_back(lines[j]);
                }

                for (size_t j = numObstLines; j < i; ++j)
                {
                    Line line;
                    double determinant = det(lines[i].direction, lines[j].direction);
                    if (std::abs(determinant) <= 1.0e-6)
                    {
                        if (lines[i].direction.dot(lines[j].direction) > 0.0f)
                            continue;
                        else
                            line.point = 0.5f * (lines[i].point + lines[j].point);
                    }
                    else
                    {
                        line.point = lines[i].point + (det(lines[j].direction, lines[i].point - lines[j].point) / determinant) * lines[i].direction;
                    }

                    line.direction = (lines[j].direction - lines[i].direction).normalized();
                    projLines.push_back(line);
                }

                const Eigen::Vector2d tempResult = result;
                if (linearProgram2(projLines, radius, Eigen::Vector2d(-lines[i].direction.y(), lines[i].direction.x()), true, result) < projLines.size())
                {
                    result = tempResult;
                }
                distance = det(lines[i].direction, lines[i].point - result);
            }
        }
    }

} // namespace distribute_control