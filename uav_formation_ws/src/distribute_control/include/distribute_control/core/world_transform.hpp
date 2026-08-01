#pragma once

#include <Eigen/Dense>

#include <cmath>

namespace distribute_control
{

class WorldTransform
{
public:
    WorldTransform(const Eigen::Vector3d &origin_xyz = Eigen::Vector3d::Zero(),
                   double origin_yaw_rad = 0.0)
        : origin_xyz_(origin_xyz),
          rotation_(Eigen::AngleAxisd(origin_yaw_rad, Eigen::Vector3d::UnitZ()))
    {
    }

    Eigen::Vector3d positionFromPx4Ned(const Eigen::Vector3d &ned_position) const
    {
        return rotation_ * nedToEnu(ned_position) + origin_xyz_;
    }

    Eigen::Vector3d velocityFromPx4Ned(const Eigen::Vector3d &ned_velocity) const
    {
        return rotation_ * nedToEnu(ned_velocity);
    }

    Eigen::Vector3d positionToPx4Ned(const Eigen::Vector3d &world_position) const
    {
        return enuToNed(rotation_.transpose() * (world_position - origin_xyz_));
    }

    Eigen::Vector3d velocityToPx4Ned(const Eigen::Vector3d &world_velocity) const
    {
        return enuToNed(rotation_.transpose() * world_velocity);
    }

private:
    static Eigen::Vector3d nedToEnu(const Eigen::Vector3d &ned)
    {
        return Eigen::Vector3d(ned.y(), ned.x(), -ned.z());
    }

    static Eigen::Vector3d enuToNed(const Eigen::Vector3d &enu)
    {
        return Eigen::Vector3d(enu.y(), enu.x(), -enu.z());
    }

    Eigen::Vector3d origin_xyz_;
    Eigen::Matrix3d rotation_;
};

}  // namespace distribute_control
