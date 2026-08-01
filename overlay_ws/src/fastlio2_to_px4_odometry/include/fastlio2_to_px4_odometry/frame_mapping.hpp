#pragma once

#include <array>
#include <cstddef>
#include <stdexcept>

#include <Eigen/Core>

namespace fastlio2_to_px4_odometry
{

struct FrameMapping
{
  std::array<int, 3> world_axis{1, 0, 2};
  std::array<double, 3> world_sign{1.0, 1.0, -1.0};
  std::array<int, 3> body_axis{0, 1, 2};
  std::array<double, 3> body_sign{1.0, -1.0, -1.0};

  static FrameMapping default_enu_world_flu_body_to_ned_world_frd_body()
  {
    return FrameMapping{};
  }

  Eigen::Matrix3d world_transform() const
  {
    return make_transform(world_axis, world_sign);
  }

  Eigen::Matrix3d body_transform() const
  {
    return make_transform(body_axis, body_sign);
  }

private:
  static Eigen::Matrix3d make_transform(
    const std::array<int, 3> & axis,
    const std::array<double, 3> & sign)
  {
    Eigen::Matrix3d transform = Eigen::Matrix3d::Zero();
    for (std::size_t row = 0; row < 3; ++row) {
      const int column = axis[row];
      if (column < 0 || column > 2) {
        throw std::out_of_range("frame mapping axis index must be in [0, 2]");
      }
      transform(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column)) = sign[row];
    }
    return transform;
  }
};

}  // namespace fastlio2_to_px4_odometry
