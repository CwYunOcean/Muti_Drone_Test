#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace distribute_control
{

inline std::string dronePrefix(int drone_id)
{
    if (drone_id < 1)
    {
        throw std::invalid_argument("drone_id must be 1-based");
    }
    return "/drone_" + std::to_string(drone_id);
}

inline std::string fmuInPrefix(int drone_id)
{
    return dronePrefix(drone_id) + "/fmu/in";
}

inline std::string fmuOutPrefix(int drone_id)
{
    return dronePrefix(drone_id) + "/fmu/out";
}

struct FormationGridOffset
{
    double x;
    double y;
};

inline FormationGridOffset formationGridOffset(int drone_id, int total_uavs, double spacing)
{
    if (drone_id < 1 || total_uavs < 1 || drone_id > total_uavs)
    {
        throw std::invalid_argument("drone_id must be within the formation size");
    }
    const int columns = std::max(1, static_cast<int>(std::ceil(std::sqrt(total_uavs))));
    const int rows = static_cast<int>(std::ceil(static_cast<double>(total_uavs) / columns));
    const int row = (drone_id - 1) / columns;
    const int column = (drone_id - 1) % columns;
    return {column * spacing - (columns - 1) * spacing / 2.0,
            row * spacing - (rows - 1) * spacing / 2.0};
}

}  // namespace distribute_control
