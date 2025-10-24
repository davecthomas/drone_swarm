#pragma once

#include <chrono>
#include <string>

namespace drone_swarm {

using SteadyClock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<SteadyClock>;
using Duration = std::chrono::duration<double>;

struct GeodeticCoordinate final {
    double latitude_deg{};
    double longitude_deg{};
    double altitude_m{};
};

struct Waypoint final {
    std::string name{};
    GeodeticCoordinate location{};
    double loiter_seconds{};
};

enum class VehicleStatus {
    Idle,
    EnRoute,
    Holding,
    ReturningHome,
    Charging,
    Faulted
};

enum class TileProviderType {
    Mapbox,
    MapLibreDemo
};

}  // namespace drone_swarm
