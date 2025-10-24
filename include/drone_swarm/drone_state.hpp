#pragma once

#include <chrono>
#include <string>

#include "drone_swarm/types.hpp"

namespace drone_swarm {

struct DroneState final {
    std::string identifier{};
    VehicleStatus status{VehicleStatus::Idle};
    GeodeticCoordinate location{};
    double speed_mps{};
    double heading_deg{};
    double battery_percent{};
    double range_remaining_m{};
    TimePoint last_update_time{SteadyClock::now()};
};

}  // namespace drone_swarm
