#pragma once

#include <chrono>
#include <string>

#include "drone_swarm/types.hpp"

namespace drone_swarm {

/**
 * @brief Captures the observable state of a vehicle at a point in time.
 */
struct DroneState final {
    std::string identifier{};                   /**< Unique identifier for the drone. */
    VehicleStatus status{VehicleStatus::Idle};  /**< High-level lifecycle status. */
    GeodeticCoordinate location{};              /**< Current geodetic position. */
    double speed_mps{};                         /**< Current ground speed in metres per second. */
    double heading_deg{};                       /**< Current track heading in degrees. */
    double battery_percent{};                   /**< Remaining battery percentage. */
    double range_remaining_m{};                 /**< Estimated range remaining in metres. */
    TimePoint last_update_time{SteadyClock::now()}; /**< Timestamp of the most recent update. */
};

}  // namespace drone_swarm
