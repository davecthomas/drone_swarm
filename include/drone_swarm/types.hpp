// === Core Types ==============================================================
//
// Collects shared type aliases and lightweight structs/enums used throughout
// the simulator (time primitives, geodetic coordinates, power models, etc.).

#pragma once

#include <chrono>
#include <string>

namespace drone_swarm {

/**
 * @brief Alias for the steady clock used across the simulation.
 */
using SteadyClock = std::chrono::steady_clock;

/**
 * @brief Alias for timestamps captured from the steady clock.
 */
using TimePoint = std::chrono::time_point<SteadyClock>;

/**
 * @brief Alias for durations measured in seconds with double precision.
 */
using Duration = std::chrono::duration<double>;

/**
 * @brief Represents a latitude/longitude/altitude triplet in degrees/metres.
 */
struct GeodeticCoordinate final {
    double latitude_deg{};   /**< Latitude in decimal degrees. */
    double longitude_deg{};  /**< Longitude in decimal degrees. */
    double altitude_m{};     /**< Altitude in metres above mean sea level. */
};

/**
 * @brief Encapsulates a navigational waypoint and optional loiter period.
 */
struct Waypoint final {
    std::string name{};          /**< Human-readable waypoint identifier. */
    GeodeticCoordinate location; /**< Geodetic position of the waypoint. */
    double loiter_seconds{};     /**< Time to loiter at the waypoint in seconds. */
};

/**
 * @brief Enumerates the possible lifecycle states for autonomous vehicles.
 */
enum class VehicleStatus {
    Idle,             /**< Vehicle is inactive and awaiting orders. */
    EnRoute,          /**< Vehicle is actively travelling toward a waypoint. */
    Holding,          /**< Vehicle is stationary or loitering on station. */
    ReturningHome,    /**< Vehicle is returning to base after completing its mission. */
    LowPowerReturn,   /**< Vehicle is returning to base due to projected power shortfall. */
    Charging,         /**< Vehicle is undergoing recharge operations. */
    Faulted           /**< Vehicle encountered a critical fault. */
};

/**
 * @brief Describes the power demand characteristics used for return-to-base calculations.
 */
struct PowertrainModel final {
    double base_reserve_percent{};              /**< Minimum battery percentage reserved for contingencies. */
    double reserve_percent_per_meter{};         /**< Additional reserve percentage required per metre of outbound distance. */
    double reserve_percent_per_second{};        /**< Additional reserve percentage consumed per second of anticipated return flight. */
    double minimum_return_speed_mps{};          /**< Lower bound for return-speed calculations to avoid divide-by-zero. */
};

/**
 * @brief Identifies which tile provider the renderer should employ.
 */
enum class TileProviderType {
    Mapbox,        /**< Use Mapbox-hosted vector/satellite tiles. */
    MapLibreDemo   /**< Use the open MapLibre demo tile source. */
};

}  // namespace drone_swarm
