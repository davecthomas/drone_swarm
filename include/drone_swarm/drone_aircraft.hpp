#pragma once

#include "drone_swarm/autonomous_vehicle.hpp"

namespace drone_swarm {

/**
 * @brief Describes the safe operating limits for a drone airframe.
 */
struct FlightEnvelope final {
    double max_altitude_m{};        /**< Highest permitted altitude in metres. */
    double min_altitude_m{};        /**< Lowest permitted altitude in metres. */
    double max_climb_rate_mps{};    /**< Maximum climb rate in m/s. */
    double max_descent_rate_mps{};  /**< Maximum descent rate in m/s. */
    double max_bank_angle_deg{};    /**< Maximum bank angle in degrees. */
};

/**
 * @brief Concrete autonomous vehicle that enforces flight-specific constraints.
 */
class DroneAircraft final : public AutonomousVehicle {
  public:
    /**
     * @brief Construct a drone aircraft with the provided airframe constraints.
     */
    DroneAircraft(
        std::string identifier,
        VehicleKinematics kinematics,
        FlightEnvelope envelope,
        GeodeticCoordinate initial_location,
        GeodeticCoordinate base_location,
        PowertrainModel powertrain_model,
        double battery_capacity_wh,
        double consumption_wh_per_m,
        CameraFeedList camera_feeds
    );

  protected:
    /**
     * @brief Clamp flight dynamics to the configured envelope each tick.
     */
    void apply_vehicle_specific_constraints(DroneState& mutable_state, const Duration& tick) override;

  private:
    FlightEnvelope envelope_; /**< Envelope used to limit altitude and rates. */
};

using DroneAircraftPtr = std::shared_ptr<DroneAircraft>;

}  // namespace drone_swarm
