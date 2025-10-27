#include "drone_swarm/drone_aircraft.hpp"

#include <algorithm>
#include <stdexcept>

#include "drone_swarm/logging.hpp"

namespace drone_swarm {

namespace {
constexpr double k_altitude_hold_ratio{0.5}; /**< Fraction of envelope span used as nominal cruise altitude. */
}

DroneAircraft::DroneAircraft(
    std::string identifier,
    VehicleKinematics kinematics,
    FlightEnvelope envelope,
    GeodeticCoordinate initial_location,
    GeodeticCoordinate base_location,
    PowertrainModel powertrain_model,
    double battery_capacity_wh,
    double consumption_wh_per_m,
    CameraFeedList camera_feeds
)
    : AutonomousVehicle(
          std::move(identifier),
          kinematics,
          initial_location,
          base_location,
          powertrain_model,
          battery_capacity_wh,
          consumption_wh_per_m,
          std::move(camera_feeds)
      ),
      envelope_(envelope) {
    if (envelope_.max_altitude_m <= envelope_.min_altitude_m) {
        throw std::invalid_argument("Flight envelope altitude bounds are invalid");
    }
}

/**
 * @brief Clamp altitude changes so the drone stays within its certified envelope.
 */
void DroneAircraft::apply_vehicle_specific_constraints(DroneState& mutable_state, const Duration& tick) {
    const double desired_altitude = envelope_.min_altitude_m + (envelope_.max_altitude_m - envelope_.min_altitude_m) * k_altitude_hold_ratio;
    const double altitude_error = desired_altitude - mutable_state.location.altitude_m;
    const double max_delta = envelope_.max_climb_rate_mps * tick.count();
    const double limited_delta = std::clamp(altitude_error, -max_delta, max_delta);
    mutable_state.location.altitude_m = std::clamp(
        mutable_state.location.altitude_m + limited_delta,
        envelope_.min_altitude_m,
        envelope_.max_altitude_m
    );
}

}  // namespace drone_swarm
