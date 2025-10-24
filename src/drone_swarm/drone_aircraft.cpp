#include "drone_swarm/drone_aircraft.hpp"

#include <algorithm>
#include <stdexcept>

#include "drone_swarm/logging.hpp"

namespace drone_swarm {

DroneAircraft::DroneAircraft(
    std::string identifier,
    VehicleKinematics kinematics,
    FlightEnvelope envelope,
    GeodeticCoordinate initial_location,
    double battery_capacity_wh,
    double consumption_wh_per_m,
    CameraFeedList camera_feeds
)
    : AutonomousVehicle(
          std::move(identifier),
          kinematics,
          initial_location,
          battery_capacity_wh,
          consumption_wh_per_m,
          std::move(camera_feeds)
      ),
      envelope_(envelope) {
    if (envelope_.max_altitude_m <= envelope_.min_altitude_m) {
        throw std::invalid_argument("Flight envelope altitude bounds are invalid");
    }
}

void DroneAircraft::apply_vehicle_specific_constraints(DroneState& mutable_state, const Duration& tick) {
    const double desired_altitude = envelope_.min_altitude_m + (envelope_.max_altitude_m - envelope_.min_altitude_m) * 0.5;
    const double altitude_error = desired_altitude - mutable_state.location.altitude_m;
    const double max_delta = envelope_.max_climb_rate_mps * tick.count();
    const double limited_delta = std::clamp(altitude_error, -max_delta, max_delta);
    mutable_state.location.altitude_m = std::clamp(
        mutable_state.location.altitude_m + limited_delta,
        envelope_.min_altitude_m,
        envelope_.max_altitude_m
    );

    if (mutable_state.battery_percent <= 5.0) {
        mutable_state.status = VehicleStatus::ReturningHome;
        auto logger = get_logger();
        logger->warn("Drone {} battery critically low; forcing return", mutable_state.identifier);
    }
}

}  // namespace drone_swarm
