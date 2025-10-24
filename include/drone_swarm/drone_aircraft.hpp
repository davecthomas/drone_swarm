#pragma once

#include "drone_swarm/autonomous_vehicle.hpp"

namespace drone_swarm {

struct FlightEnvelope final {
    double max_altitude_m{};
    double min_altitude_m{};
    double max_climb_rate_mps{};
    double max_descent_rate_mps{};
    double max_bank_angle_deg{};
};

class DroneAircraft final : public AutonomousVehicle {
  public:
    DroneAircraft(
        std::string identifier,
        VehicleKinematics kinematics,
        FlightEnvelope envelope,
        GeodeticCoordinate initial_location,
        double battery_capacity_wh,
        double consumption_wh_per_m,
        CameraFeedList camera_feeds
    );

  protected:
    void apply_vehicle_specific_constraints(DroneState& mutable_state, const Duration& tick) override;

  private:
    FlightEnvelope envelope_;
};

using DroneAircraftPtr = std::shared_ptr<DroneAircraft>;

}  // namespace drone_swarm
