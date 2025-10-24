#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drone_swarm/camera_feed.hpp"
#include "drone_swarm/drone_state.hpp"
#include "drone_swarm/telemetry_bus.hpp"

namespace drone_swarm {

struct VehicleKinematics final {
    double max_speed_mps{};
    double max_acceleration_mps2{};
    double turn_rate_deg_per_s{};
};

class AutonomousVehicle : public std::enable_shared_from_this<AutonomousVehicle> {
  public:
    AutonomousVehicle(
        std::string identifier,
        VehicleKinematics kinematics,
        GeodeticCoordinate initial_location,
        double battery_capacity_wh,
        double consumption_wh_per_m,
        CameraFeedList camera_feeds
    );

    virtual ~AutonomousVehicle() = default;

    [[nodiscard]] const std::string& identifier() const noexcept;
    [[nodiscard]] const DroneState& state() const noexcept;
    [[nodiscard]] const VehicleKinematics& kinematics() const noexcept;
    [[nodiscard]] double battery_capacity_wh() const noexcept;
    [[nodiscard]] const CameraFeedList& camera_feeds() const noexcept;

    void set_waypoints(std::vector<Waypoint> waypoints);
    void update(const Duration& tick, TelemetryBus& bus);

  protected:
    virtual void apply_vehicle_specific_constraints(DroneState& mutable_state, const Duration& tick);

  private:
    void update_navigation(const Duration& tick);
    void update_battery(const Duration& tick);
    void publish_state(TelemetryBus& bus);
    void maintain_collision_avoidance();

    std::string str_identifier_;
    VehicleKinematics kinematics_;
    double battery_capacity_wh_;
    double consumption_wh_per_m_;
    CameraFeedList camera_feeds_;
    std::vector<Waypoint> list_waypoints_;
    DroneState struct_state_;
};

using AutonomousVehiclePtr = std::shared_ptr<AutonomousVehicle>;
using AutonomousVehicleList = std::vector<AutonomousVehiclePtr>;

}  // namespace drone_swarm
