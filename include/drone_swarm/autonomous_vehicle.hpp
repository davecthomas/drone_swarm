#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drone_swarm/camera_feed.hpp"
#include "drone_swarm/drone_state.hpp"
#include "drone_swarm/telemetry_bus.hpp"
#include "drone_swarm/types.hpp"

namespace drone_swarm {

/**
 * @brief Encapsulates the dynamic limits used by navigation and flight planning.
 */
struct VehicleKinematics final {
    double max_speed_mps{};           /**< Maximum sustainable speed in m/s. */
    double max_acceleration_mps2{};   /**< Maximum acceleration in m/s². */
    double turn_rate_deg_per_s{};     /**< Maximum turn rate in degrees per second. */
};

/**
 * @brief Base class implementing shared behaviour for autonomous vehicles.
 */
class AutonomousVehicle : public std::enable_shared_from_this<AutonomousVehicle> {
  public:
    /**
     * @brief Construct a vehicle with the provided kinematics and power model.
     *
     * @param identifier Unique name for the vehicle.
     * @param kinematics Dynamic limits that govern motion planning.
     * @param initial_location Starting position of the vehicle.
     * @param base_location Reference location used for return-to-base logic.
     * @param powertrain_model Parameters describing reserve requirements.
     * @param battery_capacity_wh Total battery capacity in watt-hours.
     * @param consumption_wh_per_m Energy consumed per metre travelled.
     * @param camera_feeds Sensor feeds mounted on the platform.
     */
    AutonomousVehicle(
        std::string identifier,
        VehicleKinematics kinematics,
        GeodeticCoordinate initial_location,
        GeodeticCoordinate base_location,
        PowertrainModel powertrain_model,
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
    double calculate_return_reserve_percent() const;
    void initiate_low_power_return();
    void ensure_return_waypoint();

    std::string str_identifier_;
    VehicleKinematics kinematics_;
    GeodeticCoordinate base_location_;
    PowertrainModel powertrain_model_;
    double battery_capacity_wh_;
    double consumption_wh_per_m_;
    CameraFeedList camera_feeds_;
    std::vector<Waypoint> list_waypoints_;
    DroneState struct_state_;
};

using AutonomousVehiclePtr = std::shared_ptr<AutonomousVehicle>;
using AutonomousVehicleList = std::vector<AutonomousVehiclePtr>;

}  // namespace drone_swarm
