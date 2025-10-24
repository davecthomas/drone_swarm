#include <catch2/catch.hpp>

#include "drone_swarm/autonomous_vehicle.hpp"
#include "drone_swarm/telemetry_bus.hpp"

using namespace drone_swarm;

TEST_CASE("AutonomousVehicle publishes telemetry with updated state") {
    VehicleKinematics kinematics{};
    kinematics.max_speed_mps = 10.0;
    kinematics.max_acceleration_mps2 = 10.0;
    kinematics.turn_rate_deg_per_s = 5.0;

    CameraFeedList camera_feeds{};
    auto vehicle = std::make_shared<AutonomousVehicle>(
        "vehicle-test",
        kinematics,
        GeodeticCoordinate{0.0, 0.0, 0.0},
        100.0,
        0.1,
        camera_feeds
    );

    std::vector<Waypoint> waypoints{};
    waypoints.emplace_back(Waypoint{"wp", GeodeticCoordinate{0.0, 0.1, 0.0}, 0.0});
    vehicle->set_waypoints(waypoints);

    TelemetryBus bus{};
    vehicle->update(Duration{1.0}, bus);
    const auto event = bus.try_consume();

    REQUIRE(event.has_value());
    REQUIRE(event->state.identifier == "vehicle-test");
    REQUIRE(event->state.status != VehicleStatus::Idle);
}
