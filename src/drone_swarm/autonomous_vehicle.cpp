#include "drone_swarm/autonomous_vehicle.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

#include <spdlog/spdlog.h>

#include "drone_swarm/logging.hpp"

namespace drone_swarm {

namespace {
double degrees_to_radians(double degrees) {
    return degrees * std::numbers::pi / 180.0;
}

double radians_to_degrees(double radians) {
    return radians * 180.0 / std::numbers::pi;
}

GeodeticCoordinate advance_coordinate(const GeodeticCoordinate& start, double distance_m, double bearing_deg) {
    constexpr double k_earth_radius_m{6'371'000.0};
    const double angular_distance = distance_m / k_earth_radius_m;
    const double bearing_rad = degrees_to_radians(bearing_deg);
    const double lat_rad = degrees_to_radians(start.latitude_deg);
    const double lon_rad = degrees_to_radians(start.longitude_deg);

    const double new_lat = std::asin(
        std::sin(lat_rad) * std::cos(angular_distance) + std::cos(lat_rad) * std::sin(angular_distance) * std::cos(bearing_rad)
    );

    const double new_lon = lon_rad
        + std::atan2(
            std::sin(bearing_rad) * std::sin(angular_distance) * std::cos(lat_rad),
            std::cos(angular_distance) - std::sin(lat_rad) * std::sin(new_lat)
        );

    return GeodeticCoordinate{
        radians_to_degrees(new_lat),
        radians_to_degrees(new_lon),
        start.altitude_m
    };
}

double haversine_distance_m(const GeodeticCoordinate& from, const GeodeticCoordinate& to) {
    constexpr double k_earth_radius_m{6'371'000.0};
    const double lat1 = degrees_to_radians(from.latitude_deg);
    const double lat2 = degrees_to_radians(to.latitude_deg);
    const double delta_lat = lat2 - lat1;
    const double delta_lon = degrees_to_radians(to.longitude_deg - from.longitude_deg);

    const double a = std::pow(std::sin(delta_lat / 2.0), 2)
        + std::cos(lat1) * std::cos(lat2) * std::pow(std::sin(delta_lon / 2.0), 2);
    const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    return k_earth_radius_m * c;
}

double initial_bearing_deg(const GeodeticCoordinate& from, const GeodeticCoordinate& to) {
    const double lat1 = degrees_to_radians(from.latitude_deg);
    const double lat2 = degrees_to_radians(to.latitude_deg);
    const double delta_lon = degrees_to_radians(to.longitude_deg - from.longitude_deg);

    const double y = std::sin(delta_lon) * std::cos(lat2);
    const double x = std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(delta_lon);
    const double bearing_rad = std::atan2(y, x);
    const double bearing_deg = std::fmod(radians_to_degrees(bearing_rad) + 360.0, 360.0);
    return bearing_deg;
}

}  // namespace

AutonomousVehicle::AutonomousVehicle(
    std::string identifier,
    VehicleKinematics kinematics,
    GeodeticCoordinate initial_location,
    double battery_capacity_wh,
    double consumption_wh_per_m,
    CameraFeedList camera_feeds
)
    : str_identifier_(std::move(identifier)),
      kinematics_(kinematics),
      battery_capacity_wh_(battery_capacity_wh),
      consumption_wh_per_m_(consumption_wh_per_m),
      camera_feeds_(std::move(camera_feeds)) {
    if (str_identifier_.empty()) {
        throw std::invalid_argument("AutonomousVehicle identifier cannot be empty");
    }
    if (battery_capacity_wh_ <= 0.0) {
        throw std::invalid_argument("AutonomousVehicle battery capacity must be positive");
    }
    if (consumption_wh_per_m_ <= 0.0) {
        throw std::invalid_argument("AutonomousVehicle consumption must be positive");
    }
    struct_state_.identifier = str_identifier_;
    struct_state_.location = initial_location;
    struct_state_.heading_deg = 0.0;
    struct_state_.status = VehicleStatus::Idle;
    struct_state_.battery_percent = 100.0;
    struct_state_.range_remaining_m = (battery_capacity_wh_ / consumption_wh_per_m_);
}

const std::string& AutonomousVehicle::identifier() const noexcept {
    return str_identifier_;
}

const DroneState& AutonomousVehicle::state() const noexcept {
    return struct_state_;
}

const VehicleKinematics& AutonomousVehicle::kinematics() const noexcept {
    return kinematics_;
}

double AutonomousVehicle::battery_capacity_wh() const noexcept {
    return battery_capacity_wh_;
}

const CameraFeedList& AutonomousVehicle::camera_feeds() const noexcept {
    return camera_feeds_;
}

void AutonomousVehicle::set_waypoints(std::vector<Waypoint> waypoints) {
    list_waypoints_ = std::move(waypoints);
    auto logger = get_logger();
    logger->info("Vehicle {} loaded {} waypoints", str_identifier_, list_waypoints_.size());
}

void AutonomousVehicle::update(const Duration& tick, TelemetryBus& bus) {
    struct_state_.last_update_time = SteadyClock::now();

    if (list_waypoints_.empty()) {
        struct_state_.status = VehicleStatus::Holding;
        struct_state_.speed_mps = 0.0;
        publish_state(bus);
        return;
    }

    update_navigation(tick);
    update_battery(tick);
    maintain_collision_avoidance();
    apply_vehicle_specific_constraints(struct_state_, tick);

    for (const CameraFeedPtr& camera_feed : camera_feeds_) {
        if (camera_feed == nullptr) {
            continue;
        }
        camera_feed->update_pose(struct_state_.location, Orientation{0.0, struct_state_.heading_deg, 0.0});
    }

    publish_state(bus);
}

void AutonomousVehicle::apply_vehicle_specific_constraints(DroneState&, const Duration&) {
    // Base class does not impose additional constraints.
}

void AutonomousVehicle::update_navigation(const Duration& tick) {
    auto logger = get_logger();
    Waypoint& next_waypoint = list_waypoints_.front();
    const double delta_seconds = tick.count();

    const double distance_to_waypoint_m = haversine_distance_m(struct_state_.location, next_waypoint.location);
    if (distance_to_waypoint_m <= 1.0) {
        struct_state_.location = next_waypoint.location;
        struct_state_.status = VehicleStatus::Holding;
        struct_state_.speed_mps = 0.0;
        list_waypoints_.erase(list_waypoints_.begin());
        logger->info("Vehicle {} reached waypoint {}", str_identifier_, next_waypoint.name);
        return;
    }

    if (struct_state_.range_remaining_m <= 0.0) {
        struct_state_.status = VehicleStatus::ReturningHome;
        struct_state_.speed_mps = 0.0;
        logger->warn("Vehicle {} depleted range; entering ReturningHome state", str_identifier_);
        return;
    }

    struct_state_.status = VehicleStatus::EnRoute;
    struct_state_.heading_deg = initial_bearing_deg(struct_state_.location, next_waypoint.location);

    const double speed_target = std::min(kinematics_.max_speed_mps, struct_state_.speed_mps + kinematics_.max_acceleration_mps2 * delta_seconds);
    struct_state_.speed_mps = speed_target;

    const double travel_distance = struct_state_.speed_mps * delta_seconds;
    if (travel_distance >= distance_to_waypoint_m) {
        struct_state_.location = next_waypoint.location;
        struct_state_.status = VehicleStatus::Holding;
        struct_state_.speed_mps = 0.0;
        list_waypoints_.erase(list_waypoints_.begin());
        logger->info("Vehicle {} reached waypoint {}", str_identifier_, next_waypoint.name);
        return;
    }

    struct_state_.location = advance_coordinate(struct_state_.location, travel_distance, struct_state_.heading_deg);
}

void AutonomousVehicle::update_battery(const Duration& tick) {
    const double delta_seconds = tick.count();
    const double distance_travelled = struct_state_.speed_mps * delta_seconds;
    const double consumption_wh = consumption_wh_per_m_ * distance_travelled;

    const double remaining_wh = std::max(0.0, (struct_state_.battery_percent / 100.0) * battery_capacity_wh_ - consumption_wh);
    struct_state_.battery_percent = std::max(0.0, (remaining_wh / battery_capacity_wh_) * 100.0);
    struct_state_.range_remaining_m = std::max(0.0, remaining_wh / consumption_wh_per_m_);
}

void AutonomousVehicle::publish_state(TelemetryBus& bus) {
    TelemetryEvent event{};
    event.state = struct_state_;
    bus.publish(event);
}

void AutonomousVehicle::maintain_collision_avoidance() {
    // Placeholder for coordination logic; real implementation would query nearby vehicles.
}

}  // namespace drone_swarm
