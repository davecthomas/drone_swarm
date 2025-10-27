#include "drone_swarm/autonomous_vehicle.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

#include <spdlog/spdlog.h>

#include "drone_swarm/logging.hpp"

namespace drone_swarm {

namespace {

constexpr double k_earth_radius_m{6'371'000.0};            /**< Mean Earth radius used for geodesic calculations. */
constexpr double k_arrival_threshold_m{2.0};               /**< Distance below which a waypoint is considered reached. */
constexpr double k_percent_scale{100.0};                   /**< Conversion factor between fractional and percentage representations. */
constexpr double k_minimum_speed_guard_mps{0.1};           /**< Guard speed to avoid division-by-zero in time estimates. */
constexpr char k_return_to_base_waypoint_name[] = "return-to-base"; /**< Identifier applied to auto-generated return waypoint. */

/**
 * @brief Convert degrees to radians.
 */
constexpr double degrees_to_radians(double degrees) {
    return degrees * std::numbers::pi / 180.0;
}

/**
 * @brief Convert radians to degrees.
 */
constexpr double radians_to_degrees(double radians) {
    return radians * 180.0 / std::numbers::pi;
}

/**
 * @brief Compute a destination coordinate for a great-circle path.
 */
GeodeticCoordinate advance_coordinate(const GeodeticCoordinate& start, double distance_m, double bearing_deg) {
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

    return GeodeticCoordinate{radians_to_degrees(new_lat), radians_to_degrees(new_lon), start.altitude_m};
}

/**
 * @brief Determine the great-circle distance separating two coordinates.
 */
double haversine_distance_m(const GeodeticCoordinate& from, const GeodeticCoordinate& to) {
    const double lat1 = degrees_to_radians(from.latitude_deg);
    const double lat2 = degrees_to_radians(to.latitude_deg);
    const double delta_lat = lat2 - lat1;
    const double delta_lon = degrees_to_radians(to.longitude_deg - from.longitude_deg);

    const double a = std::pow(std::sin(delta_lat / 2.0), 2)
        + std::cos(lat1) * std::cos(lat2) * std::pow(std::sin(delta_lon / 2.0), 2);
    const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    return k_earth_radius_m * c;
}

/**
 * @brief Compute the initial great-circle bearing from one coordinate to another.
 */
double initial_bearing_deg(const GeodeticCoordinate& from, const GeodeticCoordinate& to) {
    const double lat1 = degrees_to_radians(from.latitude_deg);
    const double lat2 = degrees_to_radians(to.latitude_deg);
    const double delta_lon = degrees_to_radians(to.longitude_deg - from.longitude_deg);

    const double y = std::sin(delta_lon) * std::cos(lat2);
    const double x = std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(delta_lon);
    const double bearing_rad = std::atan2(y, x);
    return std::fmod(radians_to_degrees(bearing_rad) + 360.0, 360.0);
}

}  // namespace

AutonomousVehicle::AutonomousVehicle(
    std::string identifier,
    VehicleKinematics kinematics,
    GeodeticCoordinate initial_location,
    GeodeticCoordinate base_location,
    PowertrainModel powertrain_model,
    double battery_capacity_wh,
    double consumption_wh_per_m,
    CameraFeedList camera_feeds
)
    : str_identifier_(std::move(identifier)),
      kinematics_(kinematics),
      base_location_(base_location),
      powertrain_model_(powertrain_model),
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
    if (powertrain_model_.minimum_return_speed_mps <= 0.0) {
        throw std::invalid_argument("AutonomousVehicle minimum return speed must be positive");
    }
    struct_state_.identifier = str_identifier_;
    struct_state_.location = initial_location;
    struct_state_.heading_deg = 0.0;
    struct_state_.status = VehicleStatus::Idle;
    struct_state_.battery_percent = k_percent_scale;
    struct_state_.range_remaining_m = battery_capacity_wh_ / consumption_wh_per_m_;
}

/**
 * @brief Return the unique vehicle identifier.
 */
const std::string& AutonomousVehicle::identifier() const noexcept {
    return str_identifier_;
}

/**
 * @brief Expose the current state snapshot for observers.
 */
const DroneState& AutonomousVehicle::state() const noexcept {
    return struct_state_;
}

/**
 * @brief Provide read-only access to the kinematic limits.
 */
const VehicleKinematics& AutonomousVehicle::kinematics() const noexcept {
    return kinematics_;
}

/**
 * @brief Report the total battery capacity in watt-hours.
 */
double AutonomousVehicle::battery_capacity_wh() const noexcept {
    return battery_capacity_wh_;
}

/**
 * @brief Retrieve the list of onboard camera feeds.
 */
const CameraFeedList& AutonomousVehicle::camera_feeds() const noexcept {
    return camera_feeds_;
}

/**
 * @brief Replace the active waypoint queue.
 */
void AutonomousVehicle::set_waypoints(std::vector<Waypoint> waypoints) {
    list_waypoints_ = std::move(waypoints);
    const auto logger = get_logger();
    logger->info("Vehicle {} loaded {} waypoints", str_identifier_, list_waypoints_.size());
}

/**
 * @brief Advance the vehicle state machine and publish telemetry.
 */
void AutonomousVehicle::update(const Duration& tick, TelemetryBus& bus) {
    struct_state_.last_update_time = SteadyClock::now();

    ensure_return_waypoint();

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

/**
 * @brief Progress navigation toward the next waypoint and update motion state.
 */
void AutonomousVehicle::update_navigation(const Duration& tick) {
    auto logger = get_logger();
    Waypoint& next_waypoint = list_waypoints_.front();
    const double delta_seconds = tick.count();

    const double distance_to_waypoint_m = haversine_distance_m(struct_state_.location, next_waypoint.location);
    if (distance_to_waypoint_m <= k_arrival_threshold_m) {
        struct_state_.location = next_waypoint.location;
        struct_state_.speed_mps = 0.0;
        list_waypoints_.erase(list_waypoints_.begin());

        if (struct_state_.status == VehicleStatus::LowPowerReturn) {
            struct_state_.status = VehicleStatus::Charging;
            logger->info("Vehicle {} reached base and is commencing charging", str_identifier_);
        } else {
            struct_state_.status = VehicleStatus::Holding;
            logger->info("Vehicle {} reached waypoint {}", str_identifier_, next_waypoint.name);
        }
        return;
    }

    if (struct_state_.range_remaining_m <= 0.0) {
        struct_state_.status = VehicleStatus::ReturningHome;
        struct_state_.speed_mps = 0.0;
        logger->warn("Vehicle {} depleted range; entering ReturningHome state", str_identifier_);
        return;
    }

    if (struct_state_.status != VehicleStatus::LowPowerReturn) {
        struct_state_.status = VehicleStatus::EnRoute;
    }
    struct_state_.heading_deg = initial_bearing_deg(struct_state_.location, next_waypoint.location);

    const double speed_target = std::min(kinematics_.max_speed_mps, struct_state_.speed_mps + kinematics_.max_acceleration_mps2 * delta_seconds);
    struct_state_.speed_mps = speed_target;

    const double travel_distance = struct_state_.speed_mps * delta_seconds;
    if (travel_distance >= distance_to_waypoint_m) {
        struct_state_.location = next_waypoint.location;
        struct_state_.speed_mps = 0.0;
        list_waypoints_.erase(list_waypoints_.begin());
        if (struct_state_.status == VehicleStatus::LowPowerReturn) {
            struct_state_.status = VehicleStatus::Charging;
            logger->info("Vehicle {} reached base and is commencing charging", str_identifier_);
        } else {
            struct_state_.status = VehicleStatus::Holding;
            logger->info("Vehicle {} reached waypoint {}", str_identifier_, next_waypoint.name);
        }
        return;
    }

    struct_state_.location = advance_coordinate(struct_state_.location, travel_distance, struct_state_.heading_deg);
}

/**
 * @brief Update energy bookkeeping and trigger return-to-base if necessary.
 */
void AutonomousVehicle::update_battery(const Duration& tick) {
    const double delta_seconds = tick.count();
    const double distance_travelled_m = struct_state_.speed_mps * delta_seconds;
    const double consumption_wh = consumption_wh_per_m_ * distance_travelled_m;

    const double remaining_wh = std::max(0.0, (struct_state_.battery_percent / k_percent_scale) * battery_capacity_wh_ - consumption_wh);
    struct_state_.battery_percent = std::max(0.0, (remaining_wh / battery_capacity_wh_) * k_percent_scale);
    struct_state_.range_remaining_m = std::max(0.0, remaining_wh / consumption_wh_per_m_);

    const double required_reserve_percent = calculate_return_reserve_percent();
    if (struct_state_.battery_percent <= required_reserve_percent) {
        initiate_low_power_return();
    }
}

/**
 * @brief Publish the current vehicle state to interested subscribers.
 */
void AutonomousVehicle::publish_state(TelemetryBus& bus) {
    TelemetryEvent event{};
    event.state = struct_state_;
    bus.publish(event);
}

/**
 * @brief Placeholder for future swarm-level collision avoidance.
 */
void AutonomousVehicle::maintain_collision_avoidance() {
    // Placeholder for cooperative avoidance once a swarm coordinator is available.
}

/**
 * @brief Determine the required battery reserve to guarantee a safe return to base.
 */
double AutonomousVehicle::calculate_return_reserve_percent() const {
    const double distance_to_base_m = haversine_distance_m(struct_state_.location, base_location_);
    const double minimum_speed = std::max(powertrain_model_.minimum_return_speed_mps, k_minimum_speed_guard_mps);
    const double effective_speed = std::max(struct_state_.speed_mps, minimum_speed);
    const double expected_travel_time_s = distance_to_base_m / effective_speed;

    const double reserve = powertrain_model_.base_reserve_percent
        + distance_to_base_m * powertrain_model_.reserve_percent_per_meter
        + expected_travel_time_s * powertrain_model_.reserve_percent_per_second;
    return std::clamp(reserve, 0.0, k_percent_scale);
}

/**
 * @brief Switch the vehicle into a low-power return sequence.
 */
void AutonomousVehicle::initiate_low_power_return() {
    if (struct_state_.status == VehicleStatus::LowPowerReturn) {
        return;
    }
    const auto logger = get_logger();
    logger->warn("Vehicle {} initiating low-power return to base", str_identifier_);
    struct_state_.status = VehicleStatus::LowPowerReturn;
    list_waypoints_.clear();
    list_waypoints_.push_back(Waypoint{k_return_to_base_waypoint_name, base_location_, 0.0});
}

/**
 * @brief Ensure the return-to-base waypoint remains present when in low-power mode.
 */
void AutonomousVehicle::ensure_return_waypoint() {
    if (struct_state_.status != VehicleStatus::LowPowerReturn) {
        return;
    }

    if (list_waypoints_.empty()
        || list_waypoints_.front().location.latitude_deg != base_location_.latitude_deg
        || list_waypoints_.front().location.longitude_deg != base_location_.longitude_deg
        || list_waypoints_.front().location.altitude_m != base_location_.altitude_m) {
        list_waypoints_.clear();
        list_waypoints_.push_back(Waypoint{k_return_to_base_waypoint_name, base_location_, 0.0});
    }
}

}  // namespace drone_swarm
