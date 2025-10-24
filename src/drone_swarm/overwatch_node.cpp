#include "drone_swarm/overwatch_node.hpp"

#include <chrono>
#include <cmath>
#include <numbers>

namespace drone_swarm {

namespace {
double haversine_distance_m(const GeodeticCoordinate& from, const GeodeticCoordinate& to) {
    constexpr double k_earth_radius_m{6'371'000.0};
    const auto to_radians = [](double degrees) {
        return degrees * std::numbers::pi / 180.0;
    };

    const double lat1 = to_radians(from.latitude_deg);
    const double lat2 = to_radians(to.latitude_deg);
    const double delta_lat = to_radians(to.latitude_deg - from.latitude_deg);
    const double delta_lon = to_radians(to.longitude_deg - from.longitude_deg);

    const double a = std::pow(std::sin(delta_lat / 2.0), 2)
        + std::cos(lat1) * std::cos(lat2) * std::pow(std::sin(delta_lon / 2.0), 2);
    const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    return k_earth_radius_m * c;
}
}  // namespace

OverwatchNode::OverwatchNode(std::string identifier, GeodeticCoordinate position, double coverage_radius_m, double altitude_m)
    : str_identifier_(std::move(identifier)),
      position_(position),
      coverage_radius_m_(coverage_radius_m),
      altitude_m_(altitude_m) {}

const std::string& OverwatchNode::identifier() const noexcept {
    return str_identifier_;
}

const GeodeticCoordinate& OverwatchNode::position() const noexcept {
    return position_;
}

double OverwatchNode::coverage_radius_m() const noexcept {
    return coverage_radius_m_;
}

double OverwatchNode::altitude_m() const noexcept {
    return altitude_m_;
}

void OverwatchNode::ingest_telemetry(const TelemetryEvent& event) {
    map_vehicle_states_[event.state.identifier] = event.state;
}

std::optional<DroneState> OverwatchNode::latest_state_for(const std::string& vehicle_id) const {
    const auto iterator_state = map_vehicle_states_.find(vehicle_id);
    if (iterator_state == map_vehicle_states_.end()) {
        return std::nullopt;
    }
    return iterator_state->second;
}

CoverageSummary OverwatchNode::coverage_status(TimePoint now) const {
    bool gap_detected = false;
    for (const auto& [vehicle_id, state] : map_vehicle_states_) {
        const double distance_m = haversine_distance_m(position_, state.location);
        if (distance_m > coverage_radius_m_) {
            gap_detected = true;
            break;
        }
        const Duration staleness = now - state.last_update_time;
        if (staleness.count() > 5.0) {
            gap_detected = true;
            break;
        }
    }
    return CoverageSummary{coverage_radius_m_, gap_detected};
}

}  // namespace drone_swarm
