#include "drone_swarm/overwatch_orchestrator.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <numbers>
#include <thread>

#include <fmt/format.h>
#include "drone_swarm/logging.hpp"

namespace drone_swarm {

namespace {
GeodeticCoordinate offset_coordinate(const GeodeticCoordinate& origin, double bearing_deg, double distance_m) {
    constexpr double k_earth_radius_m{6'371'000.0};
    const double angular_distance = distance_m / k_earth_radius_m;
    const double bearing_rad = bearing_deg * std::numbers::pi / 180.0;
    const double lat_rad = origin.latitude_deg * std::numbers::pi / 180.0;
    const double lon_rad = origin.longitude_deg * std::numbers::pi / 180.0;

    const double new_lat = std::asin(
        std::sin(lat_rad) * std::cos(angular_distance) + std::cos(lat_rad) * std::sin(angular_distance) * std::cos(bearing_rad)
    );

    const double new_lon = lon_rad
        + std::atan2(
            std::sin(bearing_rad) * std::sin(angular_distance) * std::cos(lat_rad),
            std::cos(angular_distance) - std::sin(lat_rad) * std::sin(new_lat)
        );

    return GeodeticCoordinate{
        new_lat * 180.0 / std::numbers::pi,
        new_lon * 180.0 / std::numbers::pi,
        origin.altitude_m
    };
}
}  // namespace

OverwatchOrchestrator::OverwatchOrchestrator(std::string name, OrchestratorConfig config, TelemetryBus& telemetry_bus)
    : str_name_(std::move(name)),
      config_(config),
      telemetry_bus_(telemetry_bus),
      logger_(get_logger()) {
    logger_->info("Initializing orchestrator {} with radius {} m", str_name_, config_.region_radius_m);

    const int sectors = std::max(1, static_cast<int>(config_.region_radius_m / config_.overwatch_coverage_m));
    const double sector_angle = 360.0 / static_cast<double>(sectors);
    for (int index = 0; index < sectors; ++index) {
        const double bearing = index * sector_angle;
        GeodeticCoordinate position = offset_coordinate(config_.region_center, bearing, config_.region_radius_m / 2.0);
        position.altitude_m = config_.overwatch_height_m;
        auto node = std::make_shared<OverwatchNode>(
            fmt::format("{}-node-{}", str_name_, index),
            position,
            config_.overwatch_coverage_m,
            config_.overwatch_height_m
        );
        list_overwatch_nodes_.push_back(std::move(node));
    }
}

const std::string& OverwatchOrchestrator::name() const noexcept {
    return str_name_;
}

const OrchestratorConfig& OverwatchOrchestrator::config() const noexcept {
    return config_;
}

const OverwatchNodeList& OverwatchOrchestrator::overwatch_nodes() const noexcept {
    return list_overwatch_nodes_;
}

void OverwatchOrchestrator::update(TimePoint now) {
    retry_if_needed([this]() { refresh_overwatch_states(); }, "refresh_overwatch_states");
    retry_if_needed([this]() { reposition_for_gaps(); }, "reposition_for_gaps");

    for (const auto& node : list_overwatch_nodes_) {
        const CoverageSummary summary = node->coverage_status(now);
        logger_->info(
            R"({"component":"overwatch","node":"{}","gap":{},"radius_m":{}})",
            node->identifier(),
            summary.gap_detected ? "true" : "false",
            summary.coverage_radius_m
        );
    }
}

void OverwatchOrchestrator::publish_logs() {
    logger_->flush();
}

void OverwatchOrchestrator::refresh_overwatch_states() {
    std::size_t ingested_count = 0;
    while (true) {
        std::optional<TelemetryEvent> optional_event = telemetry_bus_.try_consume();
        if (!optional_event.has_value()) {
            break;
        }
        for (const auto& node : list_overwatch_nodes_) {
            node->ingest_telemetry(optional_event.value());
        }
        ++ingested_count;
    }
    logger_->debug("Orchestrator {} ingested {} telemetry events", str_name_, ingested_count);
}

void OverwatchOrchestrator::reposition_for_gaps() {
    const TimePoint now = SteadyClock::now();
    bool repositioned = false;
    for (const auto& node : list_overwatch_nodes_) {
        const CoverageSummary summary = node->coverage_status(now);
        if (!summary.gap_detected) {
            continue;
        }
        GeodeticCoordinate adjusted_position = node->position();
        adjusted_position.altitude_m += 50.0;
        logger_->warn(
            R"({"component":"overwatch","node":"{}","action":"adjust_altitude","altitude_m":{}})",
            node->identifier(),
            adjusted_position.altitude_m
        );
        repositioned = true;
        break;
    }

    if (!repositioned) {
        logger_->debug("No repositioning required for orchestrator {}", str_name_);
    }
}

void OverwatchOrchestrator::retry_if_needed(const std::function<void()>& operation, const std::string& operation_name) {
    int attempt = 0;
    while (attempt <= config_.max_retries) {
        try {
            operation();
            return;
        } catch (const std::exception& exc) {
            logger_->error(
                R"({"component":"orchestrator","operation":"{}","attempt":{},"error":"{}"})",
                operation_name,
                attempt,
                exc.what()
            );
            if (attempt == config_.max_retries) {
                throw;
            }
        }
        ++attempt;
        std::this_thread::sleep_for(std::chrono::milliseconds(200 * (attempt + 1)));
    }
}

}  // namespace drone_swarm
