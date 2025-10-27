// === Overwatch Node ==========================================================
//
// Represents a fixed observation point that monitors drone telemetry to detect
// coverage gaps within its radius. Provides APIs for telemetry ingestion and
// summarizing coverage status for the orchestrator.

#pragma once

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "drone_swarm/drone_state.hpp"
#include "drone_swarm/telemetry_bus.hpp"

namespace drone_swarm {

/** @brief Aggregated coverage information per overwatch node. */
struct CoverageSummary final {
    double coverage_radius_m{};
    bool gap_detected{};
};

/** @brief Tracks vehicle telemetry within a coverage radius to detect gaps. */
class OverwatchNode final {
  public:
    OverwatchNode(std::string identifier, GeodeticCoordinate position, double coverage_radius_m, double altitude_m);

    /** @brief Stable identifier used in logs and orchestration. */
    [[nodiscard]] const std::string& identifier() const noexcept;
    /** @brief Current geodetic position of the node. */
    [[nodiscard]] const GeodeticCoordinate& position() const noexcept;
    /** @brief Monitoring radius in meters. */
    [[nodiscard]] double coverage_radius_m() const noexcept;
    /** @brief Altitude used for visualization/debugging. */
    [[nodiscard]] double altitude_m() const noexcept;

    /** @brief Consume a telemetry event from the bus. */
    void ingest_telemetry(const TelemetryEvent& event);
    /** @brief Retrieve the latest state known for @p vehicle_id. */
    [[nodiscard]] std::optional<DroneState> latest_state_for(const std::string& vehicle_id) const;
    /** @brief Summarize coverage health as of @p now. */
    [[nodiscard]] CoverageSummary coverage_status(TimePoint now) const;

  private:
    std::string str_identifier_;
    GeodeticCoordinate position_;
    double coverage_radius_m_;
    double altitude_m_;
    std::unordered_map<std::string, DroneState> map_vehicle_states_;
};

using OverwatchNodePtr = std::shared_ptr<OverwatchNode>;
using OverwatchNodeList = std::vector<OverwatchNodePtr>;

}  // namespace drone_swarm
