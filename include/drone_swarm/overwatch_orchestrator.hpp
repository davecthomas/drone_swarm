// === Overwatch Orchestrator ==================================================
//
// Coordinates the network of overwatch nodes that monitor coverage gaps. This
// header exposes the configuration structure and orchestrator class used to
// reposition nodes and emit structured logs.

#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "drone_swarm/logging.hpp"
#include "drone_swarm/overwatch_node.hpp"
#include "drone_swarm/telemetry_bus.hpp"

namespace drone_swarm {

/**
 * @brief Tunable parameters that guide the overwatch mesh orchestration.
 *
 * These values influence mesh density, reposition cadence, and retry
 * behavior. They are populated at startup from the configuration loader and
 * treated as immutable while the simulation runs.
 */
struct OrchestratorConfig final {
    GeodeticCoordinate region_center{};
    double region_radius_m{};
    Duration update_interval{Duration{1.0}};
    double overwatch_height_m{};
    double overwatch_coverage_m{};
    int max_retries{};
};

/** @brief Manages overwatch nodes and mesh health over time. */
class OverwatchOrchestrator final {
  public:
    OverwatchOrchestrator(std::string name, OrchestratorConfig config, TelemetryBus& telemetry_bus);

    /** @brief Human-readable orchestrator identifier. */
    [[nodiscard]] const std::string& name() const noexcept;
    /** @brief Accessor for the immutable configuration bundle. */
    [[nodiscard]] const OrchestratorConfig& config() const noexcept;
    /** @brief Current list of managed overwatch nodes. */
    [[nodiscard]] const OverwatchNodeList& overwatch_nodes() const noexcept;

    /** @brief Execute a single orchestration tick at the supplied time. */
    void update(TimePoint now);
    /** @brief Flush pending log entries to the shared logger. */
    void publish_logs();

  private:
    /** @brief Refresh internal node state from telemetry. */
    void refresh_overwatch_states();
    /** @brief Reposition nodes when coverage gaps are detected. */
    void reposition_for_gaps();
    /** @brief Apply bounded retries to a critical orchestration operation. */
    void retry_if_needed(const std::function<void()>& operation, const std::string& operation_name);

    std::string str_name_;
    OrchestratorConfig config_;
    TelemetryBus& telemetry_bus_;
    OverwatchNodeList list_overwatch_nodes_;
    std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace drone_swarm
