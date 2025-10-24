#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "drone_swarm/logging.hpp"
#include "drone_swarm/overwatch_node.hpp"
#include "drone_swarm/telemetry_bus.hpp"

namespace drone_swarm {

struct OrchestratorConfig final {
    GeodeticCoordinate region_center{};
    double region_radius_m{};
    Duration update_interval{Duration{1.0}};
    double overwatch_height_m{};
    double overwatch_coverage_m{};
    int max_retries{};
};

class OverwatchOrchestrator final {
  public:
    OverwatchOrchestrator(std::string name, OrchestratorConfig config, TelemetryBus& telemetry_bus);

    [[nodiscard]] const std::string& name() const noexcept;
    [[nodiscard]] const OrchestratorConfig& config() const noexcept;
    [[nodiscard]] const OverwatchNodeList& overwatch_nodes() const noexcept;

    void update(TimePoint now);
    void publish_logs();

  private:
    void refresh_overwatch_states();
    void reposition_for_gaps();
    void retry_if_needed(const std::function<void()>& operation, const std::string& operation_name);

    std::string str_name_;
    OrchestratorConfig config_;
    TelemetryBus& telemetry_bus_;
    OverwatchNodeList list_overwatch_nodes_;
    std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace drone_swarm
