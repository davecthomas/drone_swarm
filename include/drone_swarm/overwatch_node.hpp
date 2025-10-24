#pragma once

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "drone_swarm/drone_state.hpp"
#include "drone_swarm/telemetry_bus.hpp"

namespace drone_swarm {

struct CoverageSummary final {
    double coverage_radius_m{};
    bool gap_detected{};
};

class OverwatchNode final {
  public:
    OverwatchNode(std::string identifier, GeodeticCoordinate position, double coverage_radius_m, double altitude_m);

    [[nodiscard]] const std::string& identifier() const noexcept;
    [[nodiscard]] const GeodeticCoordinate& position() const noexcept;
    [[nodiscard]] double coverage_radius_m() const noexcept;
    [[nodiscard]] double altitude_m() const noexcept;

    void ingest_telemetry(const TelemetryEvent& event);
    [[nodiscard]] std::optional<DroneState> latest_state_for(const std::string& vehicle_id) const;
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
