#pragma once

#include <mutex>
#include <optional>
#include <queue>
#include <string>

#include "drone_swarm/drone_state.hpp"

namespace drone_swarm {

struct TelemetryEvent final {
    DroneState state{};
};

class TelemetryBus final {
  public:
    void publish(const TelemetryEvent& event);
    [[nodiscard]] std::optional<TelemetryEvent> try_consume();

  private:
    mutable std::mutex mutex_;
    std::queue<TelemetryEvent> queue_events_;
};

}  // namespace drone_swarm
