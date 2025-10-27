// === Telemetry Bus ===========================================================
//
// Provides a minimal thread-safe queue for distributing telemetry events from
// autonomous vehicles to overwatch/orchestrator consumers.

#pragma once

#include <mutex>
#include <optional>
#include <queue>
#include <string>

#include "drone_swarm/drone_state.hpp"

namespace drone_swarm {

/** @brief Wrapper representing a single telemetry publication. */
struct TelemetryEvent final {
    DroneState state{};
};

/** @brief Thread-safe FIFO used to exchange telemetry events. */
class TelemetryBus final {
  public:
    /** @brief Publish a telemetry event to all consumers. */
    void publish(const TelemetryEvent& event);
    /** @brief Attempt to consume a pending event without blocking. */
    [[nodiscard]] std::optional<TelemetryEvent> try_consume();

  private:
    mutable std::mutex mutex_;
    std::queue<TelemetryEvent> queue_events_;
};

}  // namespace drone_swarm
