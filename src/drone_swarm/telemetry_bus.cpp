#include "drone_swarm/telemetry_bus.hpp"

namespace drone_swarm {

void TelemetryBus::publish(const TelemetryEvent& event) {
    std::scoped_lock lock(mutex_);
    queue_events_.push(event);
}

std::optional<TelemetryEvent> TelemetryBus::try_consume() {
    std::scoped_lock lock(mutex_);
    if (queue_events_.empty()) {
        return std::nullopt;
    }
    TelemetryEvent event = queue_events_.front();
    queue_events_.pop();
    return event;
}

}  // namespace drone_swarm
