#include <catch2/catch.hpp>

#include "drone_swarm/overwatch_node.hpp"

using namespace drone_swarm;

TEST_CASE("OverwatchNode detects stale telemetry") {
    OverwatchNode node{"node-1", GeodeticCoordinate{0.0, 0.0, 0.0}, 1'000.0, 500.0};

    DroneState state{};
    state.identifier = "vehicle-1";
    state.location = GeodeticCoordinate{0.0, 0.0, 0.0};
    state.last_update_time = SteadyClock::now() - Duration{10.0};

    TelemetryEvent event{};
    event.state = state;
    node.ingest_telemetry(event);

    const CoverageSummary summary = node.coverage_status(SteadyClock::now());
    REQUIRE(summary.gap_detected);
}
