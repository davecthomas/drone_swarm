#include <chrono>

#include <catch2/catch_test_macros.hpp>

#include "logging_test_fixture.hpp"
#include "drone_swarm/overwatch_node.hpp"

using namespace drone_swarm;

namespace {
[[maybe_unused]] const bool logger_initialized = []() {
    drone_swarm::test::ensure_logger_initialized();
    return true;
}();
}  // namespace

TEST_CASE("OverwatchNode detects stale telemetry") {
    OverwatchNode node{"node-1", GeodeticCoordinate{0.0, 0.0, 0.0}, 1'000.0, 500.0};

    DroneState state{};
    state.identifier = "vehicle-1";
    state.location = GeodeticCoordinate{0.0, 0.0, 0.0};
    const TimePoint stale_time = TimePoint{SteadyClock::now() - std::chrono::duration_cast<SteadyClock::duration>(Duration{10.0})};
    state.last_update_time = stale_time;

    TelemetryEvent event{};
    event.state = state;
    node.ingest_telemetry(event);

    const CoverageSummary summary = node.coverage_status(SteadyClock::now());
    REQUIRE(summary.gap_detected);
}
