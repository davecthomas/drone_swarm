#pragma once

#include <atomic>
#include <thread>
#include <vector>

#include "drone_swarm/autonomous_vehicle.hpp"
#include "drone_swarm/configuration.hpp"
#include "drone_swarm/earth_region.hpp"
#include "drone_swarm/overwatch_orchestrator.hpp"
#include "drone_swarm/rendering_pipeline.hpp"
#include "drone_swarm/telemetry_bus.hpp"

namespace drone_swarm {

class SimulationRuntime final {
  public:
    explicit SimulationRuntime(Configuration configuration);
    ~SimulationRuntime();

    void initialize();
    void run();
    void shutdown();

  private:
    void update_loop();
    void render_loop();

    Configuration configuration_;
    EarthRegion earth_region_;
    TelemetryBus telemetry_bus_;
    OverwatchOrchestrator orchestrator_;
    AutonomousVehicleList list_vehicles_;
    RenderingPipeline rendering_pipeline_;
    std::atomic<bool> flag_running_{false};
    std::thread update_thread_;
    std::thread render_thread_;
    std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace drone_swarm
