// === Simulation Runtime ======================================================
//
// Coordinates initialization, update, and render loops for the simulator.
// Responsible for wiring configuration, orchestrator, rendering pipeline, and
// autonomous vehicles together.

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

/** @brief High-level orchestrator managing update and render threads. */
class SimulationRuntime final {
  public:
    explicit SimulationRuntime(Configuration configuration);
    ~SimulationRuntime();

    /** @brief Initialize world state, vehicles, and rendering pipeline. */
    void initialize();
    /** @brief Start background update/render loops. */
    void run();
    /** @brief Shut down threads and release resources. */
    void shutdown();

  private:
    /** @brief Tick loop updating vehicles and orchestrator. */
    void update_loop();
    /** @brief Rendering loop driving MapLibre presentation. */
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
