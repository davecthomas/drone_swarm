// === Configuration ===========================================================
//
// Exposes strongly-typed configuration objects that describe orchestrator,
// rendering, logging, and tile-provider settings consumed across the simulator.
// `ConfigurationLoader` translates environment variables into these structures
// so downstream modules never touch `std::getenv` directly.

#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <string_view>

#include "drone_swarm/overwatch_orchestrator.hpp"
#include "drone_swarm/rendering_pipeline.hpp"
#include "drone_swarm/tile_provider.hpp"
#include "drone_swarm/types.hpp"

namespace drone_swarm {

/**
 * @brief Immutable bundle of runtime knobs for the drone swarm simulation.
 *
 * The structure surfaces strongly-typed configuration for orchestrator,
 * rendering, and authentication layers. Every field is populated by
 * ConfigurationLoader; consumers should treat the values as authoritative and
 * avoid consulting environment variables directly.
 */

struct Configuration final {
    std::string log_directory{};                   /**< Destination directory for structured logs. */
    OrchestratorConfig orchestrator{};             /**< Mesh planning configuration. */
    RenderingConfig rendering{};                   /**< Window/render settings. */
    std::unique_ptr<TileProvider> tile_provider{}; /**< Hydrated tile provider for the active session. */
    double update_hz{};                            /**< Simulation update cadence in Hertz. */
};

/**
 * @brief Utility responsible for hydrating Configuration from environment
 *        variables.
 */
class ConfigurationLoader final {
  public:
    static Configuration load(const std::filesystem::path& config_root);

  private:
    static double load_update_hz();
};

}  // namespace drone_swarm
