#pragma once

#include <filesystem>
#include <string>
#include <string_view>

#include "drone_swarm/overwatch_orchestrator.hpp"
#include "drone_swarm/rendering_pipeline.hpp"
#include "drone_swarm/types.hpp"

namespace drone_swarm {

struct Configuration final {
    std::string mapbox_token{};
    std::string log_directory{};
    OrchestratorConfig orchestrator{};
    RenderingConfig rendering{};
    TileProviderType tile_provider{TileProviderType::Mapbox};
    double update_hz{};
};

class ConfigurationLoader final {
  public:
    static Configuration load(const std::filesystem::path& config_root);

  private:
    static std::string load_mapbox_token(bool required);
    static double load_update_hz();
    static TileProviderType load_tile_provider();
};

}  // namespace drone_swarm
