#pragma once

#include <string>

#include <mbgl/util/tile_server_options.hpp>

#include "drone_swarm/types.hpp"

namespace drone_swarm {

struct TileServiceDescriptor final {
    mbgl::TileServerOptions tile_server_options{};
    std::string style_url{};
    std::string cache_db_name{};
};

TileServiceDescriptor make_tile_service_descriptor(TileProviderType provider, const std::string& mapbox_token);

}  // namespace drone_swarm
