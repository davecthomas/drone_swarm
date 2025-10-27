#include "drone_swarm/tile_service_factory.hpp"

#include <stdexcept>

namespace drone_swarm {

namespace {
constexpr char k_mapbox_style_url[] = "mapbox://styles/mapbox/satellite-streets-v12"; /**< Default Mapbox style blending raster imagery with overlays. */
constexpr char k_maplibre_demo_style_url[] = "https://demotiles.maplibre.org/style.json"; /**< Public MapLibre demo style for offline runs. */
constexpr char k_mapbox_cache_name[] = "mapbox_cache.db"; /**< Cache filename when using Mapbox tiles. */
constexpr char k_maplibre_cache_name[] = "maplibre_cache.db"; /**< Cache filename when using the MapLibre demo tiles. */
}

/**
 * @brief Build a tile service descriptor for the requested provider.
 */
TileServiceDescriptor make_tile_service_descriptor(TileProviderType provider, const std::string& mapbox_token) {
    TileServiceDescriptor descriptor{};

    switch (provider) {
        case TileProviderType::Mapbox: {
            if (mapbox_token.empty()) {
                throw std::runtime_error("Mapbox tile provider selected but MAPBOX_ACCESS_TOKEN is missing");
            }
            descriptor.tile_server_options = mbgl::TileServerOptions::MapboxConfiguration();
            descriptor.style_url = k_mapbox_style_url;
            descriptor.cache_db_name = k_mapbox_cache_name;
            break;
        }
        case TileProviderType::MapLibreDemo: {
            descriptor.tile_server_options = mbgl::TileServerOptions::MapLibreConfiguration();
            descriptor.style_url = k_maplibre_demo_style_url;
            descriptor.cache_db_name = k_maplibre_cache_name;
            break;
        }
        default:
            throw std::runtime_error("Unsupported tile provider type");
    }

    return descriptor;
}

}  // namespace drone_swarm
