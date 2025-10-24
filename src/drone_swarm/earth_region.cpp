#include "drone_swarm/earth_region.hpp"

#include <stdexcept>

#include "drone_swarm/logging.hpp"

namespace drone_swarm {

EarthRegion::EarthRegion(EarthRegionConfig config)
    : config_(std::move(config)),
      logger_(get_logger()) {
    if (config_.name.empty()) {
        throw std::invalid_argument("EarthRegion requires a name");
    }
}

const std::string& EarthRegion::name() const noexcept {
    return config_.name;
}

const std::vector<MapTile>& EarthRegion::map_tiles() const noexcept {
    return list_tiles_;
}

void EarthRegion::initialize_tiles() {
    list_tiles_.clear();
    MapTile base_tile{};
    base_tile.identifier = config_.name + "-tile-0";
    base_tile.center = config_.origin;
    base_tile.radius_m = config_.default_tile_radius_m;
    list_tiles_.push_back(base_tile);
    logger_->info("Initialized {} map tiles for {}", list_tiles_.size(), config_.name);
}

void EarthRegion::request_tile_refresh() {
    logger_->info("Requesting tile refresh for {}", config_.name);
}

}  // namespace drone_swarm
