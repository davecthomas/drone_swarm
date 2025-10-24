#pragma once

#include <optional>
#include <string>
#include <vector>

#include "drone_swarm/logging.hpp"
#include "drone_swarm/types.hpp"

namespace drone_swarm {

struct MapTile final {
    std::string identifier{};
    GeodeticCoordinate center{};
    double radius_m{};
};

struct EarthRegionConfig final {
    std::string name{};
    GeodeticCoordinate origin{};
    double default_tile_radius_m{};
};

class EarthRegion final {
  public:
    explicit EarthRegion(EarthRegionConfig config);

    [[nodiscard]] const std::string& name() const noexcept;
    [[nodiscard]] const std::vector<MapTile>& map_tiles() const noexcept;

    void initialize_tiles();
    void request_tile_refresh();

  private:
    EarthRegionConfig config_;
    std::vector<MapTile> list_tiles_;
    std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace drone_swarm
