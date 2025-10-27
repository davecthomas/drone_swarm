// === Earth Region ============================================================
//
// Models the operational area the swarm operates within, including derived map
// tiles used to seed MapLibre. Provides a thin abstraction to manage tile
// creation, refresh requests, and metadata.

#pragma once

#include <optional>
#include <string>
#include <vector>

#include "drone_swarm/logging.hpp"
#include "drone_swarm/types.hpp"

namespace drone_swarm {

/** @brief Lightweight descriptor for a single map tile. */
struct MapTile final {
    std::string identifier{};
    GeodeticCoordinate center{};
    double radius_m{};
};

/** @brief Configuration used to initialize the simulated earth region. */
struct EarthRegionConfig final {
    std::string name{};
    GeodeticCoordinate origin{};
    double default_tile_radius_m{};
};

/** @brief Owns tile metadata and orchestrates refresh cycles for the region. */
class EarthRegion final {
  public:
    explicit EarthRegion(EarthRegionConfig config);

    /** @brief Friendly region name. */
    [[nodiscard]] const std::string& name() const noexcept;
    /** @brief Tiles currently registered for rendering. */
    [[nodiscard]] const std::vector<MapTile>& map_tiles() const noexcept;

    /** @brief Generate the default tile set if not already populated. */
    void initialize_tiles();
    /** @brief Request tiles to refresh (placeholder hook for future expansion). */
    void request_tile_refresh();

  private:
    EarthRegionConfig config_;
    std::vector<MapTile> list_tiles_;
    std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace drone_swarm
