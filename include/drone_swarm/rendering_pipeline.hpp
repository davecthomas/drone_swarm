#pragma once

#include <atomic>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "drone_swarm/camera_feed.hpp"
#include "drone_swarm/drone_state.hpp"
#include "drone_swarm/logging.hpp"
#include "drone_swarm/types.hpp"

namespace mbgl {
class Map;
}

namespace drone_swarm {

/** @brief Aggregates the viewport configuration used during renderer setup. */
struct RenderingConfig final {
    int window_width_px{};  /**< Output surface width in pixels. */
    int window_height_px{}; /**< Output surface height in pixels. */
    bool enable_vsync{};    /**< Enable vertical sync when true. */
};

/** @brief Manages MapLibre rendering, tile acquisition, and scene composition. */
class RenderingPipeline final {
  public:
    /**
     * @brief Construct a rendering pipeline for the provided configuration.
     *
     * @param config Window and vsync parameters.
     * @param mapbox_token Token used when Mapbox tiles are selected.
     * @param tile_provider Active tile provider choice.
     * @param cache_directory Cache directory where tile data is stored.
     */
    RenderingPipeline(RenderingConfig config,
                      std::string mapbox_token,
                      TileProviderType tile_provider,
                      std::filesystem::path cache_directory);
    ~RenderingPipeline();

    /** @brief Initialise GLFW, MapLibre, and the rendering thread. */
    void initialize();
    /** @brief Render the latest swarm state and optional camera feed. */
    void render_scene(const std::vector<DroneState>& visible_drones, const std::optional<CameraFrame>& selected_camera_frame);
    /** @brief Tear down the rendering thread and associated resources. */
    void shutdown();
    /** @brief Record that the MapLibre style finished loading (called by the observer). */
    void notify_style_ready();

  private:
    /** @brief Worker entry point hosting the MapLibre render loop. */
    void map_thread_entry();
    /** @brief Update MapLibre GeoJSON sources with current drone positions. */
    void apply_drone_states_on_map(mbgl::Map& map, const std::vector<DroneState>& states);
    /** @brief Ensure the tile cache directory exists on disk. */
    void ensure_cache_directory() const;

    RenderingConfig config_;                 /**< Window configuration. */
    std::string mapbox_token_;               /**< Token used for Mapbox tile requests. */
    TileProviderType tile_provider_;         /**< Selected tile provider. */
    std::filesystem::path cache_directory_;  /**< Cache directory for MapLibre assets. */
    std::atomic<bool> stop_requested_{false};/**< Flag signalling the render thread to exit. */
    std::thread map_thread_;                 /**< Background renderer thread. */
    std::shared_ptr<spdlog::logger> logger_; /**< Shared logger instance. */
    std::mutex state_mutex_;                 /**< Protects queued state updates. */
    std::vector<DroneState> queued_states_;  /**< Pending drone states awaiting render. */
    bool states_dirty_{false};               /**< Tracks whether the GeoJSON layer needs refresh. */
    std::atomic<bool> style_ready_{false};   /**< Tracks whether MapLibre style is loaded. */
    bool initialized_{false};                /**< Records whether initialise() has completed. */
};

}  // namespace drone_swarm
