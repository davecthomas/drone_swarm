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

struct RenderingConfig final {
    int window_width_px{};
    int window_height_px{};
    bool enable_vsync{};
};

class RenderingPipeline final {
  public:
    RenderingPipeline(RenderingConfig config,
                      std::string mapbox_token,
                      TileProviderType tile_provider,
                      std::filesystem::path cache_directory);
    ~RenderingPipeline();

    void initialize();
    void render_scene(const std::vector<DroneState>& visible_drones, const std::optional<CameraFrame>& selected_camera_frame);
    void shutdown();

  private:
    void map_thread_entry();
    void apply_drone_states_on_map(mbgl::Map& map, const std::vector<DroneState>& states);
    void ensure_cache_directory() const;
    void notify_style_ready();

    RenderingConfig config_;
    std::string mapbox_token_;
    TileProviderType tile_provider_;
    std::filesystem::path cache_directory_;
    std::atomic<bool> stop_requested_{false};
    std::thread map_thread_;
    std::shared_ptr<spdlog::logger> logger_;
    std::mutex state_mutex_;
    std::vector<DroneState> queued_states_;
    bool states_dirty_{false};
    std::atomic<bool> style_ready_{false};
    bool initialized_{false};

    friend class PipelineMapObserver;
};

}  // namespace drone_swarm
