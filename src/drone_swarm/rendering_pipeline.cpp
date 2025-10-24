#include "drone_swarm/rendering_pipeline.hpp"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <mbgl/gfx/backend_scope.hpp>
#include <mbgl/gl/renderable_resource.hpp>
#include <mbgl/gl/renderer_backend.hpp>
#include <mbgl/map/camera.hpp>
#include <mbgl/map/edge_insets.hpp>
#include <mbgl/map/map.hpp>
#include <mbgl/map/map_options.hpp>
#include <mbgl/map/map_observer.hpp>
#include <mbgl/map/mode.hpp>
#include <mbgl/renderer/renderer.hpp>
#include <mbgl/renderer/renderer_frontend.hpp>
#include <mbgl/style/layers/circle_layer.hpp>
#include <mbgl/style/sources/geojson_source.hpp>
#include <mbgl/style/style.hpp>
#include <mbgl/util/client_options.hpp>
#include <mbgl/util/geo.hpp>
#include <mbgl/util/logging.hpp>
#include <mbgl/util/run_loop.hpp>
#include <mbgl/util/tile_server_options.hpp>
#include <mbgl/storage/resource_options.hpp>

#include <mapbox/geojson.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <numbers>
#include <stdexcept>
#include <string_view>

#include "drone_swarm/logging.hpp"
#include "drone_swarm/tile_service_factory.hpp"

namespace drone_swarm {

namespace {

constexpr double kSanDiegoLatitude = 32.715736;
constexpr double kSanDiegoLongitude = -117.161087;
constexpr double kSanDiegoCoverageMiles = 50.0;
constexpr double kMilesToMeters = 1609.344;
constexpr char kDroneSourceId[] = "drone-vehicles";
constexpr char kDroneLayerId[] = "drone-vehicles-layer";

std::once_flag glfw_once_flag;
std::atomic<int> glfw_usage_count{0};

void glfw_error_callback(int error_code, const char* description) {
    auto logger = get_logger();
    logger->error("GLFW error {}: {}", error_code, description ? description : "unknown");
}

void ensure_glfw_initialized() {
    std::call_once(
        glfw_once_flag,
        []() {
            if (!glfwInit()) {
                throw std::runtime_error("Failed to initialize GLFW");
            }
            glfwSetErrorCallback(glfw_error_callback);
        }
    );
    glfw_usage_count.fetch_add(1, std::memory_order_relaxed);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
}

void terminate_glfw_if_unused() {
    if (glfw_usage_count.fetch_sub(1, std::memory_order_relaxed) == 1) {
        glfwTerminate();
    }
}

double degrees_to_radians(double degrees) {
    return degrees * std::numbers::pi / 180.0;
}

double radians_to_degrees(double radians) {
    return radians * 180.0 / std::numbers::pi;
}

mbgl::LatLngBounds san_diego_bounds() {
    const double radius_m = kSanDiegoCoverageMiles * kMilesToMeters;
    const double earth_radius_m = 6'371'000.0;

    const double angular_distance = radius_m / earth_radius_m;
    const double delta_lat = radians_to_degrees(angular_distance);
    const double delta_lon = radians_to_degrees(angular_distance / std::cos(degrees_to_radians(kSanDiegoLatitude)));

    const mbgl::LatLng southwest{kSanDiegoLatitude - delta_lat, kSanDiegoLongitude - delta_lon};
    const mbgl::LatLng northeast{kSanDiegoLatitude + delta_lat, kSanDiegoLongitude + delta_lon};
    return mbgl::LatLngBounds::hull(southwest, northeast);
}

std::string_view to_string(VehicleStatus status) {
    switch (status) {
        case VehicleStatus::Idle:
            return "Idle";
        case VehicleStatus::EnRoute:
            return "EnRoute";
        case VehicleStatus::Holding:
            return "Holding";
        case VehicleStatus::ReturningHome:
            return "ReturningHome";
        case VehicleStatus::Charging:
            return "Charging";
        case VehicleStatus::Faulted:
            return "Faulted";
    }
    return "Unknown";
}

class PipelineMapObserver final : public mbgl::MapObserver {
  public:
    explicit PipelineMapObserver(RenderingPipeline& pipeline) : pipeline_(pipeline) {}

    void onDidFinishLoadingStyle() override {
        pipeline_.notify_style_ready();
    }

  private:
    RenderingPipeline& pipeline_;
};

class DroneGLBackend final : public mbgl::gl::RendererBackend, public mbgl::gfx::Renderable {
  public:
    DroneGLBackend(GLFWwindow* window, bool enable_vsync)
        : mbgl::gl::RendererBackend(mbgl::gfx::ContextMode::Unique),
          mbgl::gfx::Renderable(initial_framebuffer_size(window), std::make_unique<RenderableResourceImpl>(*this)),
          window_(window),
          enable_vsync_(enable_vsync) {
        glfwMakeContextCurrent(window_);
        glfwSwapInterval(enable_vsync_ ? 1 : 0);
        updateAssumedState();
    }

    ~DroneGLBackend() override = default;

    mbgl::gfx::RendererBackend& getRendererBackend() override { return *this; }
    mbgl::gfx::Renderable& getDefaultRenderable() override { return *this; }

    mbgl::Size getSize() const override {
        return size;
    }

    void setSize(const mbgl::Size new_size) override {
        size = new_size;
    }

    void update_framebuffer_dimensions() {
        int fb_width = 0;
        int fb_height = 0;
        glfwGetFramebufferSize(window_, &fb_width, &fb_height);
        fb_width = std::max(fb_width, 1);
        fb_height = std::max(fb_height, 1);
        setSize(mbgl::Size{static_cast<uint32_t>(fb_width), static_cast<uint32_t>(fb_height)});
    }

    double pixel_ratio() const {
        int window_width = 0;
        int window_height = 0;
        glfwGetWindowSize(window_, &window_width, &window_height);
        if (window_width == 0 || window_height == 0) {
            return 1.0;
        }
        return static_cast<double>(size.width) / static_cast<double>(window_width);
    }

    void swap_buffers() {
        glfwSwapBuffers(window_);
    }

  protected:
    void activate() override {
        glfwMakeContextCurrent(window_);
    }

    void deactivate() override {
        glfwMakeContextCurrent(nullptr);
    }

    mbgl::gl::ProcAddress getExtensionFunctionPointer(const char* name) override {
        return glfwGetProcAddress(name);
    }

    void updateAssumedState() override {
        assumeFramebufferBinding(0);
        setViewport(0, 0, size);
    }

  private:
    class RenderableResourceImpl final : public mbgl::gl::RenderableResource {
      public:
        explicit RenderableResourceImpl(DroneGLBackend& backend) : backend_(backend) {}

        void bind() override {
            backend_.setFramebufferBinding(0);
            backend_.setViewport(0, 0, backend_.getSize());
        }

        void swap() override {
            backend_.swap_buffers();
        }

      private:
        DroneGLBackend& backend_;
    };

    static mbgl::Size initial_framebuffer_size(GLFWwindow* window) {
        int fb_width = 0;
        int fb_height = 0;
        glfwGetFramebufferSize(window, &fb_width, &fb_height);
        fb_width = std::max(fb_width, 1);
        fb_height = std::max(fb_height, 1);
        return mbgl::Size{static_cast<uint32_t>(fb_width), static_cast<uint32_t>(fb_height)};
    }

    GLFWwindow* window_;
    bool enable_vsync_;
};

class SimpleRendererFrontend final : public mbgl::RendererFrontend {
  public:
    SimpleRendererFrontend(DroneGLBackend& backend, double pixel_ratio)
        : backend_(backend),
          renderer_(std::make_unique<mbgl::Renderer>(backend_, pixel_ratio)) {}

    ~SimpleRendererFrontend() override = default;

    void reset() override {
        renderer_.reset();
    }

    void setObserver(mbgl::RendererObserver& observer) override {
        renderer_->setObserver(&observer);
    }

    void update(std::shared_ptr<mbgl::UpdateParameters> params) override {
        {
            std::scoped_lock lock(mutex_);
            update_parameters_ = std::move(params);
        }
        needs_render_.store(true, std::memory_order_relaxed);
    }

    const mbgl::TaggedScheduler& getThreadPool() const override {
        return backend_.getThreadPool();
    }

    void render() {
        std::shared_ptr<mbgl::UpdateParameters> params;
        {
            std::scoped_lock lock(mutex_);
            params = update_parameters_;
        }
        if (!params) {
            return;
        }

        mbgl::gfx::BackendScope guard{backend_, mbgl::gfx::BackendScope::ScopeType::Implicit};
        renderer_->render(params);
        needs_render_.store(false, std::memory_order_relaxed);
    }

    bool needs_render() const {
        return needs_render_.load(std::memory_order_relaxed);
    }

  private:
    DroneGLBackend& backend_;
    std::unique_ptr<mbgl::Renderer> renderer_;
    std::shared_ptr<mbgl::UpdateParameters> update_parameters_;
    mutable std::mutex mutex_;
    std::atomic<bool> needs_render_{true};
};

mapbox::geojson::feature feature_from_state(const DroneState& state) {
    mapbox::geojson::feature feature{mapbox::geojson::point{state.location.longitude_deg, state.location.latitude_deg}};
    feature.id = state.identifier;
        feature.properties["status"] = std::string{to_string(state.status)};
        feature.properties["speed_mps"] = state.speed_mps;
        feature.properties["battery_percent"] = state.battery_percent;
        feature.properties["range_remaining_m"] = state.range_remaining_m;
        feature.properties["last_update_epoch_ms"] = std::chrono::duration_cast<std::chrono::milliseconds>(
            state.last_update_time.time_since_epoch()
        ).count();
        return feature;
}

}  // namespace

RenderingPipeline::RenderingPipeline(RenderingConfig config,
                                     std::string mapbox_token,
                                     TileProviderType tile_provider,
                                     std::filesystem::path cache_directory)
    : config_(config),
      mapbox_token_(std::move(mapbox_token)),
      tile_provider_(tile_provider),
      cache_directory_(std::move(cache_directory)),
      logger_(get_logger()) {}

RenderingPipeline::~RenderingPipeline() {
    shutdown();
}

void RenderingPipeline::initialize() {
    if (initialized_) {
        return;
    }
    ensure_cache_directory();

    logger_->info(
        "Starting MapLibre renderer ({}x{}, vsync={}, cache={})",
        config_.window_width_px,
        config_.window_height_px,
        config_.enable_vsync ? "true" : "false",
        cache_directory_.string()
    );

    stop_requested_.store(false, std::memory_order_relaxed);
    style_ready_.store(false, std::memory_order_relaxed);
    map_thread_ = std::thread(&RenderingPipeline::map_thread_entry, this);
    initialized_ = true;
}

void RenderingPipeline::render_scene(
    const std::vector<DroneState>& visible_drones,
    const std::optional<CameraFrame>& /*selected_camera_frame*/
) {
    if (!initialized_) {
        return;
    }
    std::scoped_lock lock(state_mutex_);
    queued_states_ = visible_drones;
    states_dirty_ = true;
}

void RenderingPipeline::shutdown() {
    if (!initialized_) {
        return;
    }
    logger_->info("Stopping MapLibre renderer");
    stop_requested_.store(true, std::memory_order_relaxed);

    if (glfw_usage_count.load(std::memory_order_relaxed) > 0) {
        glfwPostEmptyEvent();
    }

    if (map_thread_.joinable()) {
        map_thread_.join();
    }

    initialized_ = false;
}

void RenderingPipeline::notify_style_ready() {
    style_ready_.store(true, std::memory_order_relaxed);
}

void RenderingPipeline::ensure_cache_directory() const {
    std::error_code error;
    std::filesystem::create_directories(cache_directory_, error);
    if (error) {
        throw std::runtime_error("Unable to create cache directory at " + cache_directory_.string());
    }
}

void RenderingPipeline::map_thread_entry() {
    try {
        ensure_glfw_initialized();

        const TileServiceDescriptor tile_service = make_tile_service_descriptor(tile_provider_, mapbox_token_);

        GLFWwindow* window = glfwCreateWindow(
            config_.window_width_px,
            config_.window_height_px,
            "Drone Swarm Overwatch",
            nullptr,
            nullptr
        );

        if (window == nullptr) {
            throw std::runtime_error("Failed to create GLFW window");
        }

        DroneGLBackend backend(window, config_.enable_vsync);
        SimpleRendererFrontend frontend(backend, backend.pixel_ratio());

        PipelineMapObserver map_observer(*this);

        const auto cache_file = cache_directory_ / tile_service.cache_db_name;
        mbgl::ResourceOptions resource_options;
        resource_options.withCachePath(cache_file.string()).withTileServerOptions(tile_service.tile_server_options);
        if (!mapbox_token_.empty()) {
            resource_options.withApiKey(mapbox_token_);
        }

        mbgl::ClientOptions client_options;

        mbgl::Map map(
            frontend,
        map_observer,
        mbgl::MapOptions()
            .withSize(backend.getSize())
            .withPixelRatio(backend.pixel_ratio())
            .withMapMode(mbgl::MapMode::Continuous),
            resource_options,
            client_options
        );

        map.getStyle().loadURL(tile_service.style_url);

        const mbgl::LatLngBounds bounds = san_diego_bounds();
        const mbgl::CameraOptions camera = map.cameraForLatLngBounds(
            bounds,
            mbgl::EdgeInsets{40.0, 40.0, 40.0, 40.0},
            std::nullopt,
            45.0
        );
        map.jumpTo(camera);

        mbgl::Size last_size = backend.getSize();

        while (!stop_requested_.load(std::memory_order_relaxed)) {
            if (glfwWindowShouldClose(window) != 0) {
                stop_requested_.store(true, std::memory_order_relaxed);
                break;
            }

            glfwPollEvents();
            backend.update_framebuffer_dimensions();
            const mbgl::Size current_size = backend.getSize();
            if (current_size != last_size) {
                map.setSize(current_size);
                last_size = current_size;
            }

            std::vector<DroneState> states_copy;
            {
                std::scoped_lock lock(state_mutex_);
                if (states_dirty_) {
                    states_copy = queued_states_;
                    states_dirty_ = false;
                }
            }

            if (!states_copy.empty() && style_ready_.load(std::memory_order_relaxed)) {
                apply_drone_states_on_map(map, states_copy);
            }

            frontend.render();
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }

        glfwDestroyWindow(window);
        terminate_glfw_if_unused();
    } catch (const std::exception& exc) {
        logger_->error("Rendering thread failure: {}", exc.what());
        terminate_glfw_if_unused();
    }
}

void RenderingPipeline::apply_drone_states_on_map(mbgl::Map& map, const std::vector<DroneState>& states) {
    mapbox::geojson::feature_collection collection;
    collection.reserve(states.size());
    for (const DroneState& state : states) {
        collection.push_back(feature_from_state(state));
    }

    auto* source = map.getStyle().getSourceAs<mbgl::style::GeoJSONSource>(kDroneSourceId);
    if (source == nullptr) {
        mbgl::style::GeoJSONOptions source_options;
        auto geojson_source = std::make_unique<mbgl::style::GeoJSONSource>(kDroneSourceId, source_options);
        geojson_source->setGeoJSON(collection);
        map.getStyle().addSource(std::move(geojson_source));

        auto layer = std::make_unique<mbgl::style::CircleLayer>(kDroneLayerId, kDroneSourceId);
        layer->setCircleRadius(8.0f);
        layer->setCircleOpacity(0.85f);
        layer->setCircleColor(mbgl::Color{0.11f, 0.67f, 0.95f, 1.0f});
        layer->setCircleStrokeWidth(1.0f);
        layer->setCircleStrokeColor(mbgl::Color{0.02f, 0.21f, 0.35f, 1.0f});
        map.getStyle().addLayer(std::move(layer));
    } else {
        source->setGeoJSON(collection);
    }
}

}  // namespace drone_swarm
