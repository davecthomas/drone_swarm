#include "drone_swarm/configuration.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <stdexcept>
#include <string_view>

#include "drone_swarm/logging.hpp"

namespace drone_swarm {

namespace {
constexpr double k_default_update_hz{1.0};
constexpr double k_default_region_radius_m{5000.0};
constexpr double k_default_overwatch_height_m{1500.0};
constexpr double k_default_overwatch_coverage_m{6000.0};
constexpr int k_default_max_retries{3};
constexpr std::string_view k_default_log_directory{"logs"};

double clamp_positive(double value, double fallback) {
    if (value <= 0.0) {
        return fallback;
    }
    return value;
}

double parse_double(const char* raw_value, double fallback) {
    if (raw_value == nullptr) {
        return fallback;
    }
    try {
        const double parsed_value = std::stod(raw_value);
        return clamp_positive(parsed_value, fallback);
    } catch (const std::exception&) {
        auto logger = get_logger();
        logger->warn("Failed to parse double from environment; using fallback {}", fallback);
        return fallback;
    }
}

int parse_int(const char* raw_value, int fallback) {
    if (raw_value == nullptr) {
        return fallback;
    }
    try {
        const int parsed_value = std::stoi(raw_value);
        return parsed_value <= 0 ? fallback : parsed_value;
    } catch (const std::exception&) {
        auto logger = get_logger();
        logger->warn("Failed to parse integer from environment; using fallback {}", fallback);
        return fallback;
    }
}

std::string parse_log_directory() {
    const char* raw_directory = std::getenv("DRONE_SIM_LOG_DIR");
    if (raw_directory == nullptr || std::string_view{raw_directory}.empty()) {
        return std::string{k_default_log_directory};
    }
    return std::string{raw_directory};
}

}  // namespace

Configuration ConfigurationLoader::load(const std::filesystem::path&) {
    Configuration config{};
    config.log_directory = parse_log_directory();

    auto logger = initialize_logger(config.log_directory);
    logger->info("Loading configuration from environment");

    config.tile_provider = load_tile_provider();
    const bool requires_mapbox_token = (config.tile_provider == TileProviderType::Mapbox);
    config.mapbox_token = load_mapbox_token(requires_mapbox_token);
    config.update_hz = load_update_hz();

    config.orchestrator.region_center = GeodeticCoordinate{37.7749, -122.4194, 0.0};
    config.orchestrator.region_radius_m = parse_double(std::getenv("DRONE_SIM_REGION_RADIUS_M"), k_default_region_radius_m);
    config.orchestrator.update_interval = Duration{1.0 / config.update_hz};
    config.orchestrator.overwatch_height_m = parse_double(std::getenv("DRONE_SIM_OVERWATCH_HEIGHT_M"), k_default_overwatch_height_m);
    config.orchestrator.overwatch_coverage_m = parse_double(std::getenv("DRONE_SIM_OVERWATCH_COVERAGE_M"), k_default_overwatch_coverage_m);
    config.orchestrator.max_retries = parse_int(std::getenv("DRONE_SIM_MAX_RETRIES"), k_default_max_retries);

    config.rendering.window_width_px = 1920;
    config.rendering.window_height_px = 1080;
    config.rendering.enable_vsync = true;

    logger->info("Configuration loaded: tile_provider={} update_hz={} region_radius_m={} overwatch_height_m={}",
                 config.tile_provider == TileProviderType::Mapbox ? "mapbox" : "maplibre_demo",
                 config.update_hz,
                 config.orchestrator.region_radius_m,
                 config.orchestrator.overwatch_height_m);

    return config;
}

std::string ConfigurationLoader::load_mapbox_token(bool required) {
    const char* raw_token = std::getenv("MAPBOX_ACCESS_TOKEN");
    if (raw_token == nullptr) {
        if (required) {
            throw std::runtime_error("MAPBOX_ACCESS_TOKEN not set; cannot initialize Mapbox tile service");
        }
        return {};
    }
    return std::string{raw_token};
}

double ConfigurationLoader::load_update_hz() {
    const double parsed_value = parse_double(std::getenv("DRONE_SIM_UPDATE_HZ"), k_default_update_hz);
    return parsed_value;
}

TileProviderType ConfigurationLoader::load_tile_provider() {
    const char* raw_provider = std::getenv("DRONE_SIM_TILE_PROVIDER");
    if (raw_provider == nullptr) {
        return TileProviderType::Mapbox;
    }

    std::string provider_value{raw_provider};
    std::transform(provider_value.begin(), provider_value.end(), provider_value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });

    if (provider_value == "mapbox") {
        return TileProviderType::Mapbox;
    }
    if (provider_value == "maplibre_demo" || provider_value == "maplibre" || provider_value == "demo") {
        return TileProviderType::MapLibreDemo;
    }

    throw std::runtime_error("Unsupported DRONE_SIM_TILE_PROVIDER value: " + provider_value);
}

}  // namespace drone_swarm
