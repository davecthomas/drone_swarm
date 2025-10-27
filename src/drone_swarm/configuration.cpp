// === Configuration Loader ====================================================
//
// Centralizes parsing and validation of environment-driven settings that feed
// the simulation runtime. This implementation provides a narrow interface
// (`ConfigurationLoader`) that transforms raw environment variables into the
// strongly-typed `Configuration` structure consumed by downstream modules.
//
// Responsibilities
// - Enforce defaults and sane bounds for simulation knobs such as update
//   cadence, region radius, and overwatch coverage parameters.
// - Surface clear diagnostics via the logging subsystem whenever user input
//   cannot be parsed or violates expectations.
// - Shield the rest of the codebase from `std::getenv` lookups by returning a
//   fully-populated configuration object.
//
// External dependencies
// - `<cstdlib>` for environment access.
// - `drone_swarm/logging.hpp` to emit structured warnings.
// - `Configuration` types declared in the matching header.
//
// Note: This file intentionally avoids reading from disk; callers are expected
// to populate the process environment ahead of time (e.g., VS Code launch
// configuration or a shell-sourced `.env`).

#include "drone_swarm/configuration.hpp"

#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string_view>

#include "drone_swarm/logging.hpp"
#include "drone_swarm/tile_provider.hpp"

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

    config.tile_provider = make_tile_provider();
    config.update_hz = load_update_hz();

    config.orchestrator.region_center = GeodeticCoordinate{32.7473, -117.1661, 30.0};
    config.orchestrator.region_radius_m = parse_double(std::getenv("DRONE_SIM_REGION_RADIUS_M"), k_default_region_radius_m);
    config.orchestrator.update_interval = Duration{1.0 / config.update_hz};
    config.orchestrator.overwatch_height_m = parse_double(std::getenv("DRONE_SIM_OVERWATCH_HEIGHT_M"), k_default_overwatch_height_m);
    config.orchestrator.overwatch_coverage_m = parse_double(std::getenv("DRONE_SIM_OVERWATCH_COVERAGE_M"), k_default_overwatch_coverage_m);
    config.orchestrator.max_retries = parse_int(std::getenv("DRONE_SIM_MAX_RETRIES"), k_default_max_retries);

    config.rendering.window_width_px = 1920;
    config.rendering.window_height_px = 1080;
    config.rendering.enable_vsync = true;

    const TileProviderType provider_type = config.tile_provider->type();
    logger->info("Configuration loaded: tile_provider={} update_hz={} region_radius_m={} overwatch_height_m={}",
                 provider_type == TileProviderType::Mapbox ? "mapbox" : "maplibre_demo",
                 config.update_hz,
                 config.orchestrator.region_radius_m,
                 config.orchestrator.overwatch_height_m);

    return config;
}

double ConfigurationLoader::load_update_hz() {
    const double parsed_value = parse_double(std::getenv("DRONE_SIM_UPDATE_HZ"), k_default_update_hz);
    return parsed_value;
}

}  // namespace drone_swarm
