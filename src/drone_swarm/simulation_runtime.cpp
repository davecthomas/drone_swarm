#include "drone_swarm/simulation_runtime.hpp"

#include <chrono>
#include <filesystem>
#include <stdexcept>
#include <thread>

#include <spdlog/spdlog.h>

#include "drone_swarm/drone_aircraft.hpp"
#include "drone_swarm/logging.hpp"

namespace drone_swarm {

namespace {

EarthRegionConfig make_region_config(const Configuration& configuration) {
    EarthRegionConfig config{};
    config.name = "default-region";
    config.origin = configuration.orchestrator.region_center;
    config.default_tile_radius_m = configuration.orchestrator.region_radius_m;
    return config;
}

DroneAircraftPtr make_default_drone(const std::string& identifier, const GeodeticCoordinate& start_position) {
    VehicleKinematics kinematics{};
    kinematics.max_speed_mps = 40.0;
    kinematics.max_acceleration_mps2 = 5.0;
    kinematics.turn_rate_deg_per_s = 15.0;

    FlightEnvelope envelope{};
    envelope.max_altitude_m = 1200.0;
    envelope.min_altitude_m = 30.0;
    envelope.max_climb_rate_mps = 5.0;
    envelope.max_descent_rate_mps = 5.0;
    envelope.max_bank_angle_deg = 35.0;

    CameraFeedList camera_feeds{};
    camera_feeds.push_back(std::make_shared<CameraFeed>(
        identifier + "-cam-primary",
        start_position,
        Orientation{0.0, 0.0, 0.0}
    ));

    auto drone = std::make_shared<DroneAircraft>(
        identifier,
        kinematics,
        envelope,
        start_position,
        450.0,
        0.35,
        camera_feeds
    );

    std::vector<Waypoint> waypoints{};
    waypoints.push_back(Waypoint{identifier + "-wp1", GeodeticCoordinate{start_position.latitude_deg + 0.01, start_position.longitude_deg, start_position.altitude_m}, 0.0});
    waypoints.push_back(Waypoint{identifier + "-wp2", GeodeticCoordinate{start_position.latitude_deg, start_position.longitude_deg + 0.01, start_position.altitude_m}, 0.0});
    waypoints.push_back(Waypoint{identifier + "-wp3", start_position, 0.0});
    drone->set_waypoints(waypoints);

    return drone;
}

}  // namespace

SimulationRuntime::SimulationRuntime(Configuration configuration)
    : configuration_(std::move(configuration)),
      earth_region_(make_region_config(configuration_)),
      telemetry_bus_(),
      orchestrator_("primary", configuration_.orchestrator, telemetry_bus_),
      rendering_pipeline_(configuration_.rendering,
                          configuration_.mapbox_token,
                          configuration_.tile_provider,
                          std::filesystem::path(configuration_.log_directory) / "map_cache"),
      logger_(get_logger()) {}

SimulationRuntime::~SimulationRuntime() {
    shutdown();
}

void SimulationRuntime::initialize() {
    logger_->info("Initializing simulation runtime");
    earth_region_.initialize_tiles();
    rendering_pipeline_.initialize();

    const GeodeticCoordinate start_position = configuration_.orchestrator.region_center;
    list_vehicles_.push_back(make_default_drone("drone-alpha", start_position));
    list_vehicles_.push_back(make_default_drone("drone-bravo", GeodeticCoordinate{start_position.latitude_deg + 0.02, start_position.longitude_deg - 0.02, start_position.altitude_m}));
}

void SimulationRuntime::run() {
    if (flag_running_.exchange(true)) {
        return;
    }
    logger_->info("Starting simulation run loop");
    update_thread_ = std::thread(&SimulationRuntime::update_loop, this);
    render_thread_ = std::thread(&SimulationRuntime::render_loop, this);
}

void SimulationRuntime::shutdown() {
    if (!flag_running_.exchange(false)) {
        return;
    }
    logger_->info("Shutting down simulation runtime");
    if (update_thread_.joinable()) {
        update_thread_.join();
    }
    if (render_thread_.joinable()) {
        render_thread_.join();
    }
    rendering_pipeline_.shutdown();
}

void SimulationRuntime::update_loop() {
    const Duration tick_interval{1.0 / configuration_.update_hz};
    auto next_tick = SteadyClock::now();
    while (flag_running_.load()) {
        const TimePoint now = SteadyClock::now();
        if (now < next_tick) {
            std::this_thread::sleep_for(next_tick - now);
            continue;
        }
        try {
            for (const AutonomousVehiclePtr& vehicle : list_vehicles_) {
                if (vehicle == nullptr) {
                    continue;
                }
                vehicle->update(tick_interval, telemetry_bus_);
            }
            orchestrator_.update(now);
        } catch (const std::exception& exc) {
            logger_->error("Update loop error: {}", exc.what());
        }
        next_tick = now + tick_interval;
    }
}

void SimulationRuntime::render_loop() {
    while (flag_running_.load()) {
        std::vector<DroneState> visible_states;
        visible_states.reserve(list_vehicles_.size());
        for (const AutonomousVehiclePtr& vehicle : list_vehicles_) {
            if (vehicle == nullptr) {
                continue;
            }
            visible_states.push_back(vehicle->state());
        }

        std::optional<CameraFrame> selected_frame;
        if (!list_vehicles_.empty()) {
            const AutonomousVehiclePtr& selected_vehicle = list_vehicles_.front();
            if (selected_vehicle != nullptr && !selected_vehicle->camera_feeds().empty()) {
                selected_vehicle->camera_feeds().front()->render_frame_async();
                selected_frame = selected_vehicle->camera_feeds().front()->latest_frame();
            }
        }

        rendering_pipeline_.render_scene(visible_states, selected_frame);
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
}

}  // namespace drone_swarm
