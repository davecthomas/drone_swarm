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

constexpr char k_default_region_name[] = "default-region"; /**< Identifier applied to the primary operating region. */
constexpr GeodeticCoordinate k_julian_coordinate{33.0781, -116.6020, 45.0}; /**< Julian destination for demo mission. */
constexpr double k_default_drone_speed_mps{40.0};           /**< Nominal cruise speed for simulated drones. */
constexpr double k_default_drone_accel_mps2{5.0};           /**< Nominal acceleration capability. */
constexpr double k_default_turn_rate_deg_per_s{15.0};       /**< Nominal turn rate capability. */
constexpr double k_default_max_altitude_m{1'200.0};         /**< Maximum simulated altitude for demo drones. */
constexpr double k_default_min_altitude_m{30.0};            /**< Minimum safety altitude. */
constexpr double k_default_climb_rate_mps{5.0};             /**< Climb rate constraint. */
constexpr double k_default_descent_rate_mps{5.0};           /**< Descent rate constraint. */
constexpr double k_default_max_bank_angle_deg{35.0};        /**< Maximum bank angle for envelope enforcement. */
constexpr double k_default_battery_capacity_wh{450.0};      /**< Demo battery capacity. */
constexpr double k_default_consumption_wh_per_m{0.35};      /**< Demo energy consumption per metre. */
constexpr std::chrono::milliseconds k_render_sleep_duration{16}; /**< Sleep interval between render frames. */

constexpr double k_power_base_reserve_percent{10.0};        /**< Always keep a 10% buffer for contingencies. */
constexpr double k_power_reserve_per_meter_percent{0.0008}; /**< Reserve increment per metre from base. */
constexpr double k_power_reserve_per_second_percent{0.002}; /**< Reserve increment per second of return flight. */
constexpr double k_power_min_return_speed_mps{12.0};        /**< Conservative speed for return calculations. */

constexpr std::array<std::pair<double, double>, 5> k_launch_offsets_deg{{
    {0.0000, 0.0000},
    {0.0012, -0.0010},
    {-0.0014, 0.0011},
    {0.0018, 0.0015},
    {-0.0016, -0.0017},
}}; /**< Unique offsets to space out the five demo drones around base. */

/**
 * @brief Construct the Earth region config from global configuration.
 */
EarthRegionConfig make_region_config(const Configuration& configuration) {
    EarthRegionConfig config{};
    config.name = k_default_region_name;
    config.origin = configuration.orchestrator.region_center;
    config.default_tile_radius_m = configuration.orchestrator.region_radius_m;
    return config;
}

/**
 * @brief Bootstrap a demo drone with canned kinematics and power parameters.
 */
DroneAircraftPtr make_default_drone(const std::string& identifier,
                                    const GeodeticCoordinate& start_position,
                                    const GeodeticCoordinate& base_location,
                                    const GeodeticCoordinate& destination) {
    VehicleKinematics kinematics{};
    kinematics.max_speed_mps = k_default_drone_speed_mps;
    kinematics.max_acceleration_mps2 = k_default_drone_accel_mps2;
    kinematics.turn_rate_deg_per_s = k_default_turn_rate_deg_per_s;

    FlightEnvelope envelope{};
    envelope.max_altitude_m = k_default_max_altitude_m;
    envelope.min_altitude_m = k_default_min_altitude_m;
    envelope.max_climb_rate_mps = k_default_climb_rate_mps;
    envelope.max_descent_rate_mps = k_default_descent_rate_mps;
    envelope.max_bank_angle_deg = k_default_max_bank_angle_deg;

    PowertrainModel powertrain{};
    powertrain.base_reserve_percent = k_power_base_reserve_percent;
    powertrain.reserve_percent_per_meter = k_power_reserve_per_meter_percent;
    powertrain.reserve_percent_per_second = k_power_reserve_per_second_percent;
    powertrain.minimum_return_speed_mps = k_power_min_return_speed_mps;

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
        base_location,
        powertrain,
        k_default_battery_capacity_wh,
        k_default_consumption_wh_per_m,
        camera_feeds
    );

    std::vector<Waypoint> waypoints{};
    waypoints.push_back(Waypoint{identifier + "-wp1", GeodeticCoordinate{(start_position.latitude_deg + destination.latitude_deg) * 0.5,
                                                                         (start_position.longitude_deg + destination.longitude_deg) * 0.5,
                                                                         destination.altitude_m}, 0.0});
    waypoints.push_back(Waypoint{identifier + "-wp2", destination, 0.0});
    waypoints.push_back(Waypoint{identifier + "-wp3", base_location, 0.0});
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

/**
 * @brief Prepare world assets, rendering pipeline, and demo vehicles.
 */
void SimulationRuntime::initialize() {
    logger_->info("Initializing simulation runtime");
    earth_region_.initialize_tiles();
    rendering_pipeline_.initialize();

    const GeodeticCoordinate base_location = configuration_.orchestrator.region_center;

    for (std::size_t index = 0; index < k_launch_offsets_deg.size(); ++index) {
        const auto [lat_offset, lon_offset] = k_launch_offsets_deg[index];
        GeodeticCoordinate launch_coordinate{
            base_location.latitude_deg + lat_offset,
            base_location.longitude_deg + lon_offset,
            base_location.altitude_m
        };
        const std::string drone_name = "drone-" + std::to_string(index + 1);
        list_vehicles_.push_back(make_default_drone(drone_name, launch_coordinate, base_location, k_julian_coordinate));
    }
}

/**
 * @brief Start the background update and render threads.
 */
void SimulationRuntime::run() {
    if (flag_running_.exchange(true)) {
        return;
    }
    logger_->info("Starting simulation run loop");
    update_thread_ = std::thread(&SimulationRuntime::update_loop, this);
    render_thread_ = std::thread(&SimulationRuntime::render_loop, this);
}

/**
 * @brief Stop worker threads and tear down rendering infrastructure.
 */
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

/**
 * @brief Fixed-timestep simulation loop that advances vehicle state and orchestrator logic.
 */
void SimulationRuntime::update_loop() {
    const Duration tick_interval{1.0 / configuration_.update_hz};
    const SteadyClock::duration steady_tick_interval = std::chrono::duration_cast<SteadyClock::duration>(tick_interval);
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
        next_tick = now + steady_tick_interval;
    }
}

/**
 * @brief Render loop that gathers vehicle state and drives the MapLibre pipeline.
 */
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
        std::this_thread::sleep_for(k_render_sleep_duration);
    }
}

}  // namespace drone_swarm
