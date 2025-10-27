set(DRONE_SWARM_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include/drone_swarm/version.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include/drone_swarm/configuration.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include/drone_swarm/types.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include/drone_swarm/drone_state.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include/drone_swarm/camera_feed.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include/drone_swarm/autonomous_vehicle.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include/drone_swarm/drone_aircraft.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include/drone_swarm/telemetry_bus.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include/drone_swarm/overwatch_node.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include/drone_swarm/overwatch_orchestrator.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include/drone_swarm/earth_region.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include/drone_swarm/rendering_pipeline.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include/drone_swarm/simulation_runtime.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include/drone_swarm/logging.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include/drone_swarm/tile_service_factory.hpp
)

set(DRONE_SWARM_SOURCES
    configuration.cpp
    drone_state.cpp
    camera_feed.cpp
    autonomous_vehicle.cpp
    drone_aircraft.cpp
    telemetry_bus.cpp
    overwatch_node.cpp
    overwatch_orchestrator.cpp
    earth_region.cpp
    rendering_pipeline.cpp
    simulation_runtime.cpp
    logging.cpp
    tile_service_factory.cpp
)

add_library(drone_swarm_lib ${DRONE_SWARM_HEADERS} ${DRONE_SWARM_SOURCES})

target_include_directories(
    drone_swarm_lib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(
    drone_swarm_lib
    PUBLIC
        spdlog::spdlog
        nlohmann_json::nlohmann_json
        glm
        mbgl-core
        MapLibreNative::Base
        glfw
)

enable_project_warnings(drone_swarm_lib)

target_compile_features(drone_swarm_lib PUBLIC cxx_std_23)
