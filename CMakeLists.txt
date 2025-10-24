cmake_minimum_required(VERSION 3.26)

project(
    drone_swarm
    VERSION 0.2.0
    DESCRIPTION "Drone Swarm Simulator for macOS using C++23 and MapLibre Native"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(DRONE_SWARM_BUILD_TESTS "Build the unit tests" ON)

include(cmake/CompilerWarnings.cmake)
include(FetchContent)

# Ensure reproducible builds
set(FETCHCONTENT_QUIET OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# MapLibre Native configuration
set(MLN_WITH_CORE_ONLY ON CACHE BOOL "" FORCE)
set(MLN_WITH_GLFW OFF CACHE BOOL "" FORCE)
set(MLN_WITH_QT OFF CACHE BOOL "" FORCE)
set(MLN_WITH_NODE OFF CACHE BOOL "" FORCE)
set(MLN_WITH_METAL OFF CACHE BOOL "" FORCE)
set(MLN_WITH_OPENGL ON CACHE BOOL "" FORCE)
set(MLN_WITH_WERROR OFF CACHE BOOL "" FORCE)
set(MLN_WITH_WEBGPU OFF CACHE BOOL "" FORCE)
set(MLN_WITH_VULKAN OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)

# Third-party dependencies
FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.12.0
)

FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
)

FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG 0.9.9.8
)

FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    maplibre_native
    GIT_REPOSITORY https://github.com/maplibre/maplibre-native.git
    GIT_TAG v2.5.0
    GIT_SHALLOW OFF
)

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.5.3
)

FetchContent_MakeAvailable(spdlog nlohmann_json glm glfw maplibre_native Catch2)

add_subdirectory(src/drone_swarm)
add_subdirectory(apps/simulator)

if (DRONE_SWARM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
