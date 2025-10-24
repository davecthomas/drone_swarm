add_executable(drone_swarm_app main.cpp)

target_link_libraries(drone_swarm_app PRIVATE drone_swarm_lib)

enable_project_warnings(drone_swarm_app)
