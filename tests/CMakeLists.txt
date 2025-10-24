add_executable(drone_swarm_tests
    test_drone_state.cpp
    test_overwatch.cpp
)

target_link_libraries(drone_swarm_tests PRIVATE drone_swarm_lib Catch2::Catch2WithMain)

enable_project_warnings(drone_swarm_tests)

add_test(NAME drone_swarm_tests COMMAND drone_swarm_tests)
