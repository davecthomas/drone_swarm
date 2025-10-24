#include <atomic>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <thread>

#include <spdlog/spdlog.h>

#include "drone_swarm/configuration.hpp"
#include "drone_swarm/logging.hpp"
#include "drone_swarm/simulation_runtime.hpp"

namespace {
std::atomic<bool> should_terminate{false};

void handle_signal(int) {
    should_terminate.store(true);
}
}  // namespace

int main() {
    using namespace drone_swarm;

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    try {
        Configuration configuration = ConfigurationLoader::load(".");

        if (const char* desired_level = std::getenv("DRONE_SIM_LOG_LEVEL"); desired_level != nullptr) {
            set_log_level(desired_level);
        }

        SimulationRuntime runtime{configuration};
        runtime.initialize();
        runtime.run();

        while (!should_terminate.load()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        runtime.shutdown();
    } catch (const std::exception& exc) {
        try {
            auto logger = get_logger();
            logger->critical("Fatal error: {}", exc.what());
        } catch (const std::exception&) {
            std::cerr << "Fatal error before logger initialization: " << exc.what() << '\n';
        }
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
