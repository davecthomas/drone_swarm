#pragma once

#include "drone_swarm/logging.hpp"

#include <filesystem>
#include <memory>

namespace drone_swarm::test {

inline void ensure_logger_initialized() {
    static const std::shared_ptr<spdlog::logger> logger_handle = []() {
        const auto log_dir = std::filesystem::temp_directory_path() / "drone_swarm_tests_logs";
        return drone_swarm::initialize_logger(log_dir.string());
    }();
    (void)logger_handle;
}

}  // namespace drone_swarm::test

