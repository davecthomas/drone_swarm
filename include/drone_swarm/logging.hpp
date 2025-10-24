#pragma once

#include <memory>
#include <spdlog/logger.h>

namespace drone_swarm {

std::shared_ptr<spdlog::logger> initialize_logger(const std::string& log_directory);

std::shared_ptr<spdlog::logger> get_logger();

void set_log_level(const std::string& str_level);

}  // namespace drone_swarm
