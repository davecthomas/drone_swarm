#pragma once

#include <memory>

// Force spdlog to use its bundled fmt implementation. Some build modes define
// SPDLOG_FMT_EXTERNAL implicitly, which results in missing <fmt/core.h>
// headers when the external dependency is not present. Undefining the macro
// before including spdlog headers guarantees consistent behaviour across
// translation units.
#ifdef SPDLOG_FMT_EXTERNAL
#undef SPDLOG_FMT_EXTERNAL
#endif

#include <spdlog/logger.h>

namespace drone_swarm {

std::shared_ptr<spdlog::logger> initialize_logger(const std::string& log_directory);

std::shared_ptr<spdlog::logger> get_logger();

void set_log_level(const std::string& str_level);

}  // namespace drone_swarm
