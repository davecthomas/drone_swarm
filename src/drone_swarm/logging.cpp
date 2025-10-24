#include "drone_swarm/logging.hpp"

#include <filesystem>
#include <mutex>
#include <stdexcept>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace drone_swarm {

namespace {
std::once_flag logger_once_flag;
std::shared_ptr<spdlog::logger> shared_logger;
constexpr std::size_t k_max_file_size_bytes{10 * 1024 * 1024};
constexpr std::size_t k_max_files{5};
}  // namespace

std::shared_ptr<spdlog::logger> initialize_logger(const std::string& log_directory) {
    std::call_once(
        logger_once_flag,
        [&log_directory]() {
            const std::filesystem::path path_log_dir{log_directory};
            std::error_code error_directory;
            std::filesystem::create_directories(path_log_dir, error_directory);
            if (error_directory) {
                throw std::runtime_error("Unable to create log directory at " + path_log_dir.string());
            }

            const std::filesystem::path path_log_file = path_log_dir / "drone_swarm.log";

            auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
            console_sink->set_pattern("[%l] %v");
            auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
                path_log_file.string(),
                k_max_file_size_bytes,
                k_max_files
            );
            file_sink->set_pattern(R"({"ts":"%Y-%m-%dT%H:%M:%S.%eZ","level":"%l","msg":%v})");

            spdlog::sinks_init_list sinks{console_sink, file_sink};
            shared_logger = std::make_shared<spdlog::logger>("drone_swarm", sinks);
            shared_logger->set_level(spdlog::level::info);
            spdlog::register_logger(shared_logger);
        }
    );
    return shared_logger;
}

std::shared_ptr<spdlog::logger> get_logger() {
    if (!shared_logger) {
        throw std::runtime_error("Logger not initialized");
    }
    return shared_logger;
}

void set_log_level(const std::string& str_level) {
    if (!shared_logger) {
        return;
    }
    try {
        const auto level = spdlog::level::from_str(str_level);
        shared_logger->set_level(level);
    } catch (const std::exception&) {
        shared_logger->warn("Unknown log level {}; defaulting to info", str_level);
        shared_logger->set_level(spdlog::level::info);
    }
}

}  // namespace drone_swarm
