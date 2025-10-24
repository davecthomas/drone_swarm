#include "drone_swarm/camera_feed.hpp"

#include <spdlog/spdlog.h>

#include "drone_swarm/logging.hpp"

namespace drone_swarm {

CameraFeed::CameraFeed(std::string identifier, GeodeticCoordinate mount_location, Orientation mount_orientation)
    : str_identifier_(std::move(identifier)),
      mount_location_(mount_location),
      mount_orientation_(mount_orientation) {}

const std::string& CameraFeed::identifier() const noexcept {
    return str_identifier_;
}

const GeodeticCoordinate& CameraFeed::mount_location() const noexcept {
    return mount_location_;
}

const Orientation& CameraFeed::mount_orientation() const noexcept {
    return mount_orientation_;
}

void CameraFeed::update_pose(GeodeticCoordinate new_location, Orientation new_orientation) {
    mount_location_ = new_location;
    mount_orientation_ = new_orientation;
}

void CameraFeed::render_frame_async() {
    auto logger = get_logger();
    logger->debug("Rendering camera frame for {}", str_identifier_);
    std::vector<std::byte> frame_buffer;
    frame_buffer.resize(1280 * 720 * 4);

    CameraFrame new_frame{};
    new_frame.captured_at = SteadyClock::now();
    new_frame.buffer = std::move(frame_buffer);
    new_frame.width_px = 1280;
    new_frame.height_px = 720;

    optional_latest_frame_ = std::move(new_frame);
}

std::optional<CameraFrame> CameraFeed::latest_frame() const {
    return optional_latest_frame_;
}

}  // namespace drone_swarm
