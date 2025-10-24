#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "drone_swarm/types.hpp"

namespace drone_swarm {

struct Orientation final {
    double pitch_deg{};
    double yaw_deg{};
    double roll_deg{};
};

struct CameraFrame final {
    TimePoint captured_at{SteadyClock::now()};
    std::vector<std::byte> buffer{};
    int width_px{};
    int height_px{};
};

class CameraFeed final {
  public:
    CameraFeed(std::string identifier, GeodeticCoordinate mount_location, Orientation mount_orientation);

    [[nodiscard]] const std::string& identifier() const noexcept;
    [[nodiscard]] const GeodeticCoordinate& mount_location() const noexcept;
    [[nodiscard]] const Orientation& mount_orientation() const noexcept;

    void update_pose(GeodeticCoordinate new_location, Orientation new_orientation);

    void render_frame_async();
    [[nodiscard]] std::optional<CameraFrame> latest_frame() const;

  private:
    std::string str_identifier_;
    GeodeticCoordinate mount_location_;
    Orientation mount_orientation_;
    std::optional<CameraFrame> optional_latest_frame_;
};

using CameraFeedPtr = std::shared_ptr<CameraFeed>;
using CameraFeedList = std::vector<CameraFeedPtr>;

}  // namespace drone_swarm
