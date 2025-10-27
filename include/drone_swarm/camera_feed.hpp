// === Camera Feed =============================================================
//
// Declares lightweight structures for camera orientation/frame metadata plus
// the `CameraFeed` class that simulates an onboard sensor. RenderingPipeline
// consumes these feeds to display point-of-view windows for selected drones.

#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "drone_swarm/types.hpp"

namespace drone_swarm {

/** @brief Euler orientation used to mount gimbals on the airframe. */
struct Orientation final {
    double pitch_deg{};
    double yaw_deg{};
    double roll_deg{};
};

/** @brief Captured frame buffer plus metadata for simulated cameras. */
struct CameraFrame final {
    TimePoint captured_at{SteadyClock::now()};
    std::vector<std::byte> buffer{};
    int width_px{};
    int height_px{};
};

/** @brief Represents a single simulated camera mounted on a vehicle. */
class CameraFeed final {
  public:
    CameraFeed(std::string identifier, GeodeticCoordinate mount_location, Orientation mount_orientation);

    /** @brief Unique identifier for UI presentation. */
    [[nodiscard]] const std::string& identifier() const noexcept;
    /** @brief Mounting location on the host vehicle. */
    [[nodiscard]] const GeodeticCoordinate& mount_location() const noexcept;
    /** @brief Current orientation of the camera mount. */
    [[nodiscard]] const Orientation& mount_orientation() const noexcept;

    /** @brief Update the camera pose after the host vehicle moves. */
    void update_pose(GeodeticCoordinate new_location, Orientation new_orientation);

    /** @brief Launch asynchronous rendering for the next frame (stub impl). */
    void render_frame_async();
    /** @brief Retrieve the latest rendered frame if available. */
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
