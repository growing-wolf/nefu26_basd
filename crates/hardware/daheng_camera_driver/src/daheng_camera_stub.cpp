/**
 * @brief Daheng camera stub implementation (SDK not available)
 */
#include "daheng_camera.hpp"

namespace dahengcamera {

std::string error_code_to_message(int code) noexcept {
    return fmt::format("unsupported (error code: {})", code);
}

class ImageCapturer::Impl {};

ImageCapturer::ImageCapturer() noexcept
    : impl_(nullptr) {}

ImageCapturer::~ImageCapturer() noexcept = default;

std::expected<std::unique_ptr<ImageCapturer>, CameraError> ImageCapturer::create(
    const CameraProfile& /*profile*/, const char* /*user_defined_name*/) noexcept {
    return std::unexpected("Daheng SDK not available (stub build)");
}

std::expected<void, CameraError> ImageCapturer::init(
    const CameraProfile& /*profile*/, const char* /*user_defined_name*/) noexcept {
    return std::unexpected("Daheng SDK not available (stub build)");
}

std::expected<cv::Mat, CameraError>
    ImageCapturer::read(std::chrono::duration<unsigned int, std::micro> /*timeout*/) noexcept {
    return std::unexpected("Daheng SDK not available (stub build)");
}

std::expected<std::tuple<int, int>, CameraError> ImageCapturer::get_width_height() const noexcept {
    return std::unexpected("Daheng SDK not available (stub build)");
}

std::expected<void, CameraError> ImageCapturer::software_trigger_on() noexcept {
    return std::unexpected("Daheng SDK not available (stub build)");
}

std::expected<void, CameraError>
    ImageCapturer::set_frame_rate_inner_trigger_mode(float /*frame_rate*/) noexcept {
    return std::unexpected("Daheng SDK not available (stub build)");
}

std::expected<void, CameraError> ImageCapturer::stop_grabbing() noexcept {
    return std::unexpected("Daheng SDK not available (stub build)");
}

bool ImageCapturer::valid() const noexcept { return false; }

} // namespace dahengcamera
