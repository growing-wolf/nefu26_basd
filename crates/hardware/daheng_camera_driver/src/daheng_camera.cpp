#include "daheng_camera.hpp"

#include "GxIAPI.h"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <spdlog/spdlog.h>

namespace dahengcamera {

std::string error_code_to_message(int code) noexcept {
    switch (code) {
    case GX_STATUS_SUCCESS: return "Success";

    case GX_STATUS_ERROR: return "Unspecified internal error";
    case GX_STATUS_NOT_FOUND_TL:
        return "Transport layer library not found (check GxU3VTL.cti / GxGVTL.cti)";
    case GX_STATUS_NOT_FOUND_DEVICE: return "Device not found";
    case GX_STATUS_OFFLINE: return "Device is offline";
    case GX_STATUS_INVALID_PARAMETER: return "Invalid parameter";
    case GX_STATUS_INVALID_HANDLE: return "Invalid handle";
    case GX_STATUS_INVALID_CALL: return "Invalid call (interface logic error)";
    case GX_STATUS_INVALID_ACCESS:
        return "Feature currently inaccessible or wrong device access mode";
    case GX_STATUS_NEED_MORE_BUFFER: return "User buffer is too small";
    case GX_STATUS_ERROR_TYPE: return "Wrong feature API type for feature ID";
    case GX_STATUS_OUT_OF_RANGE: return "Value out of range";
    case GX_STATUS_NOT_IMPLEMENTED: return "Feature not implemented";
    case GX_STATUS_NOT_INIT_API: return "GxIAPI not initialized";
    case GX_STATUS_TIMEOUT: return "Timeout";

    default: return fmt::format("Unknown error code: {}", code);
    }
}

namespace {

constexpr uint64_t kAcquisitionBufferNumber = 6;

/// Bounded string view over a fixed-size C string field of the SDK structs.
std::string_view fixed_string_view(const char* data, std::size_t capacity) noexcept {
    const auto* end = static_cast<const char*>(std::memchr(data, '\0', capacity));
    return {data, static_cast<std::size_t>(end == nullptr ? capacity : end - data)};
}

template <std::size_t N>
std::string_view fixed_string_view(const char (&value)[N]) noexcept {
    return fixed_string_view(value, N);
}

/// Human-readable valid range for a float feature, appended to "out of range"
/// errors so a bad config value is self-diagnosing.
std::string float_range_suffix(GX_DEV_HANDLE handle, GX_FEATURE_ID_CMD feature) noexcept {
    GX_FLOAT_RANGE range{};
    if (GXGetFloatRange(handle, feature, &range) != GX_STATUS_SUCCESS) {
        return {};
    }
    const auto unit = fixed_string_view(range.szUnit);
    if (unit.empty()) {
        return fmt::format(" (valid range: [{}, {}])", range.dMin, range.dMax);
    }
    return fmt::format(" (valid range: [{}, {}] {})", range.dMin, range.dMax, unit);
}

/// Set a float feature, including its valid range in the error message when the
/// camera rejects the value.
std::expected<void, CameraError> set_float_feature(
    GX_DEV_HANDLE handle, GX_FEATURE_ID_CMD feature, double value, std::string_view name) noexcept {
    const auto ret = GXSetFloat(handle, feature, value);
    if (ret == GX_STATUS_SUCCESS) {
        return {};
    }
    return std::unexpected(
        CameraError{
            fmt::format("set {} to {:g}{}", name, value, float_range_suffix(handle, feature)),
            static_cast<int>(ret)});
}

std::string format_device_info(const GX_DEVICE_BASE_INFO& info) {
    return fmt::format(
        "model={}, sn={}, display_name={}, device_id={}", fixed_string_view(info.szModelName),
        fixed_string_view(info.szSN), fixed_string_view(info.szDisplayName),
        fixed_string_view(info.szDeviceID));
}

bool device_matches(const GX_DEVICE_BASE_INFO& info, const char* name) noexcept {
    if (name == nullptr) {
        return false;
    }
    const std::string_view target{name};
    return fixed_string_view(info.szDisplayName) == target
        || fixed_string_view(info.szModelName) == target || fixed_string_view(info.szSN) == target
        || fixed_string_view(info.szUserID) == target;
}

/// Enumerate all connected Daheng devices (GXUpdateDeviceList must run first).
std::expected<std::vector<GX_DEVICE_BASE_INFO>, CameraError> list_devices() noexcept {
    uint32_t device_count = 0;
    GX_STATUS ret         = GXUpdateDeviceList(&device_count, 1000);
    if (ret != GX_STATUS_SUCCESS) {
        return std::unexpected(CameraError{"enumerate devices", ret});
    }
    if (device_count == 0) {
        return std::unexpected(CameraError{"no daheng camera found"});
    }

    size_t buffer_size = 0;
    ret                = GXGetAllDeviceBaseInfo(nullptr, &buffer_size);
    if (ret != GX_STATUS_SUCCESS) {
        return std::unexpected(CameraError{"query device info size", ret});
    }

    const auto struct_size = sizeof(GX_DEVICE_BASE_INFO);
    std::vector<GX_DEVICE_BASE_INFO> devices((buffer_size + struct_size - 1) / struct_size);
    size_t actual_size = devices.size() * struct_size;
    ret                = GXGetAllDeviceBaseInfo(devices.data(), &actual_size);
    if (ret != GX_STATUS_SUCCESS) {
        return std::unexpected(CameraError{"get device info", ret});
    }
    devices.resize(actual_size / struct_size);
    return devices;
}

/// Select the target device by name (display name / model / SN / user ID) or,
/// when the choice is unambiguous, the only connected camera.
std::expected<GX_DEVICE_BASE_INFO, CameraError>
    search_device(const char* user_defined_name) noexcept {
    auto devices = list_devices();
    if (!devices) {
        return std::unexpected(devices.error());
    }

    SPDLOG_INFO("found {} daheng device(s):", devices->size());
    for (const auto& device : *devices) {
        SPDLOG_INFO("  {}", format_device_info(device));
    }

    if (user_defined_name == nullptr) {
        if (devices->size() > 1) {
            return std::unexpected(
                CameraError{fmt::format(
                    "must pass in the device name because {} devices were found",
                    devices->size())});
        }
        return devices->front();
    }

    for (const auto& device : *devices) {
        if (device_matches(device, user_defined_name)) {
            return device;
        }
    }
    return std::unexpected(
        CameraError{fmt::format(
            "{} devices were found, but no device matches the name passed in: {}", devices->size(),
            user_defined_name)});
}

/// Convert the SDK frame to BGR using OpenCV's demosaicing.
///
/// OpenCV names a Bayer pattern by the components in the *second* row, second
/// and third columns. A sensor whose first row starts with R (vendor "RG") is
/// therefore `COLOR_BayerBG2BGR` in OpenCV — hence the RG<->BG, GR<->GB swap.
std::expected<cv::Mat, CameraError> convert_to_bgr(const GX_FRAME_BUFFER& frame) noexcept {
    if (frame.pImgBuf == nullptr || frame.nWidth <= 0 || frame.nHeight <= 0) {
        return std::unexpected(
            CameraError{fmt::format(
                "invalid frame: buf={}, width={}, height={}", frame.pImgBuf, frame.nWidth,
                frame.nHeight)});
    }

    const int width  = frame.nWidth;
    const int height = frame.nHeight;

    const auto demosaic = [&frame, width, height](int code) {
        const cv::Mat raw{height, width, CV_8UC1, frame.pImgBuf};
        cv::Mat bgr;
        cv::cvtColor(raw, bgr, code);
        return bgr;
    };

    switch (frame.nPixelFormat) {
    case GX_PIXEL_FORMAT_BAYER_RG8: return demosaic(cv::COLOR_BayerBG2BGR_EA);
    case GX_PIXEL_FORMAT_BAYER_GR8: return demosaic(cv::COLOR_BayerGB2BGR_EA);
    case GX_PIXEL_FORMAT_BAYER_GB8: return demosaic(cv::COLOR_BayerGR2BGR_EA);
    case GX_PIXEL_FORMAT_BAYER_BG8: return demosaic(cv::COLOR_BayerRG2BGR_EA);
    case GX_PIXEL_FORMAT_RGB8: {
        const cv::Mat raw{height, width, CV_8UC3, frame.pImgBuf};
        cv::Mat bgr;
        cv::cvtColor(raw, bgr, cv::COLOR_RGB2BGR);
        return bgr;
    }
    case GX_PIXEL_FORMAT_BGR8: {
        const cv::Mat raw{height, width, CV_8UC3, frame.pImgBuf};
        return raw.clone();
    }
    case GX_PIXEL_FORMAT_MONO8: {
        const cv::Mat raw{height, width, CV_8UC1, frame.pImgBuf};
        cv::Mat bgr;
        cv::cvtColor(raw, bgr, cv::COLOR_GRAY2BGR);
        return bgr;
    }
    default:
        return std::unexpected(
            CameraError{fmt::format("unsupported pixel format: 0x{:08X}", frame.nPixelFormat)});
    }
}

// SDK 全局初始化实现
// 使用 Meyer's Singleton 确保线程安全和正确的初始化顺序。
//
// 注意：这里刻意不在静态析构时调用 GXCloseLib。大恒 SDK 的 GXCloseLib 在
// 进程退出阶段（静态析构/atexit 之后）调用会在内部 CGXIError::SetErrTxt 中
// 触发 double free 导致 abort（实测 1.0.1904.8241）。GxIAPI 保持初始化到
// 进程结束由 OS 回收，是安全的；设备句柄本身仍由 Impl 析构时的 GXCloseDevice
// 正常关闭。
class SDKInitializer {
public:
    static SDKInitializer& instance() noexcept {
        static SDKInitializer init;
        return init;
    }

    std::expected<void, CameraError> initialize() noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        if (initialized_) {
            return {};
        }

        const auto ret = GXInitLib();
        if (ret != GX_STATUS_SUCCESS) {
            return std::unexpected(CameraError{"init SDK", ret});
        }

        initialized_ = true;
        return {};
    }

private:
    SDKInitializer() noexcept  = default;
    ~SDKInitializer() noexcept = default;

    SDKInitializer(const SDKInitializer&)            = delete;
    SDKInitializer& operator=(const SDKInitializer&) = delete;

    std::mutex mutex_;
    bool initialized_ = false;
};

} // namespace

#define SDK_RET_ASSERT(expr, message)                                                  \
    do {                                                                               \
        const auto sdk_ret = (expr);                                                   \
        if (sdk_ret != GX_STATUS_SUCCESS) {                                            \
            return std::unexpected(CameraError{(message), static_cast<int>(sdk_ret)}); \
        }                                                                              \
    } while (false)

class ImageCapturer::Impl {
public:
    std::expected<void, CameraError>
        init(const CameraProfile& profile, const char* user_defined_name) noexcept {
        auto init_sdk_result = SDKInitializer::instance().initialize();
        if (!init_sdk_result) {
            return std::unexpected(fmt::format("init SDK: {}", init_sdk_result.error()));
        }

        SPDLOG_INFO("searching daheng camera..");
        auto device = search_device(user_defined_name);
        if (!device) {
            return std::unexpected(fmt::format("search camera: {}", device.error()));
        }

        SPDLOG_INFO("initializing daheng camera: {}", format_device_info(*device));
        auto init_result = init_camera(*device, profile);
        if (!init_result) {
            return std::unexpected(fmt::format("init camera: {}", init_result.error()));
        }
        return {};
    }

    ~Impl() noexcept { uninit_camera(); }

    std::tuple<int, int> get_image_size() const noexcept {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        return {image_width_, image_height_};
    }

    std::expected<cv::Mat, CameraError>
        read(std::chrono::duration<unsigned int, std::micro> timeout) noexcept {
        std::lock_guard<std::mutex> lock(camera_mutex_);

        if (handle_ == nullptr) {
            return std::unexpected(CameraError{"get frame buffer: camera handle is null"});
        }
        if (!valid_.load(std::memory_order_acquire)) {
            return std::unexpected(CameraError{"get frame buffer: camera is not valid"});
        }
        if (!grabbing_) {
            return std::unexpected(CameraError{"get frame buffer: camera is not grabbing"});
        }

        // GXDQBuf takes milliseconds; round up so sub-millisecond timeouts still block.
        const auto timeout_ms = static_cast<uint32_t>(
            std::max<std::uint64_t>(1, (static_cast<std::uint64_t>(timeout.count()) + 999) / 1000));

        PGX_FRAME_BUFFER frame_buffer = nullptr;
        const auto ret                = GXDQBuf(handle_, &frame_buffer, timeout_ms);
        if (ret != GX_STATUS_SUCCESS) {
            if (ret == GX_STATUS_TIMEOUT) {
                return std::unexpected(CameraError{"get frame buffer: timeout", ret});
            }
            // Any other stream error means the link is gone; let the caller
            // (re)create the capturer.
            valid_.store(false, std::memory_order_release);
            return std::unexpected(CameraError{"get frame buffer", ret});
        }
        if (frame_buffer == nullptr) {
            return std::unexpected(CameraError{"get frame buffer: null frame buffer"});
        }

        // Return the buffer to the SDK exactly once, including on error paths.
        struct FrameBufferGuard {
            GX_DEV_HANDLE handle;
            PGX_FRAME_BUFFER frame;

            ~FrameBufferGuard() noexcept { (void)GXQBuf(handle, frame); }
        } buffer_guard{handle_, frame_buffer};

        if (frame_buffer->nStatus != GX_FRAME_STATUS_SUCCESS) {
            return std::unexpected(
                CameraError{fmt::format(
                    "abnormal frame status {} (frame id {})", frame_buffer->nStatus,
                    frame_buffer->nFrameID)});
        }

        try {
            auto converted = convert_to_bgr(*frame_buffer);
            if (!converted) {
                return std::unexpected(converted.error());
            }

            cv::Mat img = std::move(*converted);
            switch (rotate_type_) {
            case RotateType::None: break;
            case RotateType::Clockwise90: cv::rotate(img, img, cv::ROTATE_90_CLOCKWISE); break;
            case RotateType::Clockwise180: cv::rotate(img, img, cv::ROTATE_180); break;
            case RotateType::Clockwise270:
                cv::rotate(img, img, cv::ROTATE_90_COUNTERCLOCKWISE);
                break;
            }

            image_width_  = frame_buffer->nWidth;
            image_height_ = frame_buffer->nHeight;
            return img;
        } catch (const cv::Exception& e) {
            return std::unexpected(fmt::format("convert image: {}", e.what()));
        } catch (const std::exception& e) {
            return std::unexpected(fmt::format("convert image: {}", e.what()));
        }
    }

    std::expected<void, CameraError> software_trigger_on() noexcept {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        if (handle_ == nullptr) {
            return std::unexpected(CameraError{"send software trigger: camera handle is null"});
        }
        if (!valid_.load(std::memory_order_acquire)) {
            return std::unexpected(CameraError{"send software trigger: camera is not valid"});
        }
        if (!grabbing_) {
            return std::unexpected(CameraError{"send software trigger: camera is not grabbing"});
        }
        SDK_RET_ASSERT(
            GXSendCommand(handle_, GX_COMMAND_TRIGGER_SOFTWARE), "send software trigger");
        return {};
    }

    std::expected<void, CameraError> set_frame_rate_inner_trigger_mode(float frame_rate) noexcept {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        if (handle_ == nullptr) {
            return std::unexpected(CameraError{"set frame rate: camera handle is null"});
        }
        if (!valid_.load(std::memory_order_acquire)) {
            return std::unexpected(CameraError{"set frame rate: camera is not valid"});
        }

        // 在触发模式下，帧率由触发信号控制，AcquisitionFrameRate 设置无效
        if (trigger_mode_) {
            return std::unexpected("set frame rate in trigger mode");
        }

        SDK_RET_ASSERT(
            GXSetEnum(
                handle_, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ACQUISITION_FRAME_RATE_MODE_ON),
            "set frame rate control enable");
        SDK_RET_ASSERT(
            GXSetFloat(handle_, GX_FLOAT_ACQUISITION_FRAME_RATE, frame_rate), "set frame rate");
        return {};
    }

    std::expected<void, CameraError> stop_grabbing() noexcept {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        if (handle_ == nullptr) {
            return {};
        }
        if (!grabbing_) {
            return {};
        }
        const auto ret = GXStreamOff(handle_);
        if (ret != GX_STATUS_SUCCESS) {
            return std::unexpected(CameraError{"stop grabbing", ret});
        }
        grabbing_ = false;
        valid_.store(false, std::memory_order_release);
        return {};
    }

    bool valid() const noexcept { return valid_.load(std::memory_order_acquire); }

private:
    RotateType rotate_type_ = RotateType::None;

    // 缓存图像尺寸（init 时查询，之后随每帧更新）
    int image_width_  = 0;
    int image_height_ = 0;

    // 记录当前是否处于触发模式（在 init_camera 时确定）
    bool trigger_mode_ = false;
    bool grabbing_     = false;

    GX_DEV_HANDLE handle_ = nullptr;
    mutable std::mutex camera_mutex_;
    std::atomic_bool valid_{false};

    std::expected<void, CameraError>
        init_camera(const GX_DEVICE_BASE_INFO& device, const CameraProfile& profile) noexcept {
        rotate_type_ = profile.rotate_type;

        const auto serial = fixed_string_view(device.szSN);
        GX_STATUS ret     = GX_STATUS_SUCCESS;
        if (!serial.empty()) {
            GX_OPEN_PARAM open_param{};
            open_param.pszContent = const_cast<char*>(serial.data());
            open_param.openMode   = GX_OPEN_SN;
            open_param.accessMode = GX_ACCESS_EXCLUSIVE;
            ret                   = GXOpenDevice(&open_param, &handle_);
        } else {
            ret = GXOpenDeviceByIndex(1, &handle_);
        }
        SDK_RET_ASSERT(ret, "open device");

        ret = GXSetAcqusitionBufferNumber(handle_, kAcquisitionBufferNumber);
        if (ret != GX_STATUS_SUCCESS) {
            SPDLOG_WARN("set acquisition buffer number: {}", error_code_to_message(ret));
        }

        // 8-bit Bayer RG (RGGB) is the native format of the MER2 color series
        // and the cheapest to transfer. read() still handles the other common
        // formats if the camera refuses this one.
        ret = GXSetEnum(handle_, GX_ENUM_PIXEL_FORMAT, GX_PIXEL_FORMAT_BAYER_RG8);
        if (ret != GX_STATUS_SUCCESS) {
            SPDLOG_WARN("set pixel format to BayerRG8: {}", error_code_to_message(ret));
        }

        trigger_mode_ = profile.trigger_mode;
        SDK_RET_ASSERT(
            GXSetEnum(
                handle_, GX_ENUM_TRIGGER_MODE,
                trigger_mode_ ? GX_TRIGGER_MODE_ON : GX_TRIGGER_MODE_OFF),
            "set trigger mode");
        if (trigger_mode_) {
            // External hardware trigger on Line0 (rising edge) — matches the
            // RoboMaster trigger box wiring.
            SDK_RET_ASSERT(
                GXSetEnum(handle_, GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_LINE0),
                "set trigger source");
            SDK_RET_ASSERT(
                GXSetEnum(handle_, GX_ENUM_TRIGGER_ACTIVATION, GX_TRIGGER_ACTIVATION_RISINGEDGE),
                "set trigger activation");
        }

        ret = GXSetBool(handle_, GX_BOOL_REVERSE_X, profile.invert_image);
        if (ret != GX_STATUS_SUCCESS) {
            SPDLOG_WARN(
                "set reverse x to {}: {}", profile.invert_image, error_code_to_message(ret));
        }
        ret = GXSetBool(handle_, GX_BOOL_REVERSE_Y, profile.invert_image);
        if (ret != GX_STATUS_SUCCESS) {
            SPDLOG_WARN(
                "set reverse y to {}: {}", profile.invert_image, error_code_to_message(ret));
        }

        SDK_RET_ASSERT(
            GXSetEnum(handle_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF),
            "set auto exposure off");
        if (auto set_exposure = set_float_feature(
                handle_, GX_FLOAT_EXPOSURE_TIME, profile.exposure_time.count(), "exposure time");
            !set_exposure) {
            return std::unexpected(set_exposure.error());
        }

        SDK_RET_ASSERT(
            GXSetEnum(handle_, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_OFF), "set auto gain off");
        if (auto set_gain = set_float_feature(handle_, GX_FLOAT_GAIN, profile.gain, "gain");
            !set_gain) {
            return std::unexpected(set_gain.error());
        }

        // Color cameras only; monochrome models do not expose this feature.
        ret = GXSetEnum(handle_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
        if (ret != GX_STATUS_SUCCESS) {
            SPDLOG_WARN("set auto white balance: {}", error_code_to_message(ret));
        }

        int64_t width  = 0;
        int64_t height = 0;
        SDK_RET_ASSERT(GXGetInt(handle_, GX_INT_WIDTH, &width), "get width");
        SDK_RET_ASSERT(GXGetInt(handle_, GX_INT_HEIGHT, &height), "get height");
        image_width_  = static_cast<int>(width);
        image_height_ = static_cast<int>(height);
        SPDLOG_INFO("daheng camera image size: {}x{}", image_width_, image_height_);

        SDK_RET_ASSERT(GXStreamOn(handle_), "stream on");
        grabbing_ = true;
        valid_.store(true, std::memory_order_release);
        return {};
    }

    void uninit_camera() noexcept {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        valid_.store(false, std::memory_order_release);
        if (handle_ == nullptr) {
            return;
        }
        if (grabbing_) {
            const auto ret = GXStreamOff(handle_);
            if (ret != GX_STATUS_SUCCESS) {
                SPDLOG_WARN("stream off: {}", error_code_to_message(ret));
            }
            grabbing_ = false;
        }
        (void)GXCloseDevice(handle_);
        handle_ = nullptr;
    }
};

// Factory method implementation
std::expected<std::unique_ptr<ImageCapturer>, CameraError>
    ImageCapturer::create(const CameraProfile& profile, const char* user_defined_name) noexcept {
    auto capturer    = std::make_unique<ImageCapturer>();
    auto init_result = capturer->init(profile, user_defined_name);
    if (!init_result) {
        return std::unexpected(init_result.error());
    }
    return capturer;
}

ImageCapturer::ImageCapturer() noexcept
    : impl_(std::make_unique<ImageCapturer::Impl>()) {}

ImageCapturer::~ImageCapturer() noexcept = default;

std::expected<void, CameraError>
    ImageCapturer::init(const CameraProfile& profile, const char* user_defined_name) noexcept {
    return impl_->init(profile, user_defined_name);
}

std::expected<void, CameraError> ImageCapturer::software_trigger_on() noexcept {
    return impl_->software_trigger_on();
}

std::expected<void, CameraError>
    ImageCapturer::set_frame_rate_inner_trigger_mode(float frame_rate) noexcept {
    return impl_->set_frame_rate_inner_trigger_mode(frame_rate);
}

std::expected<void, CameraError> ImageCapturer::stop_grabbing() noexcept {
    return impl_->stop_grabbing();
}

std::expected<cv::Mat, CameraError>
    ImageCapturer::read(std::chrono::duration<unsigned int, std::micro> timeout) noexcept {
    return impl_->read(timeout);
}

std::expected<std::tuple<int, int>, CameraError> ImageCapturer::get_width_height() const noexcept {
    return impl_->get_image_size();
}

bool ImageCapturer::valid() const noexcept { return impl_->valid(); }

} // namespace dahengcamera
