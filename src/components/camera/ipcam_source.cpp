#include "components/camera/ipcam_source.hpp"

#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <array>
#include <exception>
#include <stdexcept>
#include <utility>
#include <vector>

namespace rmvision::components {
namespace {

// PnP 测试参数集中在这里修改，无需新增标定文件。
// 以下仅为合成测试示例，真实测距请替换为手机相机的标定值及对应分辨率。
constexpr int kCalibrationWidth = 640;
constexpr int kCalibrationHeight = 480;
constexpr std::array<double, 9> kCameraMatrix{
    800.0, 0.0, 320.0,
    0.0, 800.0, 240.0,
    0.0, 0.0, 1.0};
// 顺序为 k1、k2、p1、p2、k3；当前求解器固定读取 5 个系数。
const std::vector<double> kDistCoeffs{0.0, 0.0, 0.0, 0.0, 0.0};

}  // namespace

IpCamFrameSource::IpCamFrameSource(IpCamOptions options) : options_(std::move(options)) {
    if (options_.url.empty() || options_.timeout_ms <= 0) {
        throw std::invalid_argument("IPCam needs a non-empty IP:port and a positive timeout");
    }
    // 手机相机只需填写 IP:端口，自动补全 MJPEG 地址；完整地址和本地回放路径仍可使用。
    if (options_.url.find("://") == std::string::npos &&
        options_.url.find(':') != std::string::npos &&
        options_.url.find_first_of("/\\") == std::string::npos) {
        options_.url = "http://" + options_.url + "/video";
    }
}

void IpCamFrameSource::open() {
    capture_.release();
    image_size_ = {};
    pnp_solver_.reset();
    // 超时必须在 open 时传入；事后 set 对 FFmpeg 的这两个参数无效。
    // 固定 FFmpeg 后端，避免自动回退到不支持超时的后端后永久阻塞。
    const std::vector<int> parameters{
        cv::CAP_PROP_OPEN_TIMEOUT_MSEC, options_.timeout_ms,
        cv::CAP_PROP_READ_TIMEOUT_MSEC, options_.timeout_ms};
    if (!capture_.open(options_.url, cv::CAP_FFMPEG, parameters)) {
        throw std::runtime_error("Cannot open IPCam: " + options_.url +
                                 " (check stream URL, network and OpenCV FFmpeg support)");
    }
    next_sequence_ = 0;
}

std::optional<ImageFrame> IpCamFrameSource::next() {
    if (!capture_.isOpened()) {
        throw std::logic_error("Open IPCam before requesting frames");
    }
    ImageFrame frame;
    if (!capture_.read(frame.image) || frame.image.empty()) {
        return std::nullopt;
    }
    // 这是本机收到解码帧的时间，不是手机传感器曝光时间，包含网络和解码延迟。
    frame.capture_time = std::chrono::steady_clock::now();
    frame.sequence = next_sequence_++;
    if (frame.image.type() != CV_8UC3) {
        throw std::runtime_error("IPCam must decode to an 8-bit BGR image");
    }
    if (image_size_ != frame.image.size()) {
        image_size_ = frame.image.size();
        pnp_solver_.reset();
    }
    return frame;
}

std::vector<IpCamPoseResult> IpCamFrameSource::solvePoses(const DetectionFrame& detections) {
    std::vector<IpCamPoseResult> poses;
    if (detections.detections.empty()) {
        return poses;
    }
    if (image_size_.empty()) {
        throw std::logic_error("Read an IPCam frame before testing PnP");
    }
    if (!pnp_solver_) {
        if (kCalibrationWidth <= 0 || kCalibrationHeight <= 0 ||
            kDistCoeffs.size() != 5 || kCameraMatrix[0] <= 0 || kCameraMatrix[4] <= 0) {
            throw std::invalid_argument("Invalid PnP test parameters in ipcam_source.cpp");
        }
        auto intrinsics = kCameraMatrix;
        // 按参考分辨率缩放内参；仅适用于同一视场的图像缩放，不适用于裁剪或变焦。
        const double scale_x = static_cast<double>(image_size_.width) / kCalibrationWidth;
        const double scale_y = static_cast<double>(image_size_.height) / kCalibrationHeight;
        for (int i = 0; i < 3; ++i) {
            intrinsics[i] *= scale_x;
            intrinsics[i + 3] *= scale_y;
        }
        camera_matrix_ = cv::Mat(3, 3, CV_64F, intrinsics.data()).clone();
        distortion_ = cv::Mat(kDistCoeffs).reshape(1, 1).clone();
        if (!cv::checkRange(camera_matrix_) || !cv::checkRange(distortion_)) {
            throw std::invalid_argument("Non-finite PnP camera parameters in ipcam_source.cpp");
        }
        pnp_solver_.emplace(intrinsics, kDistCoeffs);
    }
    poses.reserve(detections.detections.size());
    DetectionFrame single;
    single.detections.reserve(1);
    for (const auto& armor : detections.detections) {
        // 用户的接口只解算 back()，每次放入一个目标，保证多目标一一对应。
        single.detections.assign(1, armor);
        IpCamPoseResult pose;
        try {
            pose.valid = pnp_solver_->PnPsolver(single, pose.rvec, pose.tvec);
            if (!pose.valid) {
                pose.error = "solvePnP returned false";
            } else if (!cv::checkRange(pose.rvec) || !cv::checkRange(pose.tvec) ||
                       pose.tvec.at<double>(2) <= 0) {
                pose.valid = false;
                pose.error = "Non-finite pose or target behind camera";
            } else {
                pose.distance_m = cv::norm(pose.tvec);
            }
        } catch (const std::exception& error) {
            pose.valid = false;
            pose.error = error.what();
        }
        poses.push_back(std::move(pose));
    }
    return poses;
}

void IpCamFrameSource::drawPnPAxes(cv::Mat& image, const IpCamPoseResult& pose) const {
    if (pose.valid && pnp_solver_) {
        cv::drawFrameAxes(image, camera_matrix_, distortion_, pose.rvec, pose.tvec, 0.05F, 2);
    }
}

}  // namespace rmvision::components
