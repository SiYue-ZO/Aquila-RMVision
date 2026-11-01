#pragma once

#include "components/camera/frame_source.hpp"
#include "components/detector/detector.hpp"

#include <opencv2/videoio.hpp>

#include <string>
#include <optional>
#include <vector>

namespace rmvision::components {

struct IpCamOptions {
    // 手机相机填写 IP:端口，自动使用 http://IP:端口/video；也支持完整 URL 和本地视频。
    std::string url;
    int timeout_ms{5000};
};

struct IpCamPoseResult {
    bool valid{false};
    cv::Mat rvec;
    cv::Mat tvec;
    double distance_m{};
    std::string error;
};

// 临时网络相机适配器，同时提供 PnP 测试；检测与窗口管理仍由运行时处理。
class IpCamFrameSource final : public FrameSource {
public:
    explicit IpCamFrameSource(IpCamOptions options = {});

    void open();
    // 断流、超时或本地回放结束返回 nullopt；未打开时调用则抛出异常。
    std::optional<ImageFrame> next() override;

    // 使用本文件对应 cpp 中的相机参数逐个解算，空检测返回空结果。
    std::vector<IpCamPoseResult> solvePoses(const DetectionFrame& detections);
    void drawPnPAxes(cv::Mat& image, const IpCamPoseResult& pose) const;

private:
    IpCamOptions options_;
    cv::VideoCapture capture_;
    std::uint64_t next_sequence_{};
    cv::Size image_size_;
    cv::Mat camera_matrix_;
    cv::Mat distortion_;
    std::optional<Armor_PnPsolver> pnp_solver_;
};

}  // namespace rmvision::components
