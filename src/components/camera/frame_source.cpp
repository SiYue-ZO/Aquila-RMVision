#include "components/camera/frame_source.hpp"

#include <opencv2/core.hpp>

namespace rmvision::components {

std::optional<ImageFrame> DemoFrameSource::next() {
    if (next_frame_ >= frame_count_) {
        return std::nullopt;
    }

    ImageFrame frame;
    frame.sequence = static_cast<std::uint64_t>(next_frame_);
    frame.capture_time = std::chrono::steady_clock::now();
    frame.image = cv::Mat(480, 640,  CV_8UC3, cv::Scalar(20, 20, 20));

    ++next_frame_;
    return frame;
}

}  // namespace rmvision::components
