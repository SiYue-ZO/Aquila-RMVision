#include "components/detector/detector.hpp"

namespace rmvision::components {

DetectionFrame DemoDetector::detect(const ImageFrame& frame) {
    DetectionFrame result;
    result.source_sequence = frame.sequence;

    // 固定的模拟目标，用于验证 camera -> detector -> tracker 的数据链路。
    result.detections.push_back(Detection{
        .target_id = 1,
        .bounding_box = cv::Rect(280, 200, 80, 60),
        .confidence = 0.95F,
    });
    return result;
}

}  // namespace rmvision::components
