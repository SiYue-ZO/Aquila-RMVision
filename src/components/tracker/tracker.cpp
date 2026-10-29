#include "components/tracker/tracker.hpp"

namespace rmvision::components {

TrackState SimpleTracker::update(const DetectionFrame& detections) {
    if (detections.detections.empty()) {
        return TrackState{
            .valid = false,
            .target_id = 0,
            .source_sequence = detections.source_sequence,
            .center = cv::Point2f(),
            .confidence = 0.0F,
            .capture_time = detections.capture_time,
        };
    }

    const auto& detection = detections.detections.front();
    return TrackState{
        .valid = true,
        .target_id = detection.target_id,
        .source_sequence = detections.source_sequence,
        .center = cv::Point2f(detection.bounding_box.x + detection.bounding_box.width / 2.0F,
                              detection.bounding_box.y + detection.bounding_box.height / 2.0F),
        .confidence = detection.confidence,
        .capture_time = detections.capture_time,
    };
}

} // namespace rmvision::components
