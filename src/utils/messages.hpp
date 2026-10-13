#pragma once

#include <opencv2/core.hpp>

#include <chrono>
#include <cstdint>
#include <vector>

namespace rmvision {

struct ImageFrame {
    std::uint64_t sequence{};
    cv::Mat image;
    std::chrono::steady_clock::time_point capture_time{};
};

struct Detection {
    int target_id{};
    cv::Rect bounding_box;
    float confidence{};
};

struct DetectionFrame {
    std::uint64_t source_sequence{};
    std::vector<Detection> detections;
};

struct TrackState {
    bool valid{};
    int target_id{};
    std::uint64_t source_sequence{};
    cv::Point2f center;
    float confidence{};
};

}  // namespace rmvision
