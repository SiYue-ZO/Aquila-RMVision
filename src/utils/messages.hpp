#pragma once

#include <opencv2/core.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <vector>
#include <array>

namespace rmvision {

struct ImageFrame {
    std::uint64_t sequence{};
    cv::Mat image;
    std::chrono::steady_clock::time_point capture_time{};
};

enum class ArmorColor { unknown, red, blue };
enum class ArmorType { unknown, small, big };

struct Detection {
    // 传统检测不识别数字，0 表示编号未知，不是跟踪 ID。
    int target_id{};
    cv::Rect bounding_box;
    // 几何匹配评分，不代表经过标定的识别概率。
    float confidence{};
    ArmorColor color{ArmorColor::unknown};
    ArmorType armor_type{ArmorType::unknown};
    // 原图像素坐标，顺序为左上、右上、右下、左下灯条端点。
    std::array<cv::Point2f, 4> corners{};
};

struct DetectionFrame {
    std::uint64_t source_sequence{};
    std::chrono::steady_clock::time_point capture_time{};
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
