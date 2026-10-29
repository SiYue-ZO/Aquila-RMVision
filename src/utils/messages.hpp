#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <opencv2/core.hpp>
#include <vector>

namespace rmvision {

struct ImageFrame {
    std::uint64_t sequence{};
    cv::Mat image;
    std::chrono::steady_clock::time_point capture_time{};
};

enum class ArmorColor { unknown, red, blue };
enum class ArmorType { unknown, small, big };
enum class DetectionSource { unknown, traditional, neural };
enum class DetectionScoreKind { unknown, heuristic, model };

struct Detection {
    // 传统检测不识别数字，0 表示编号未知，不是跟踪 ID。
    int target_id{};
    cv::Rect bounding_box;
    // 来源相关的排序分数，不是统一的识别概率；消费者需检查 score_kind。
    float confidence{};
    ArmorColor color{ArmorColor::unknown};
    ArmorType armor_type{ArmorType::unknown};
    // 原图像素坐标，顺序为左上、右上、右下、左下灯条端点。
    std::array<cv::Point2f, 4> corners{};
    DetectionSource source{DetectionSource::unknown};
    DetectionScoreKind score_kind{DetectionScoreKind::unknown};
    float geometry_score{};
    // [0,1]：灯条方向的可观测程度，不是像素误差或概率。模糊亮斑通常较低。
    float corner_quality{};
    // 仅表示四点有限、凸且位于图像内；不保证适合精确 PnP。
    bool corners_valid{};
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
    // target_id 是数字类别；track_id 才是追踪器分配的身份，0 表示尚未关联。
    std::uint64_t track_id{};
    std::chrono::steady_clock::time_point capture_time{};
};

} // namespace rmvision
