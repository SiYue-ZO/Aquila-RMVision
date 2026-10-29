#pragma once

#include "utils/messages.hpp"

#include <cstddef>

namespace rmvision::components {

class Detector {
  public:
    virtual ~Detector() = default;
    virtual DetectionFrame detect(const ImageFrame& frame) = 0;
};

struct TraditionalDetectorParams {
    // 亮度改用目标颜色通道，避免鲜红/鲜蓝灯条被灰度权重压暗。
    int brightness_threshold{200};
    int color_difference_threshold{70};
    ArmorColor target_color{ArmorColor::unknown}; // unknown 表示同时检测红蓝。

    double min_light_area{20.0};
    float min_light_length{8.0F};
    // 接近竖直的矩形边 / 横向边；允许横向运动拖影造成宽亮斑。
    float min_light_aspect_ratio{0.65F};
    float max_light_aspect_ratio{20.0F};
    float max_light_tilt_deg{45.0F};
    float reliable_light_aspect_ratio{2.5F};

    float max_length_ratio{1.8F};
    float max_angle_difference_deg{20.0F};
    float max_center_line_angle_deg{25.0F};
    float max_rectangular_error_deg{25.0F};
    // 配对距离和高度差以平均灯条长度归一化，适应不同目标尺度。
    float min_pair_distance_ratio{1.0F};
    float max_pair_distance_ratio{4.5F};
    float max_vertical_offset_ratio{1.2F};
    float big_armor_distance_ratio{3.2F}; // 仅按几何比例粗分大小装甲。
    float min_pair_score{0.45F};
    float pattern_score_weight{0.4F};
    int pattern_brightness_threshold{140};
    // 包含宽亮斑时需要板内图案支持，避免把圆形反光或板间空隙配成装甲板。
    float min_blob_pattern_score{0.08F};
};

// 仅用于传统检测的诊断，不混入供 YOLO/追踪器共享的观测契约。
struct TraditionalDetectorDiagnostics {
    std::size_t contours{};
    std::size_t rejected_area{};
    std::size_t rejected_light_geometry{};
    std::size_t lights{};
    std::size_t uncertain_lights{};
    std::size_t pairs{};
    std::size_t rejected_pair_geometry{};
    std::size_t rejected_intermediate{};
    std::size_t rejected_corners{};
    std::size_t rejected_score{};
    std::size_t candidates{};
    std::size_t suppressed{};
};

class TraditionalArmorDetector final : public Detector {
  public:
    explicit TraditionalArmorDetector(TraditionalDetectorParams params = {});

    // 接收 BGR/BGRA 的 8 位图像；不修改输入，也不负责显示、数字识别和 PnP。
    DetectionFrame detect(const ImageFrame& frame) override;
    DetectionFrame detect(const ImageFrame& frame, TraditionalDetectorDiagnostics& diagnostics);

    // 为未来网络观测提供可选修正；仅采用附近清晰灯条，失败时完全保留输入观测。
    // source、score_kind、confidence、target_id 始终由调用方的检测后端决定。
    bool refineCorners(const ImageFrame& frame, Detection& detection);

  private:
    TraditionalDetectorParams params_;
};

} // namespace rmvision::components
