#pragma once

#include "utils/messages.hpp"

namespace rmvision::components {

class Detector {
public:
    virtual ~Detector() = default;
    virtual DetectionFrame detect(const ImageFrame& frame) = 0;
};

struct TraditionalDetectorParams {
    // 亮度与红蓝通道差同时满足阈值，抑制白色数字和背景反光。
    int brightness_threshold{170};
    int color_difference_threshold{70};
    ArmorColor target_color{ArmorColor::unknown};  // unknown 表示同时检测红蓝。

    double min_light_area{20.0};
    float min_light_length{12.0F};
    float min_light_aspect_ratio{2.0F};
    float max_light_aspect_ratio{20.0F};
    float max_light_tilt_deg{30.0F};

    float max_length_ratio{1.5F};
    float max_angle_difference_deg{12.0F};
    float max_center_line_angle_deg{20.0F};
    // 配对距离和高度差以平均灯条长度归一化，适应不同目标尺度。
    float min_pair_distance_ratio{1.0F};
    float max_pair_distance_ratio{4.2F};
    float max_vertical_offset_ratio{0.5F};
    float big_armor_distance_ratio{3.2F};  // 仅按几何比例粗分大小装甲。
};

class TraditionalArmorDetector final : public Detector {
public:
    explicit TraditionalArmorDetector(TraditionalDetectorParams params = {});

    // 接收 BGR/BGRA 的 8 位图像；不修改输入，也不负责显示、数字识别和 PnP。
    DetectionFrame detect(const ImageFrame& frame) override;

private:
    TraditionalDetectorParams params_;
};

}  // namespace rmvision::components
