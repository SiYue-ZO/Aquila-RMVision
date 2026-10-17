#include "components/detector/detector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <stdexcept>
#include <vector>

namespace rmvision::components {
namespace {

constexpr float kRadiansToDegrees = 180.0F / static_cast<float>(CV_PI);

struct LightBar {
    cv::Point2f center;
    cv::Point2f top;
    cv::Point2f bottom;
    float length;
    float tilt_deg;
    ArmorColor color;
};

struct ArmorCandidate {
    std::size_t left;
    std::size_t right;
    Detection detection;
};

/**
 * @brief 根据亮度、颜色差分和几何约束提取候选灯条，不修改原图。
 * @param bgr 输入的 8 位 BGR 图像。
 * @param params 颜色选择、分割阈值及灯条几何参数。
 * @return 按中心横坐标从左到右排列的灯条，包含原图端点、长度和倾角。
 */
std::vector<LightBar> findLights(const cv::Mat& bgr, const TraditionalDetectorParams& params) {
    // 分离 B、G、R 通道用于颜色判断，灰度阈值则限定灯条的高亮区域。
    std::vector<cv::Mat> channels;
    cv::split(bgr, channels);
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    const cv::Mat bright_mask = gray > params.brightness_threshold;
    const auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3});

    std::vector<LightBar> lights;
    for (const auto color : {ArmorColor::red, ArmorColor::blue}) {
        // 按配置处理单色或红蓝两色，避免异色灯条进入同一配对。
        if (params.target_color != ArmorColor::unknown && params.target_color != color) {
            continue;
        }

        // 红色取 R-B，蓝色取 B-R；同时满足亮度与颜色差，排除白色高亮。
        cv::Mat difference;
        const int dominant = color == ArmorColor::red ? 2 : 0;
        const int opposite = color == ArmorColor::red ? 0 : 2;
        cv::subtract(channels[dominant], channels[opposite], difference);
        cv::Mat mask = (difference > params.color_difference_threshold) & bright_mask;
        // 只做小尺度闭运算，连接灯条亮斑，避免膨胀过度合并相邻灯条。
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        // 提取外轮廓，并先用面积排除细小噪点。
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (const auto& contour : contours) {
            if (cv::contourArea(contour) < params.min_light_area) {
                continue;
            }
            // 用最小外接矩形拟合轮廓，将较长的边统一作为灯条主轴。
            const auto rect = cv::minAreaRect(contour);
            cv::Point2f vertices[4];
            rect.points(vertices);
            cv::Point2f axis = vertices[1] - vertices[0];
            const auto other_axis = vertices[2] - vertices[1];
            float length = static_cast<float>(cv::norm(axis));
            float width = static_cast<float>(cv::norm(other_axis));
            if (width > length) {
                std::swap(length, width);
                axis = other_axis;
            }
            // 排除退化矩形、过短灯条和长宽比不合理的亮斑。
            if (width <= 0.0F || length < params.min_light_length) {
                continue;
            }
            const float aspect_ratio = length / width;
            if (aspect_ratio < params.min_light_aspect_ratio ||
                aspect_ratio > params.max_light_aspect_ratio) {
                continue;
            }

            // 从长轴计算端点，避免依赖 OpenCV 旋转矩形的角度表示范围。
            axis *= 1.0F / length;
            if (axis.y < 0.0F) {
                axis = -axis;
            }
            // 主轴朝图像下方时，atan2(x, y) 给出相对竖直方向的倾角，单位为度。
            const float tilt = std::atan2(axis.x, axis.y) * kRadiansToDegrees;
            if (std::abs(tilt) > params.max_light_tilt_deg) {
                continue;
            }
            // 从中心沿主轴各移动半条长度，得到上、下端点。
            lights.push_back({rect.center, rect.center - axis * (length / 2.0F),
                              rect.center + axis * (length / 2.0F), length, tilt, color});
        }
    }
    // 按横坐标排序，后续下标较小的灯条作为左灯条。
    std::sort(lights.begin(), lights.end(),
              [](const LightBar& a, const LightBar& b) { return a.center.x < b.center.x; });
    return lights;
}

/**
 * @brief 检查配对灯条之间是否夹有同色、长度相近且靠近中心连线的灯条。
 * @param lights 已按横坐标排序的灯条列表；left、right 指向横坐标严格递增的两根灯条。
 * @param left 左侧灯条下标。
 * @param right 右侧灯条下标。
 * @param mean_length 配对灯条的平均长度，用于归一化长度和高度差。
 * @param max_length_ratio 中间灯条与平均长度之间允许的最大长度比。
 * @return 存在中间灯条时返回 true，调用方据此排除跨灯条配对。
 */
bool hasIntermediateLight(const std::vector<LightBar>& lights, std::size_t left,
                          std::size_t right, float mean_length, float max_length_ratio) {
    const auto& a = lights[left];
    const auto& b = lights[right];
    // 只检查左右灯条之间的候选，忽略异色或尺度明显不同的灯条。
    for (std::size_t i = left + 1; i < right; ++i) {
        const auto& middle = lights[i];
        if (middle.color != a.color || middle.length < mean_length / max_length_ratio ||
            middle.length > mean_length * max_length_ratio) {
            continue;
        }
        // 在两端中心连线上插值得到期望高度，避免把上下错层目标误判为中间灯条。
        const float t = (middle.center.x - a.center.x) / (b.center.x - a.center.x);
        const float expected_y = a.center.y + t * (b.center.y - a.center.y);
        if (std::abs(middle.center.y - expected_y) < mean_length * 0.5F) {
            return true;
        }
    }
    return false;
}

}  // namespace

/**
 * @brief 保存检测参数，并在处理图像前检查参数的有限性、范围及相互约束。
 * @param params 传统装甲板检测参数，省略时使用头文件中的默认值。
 * @throws std::invalid_argument 参数无效，例如阈值越界或最小间距不小于最大间距。
 */
TraditionalArmorDetector::TraditionalArmorDetector(TraditionalDetectorParams params)
    : params_(params) {
    // 先检查浮点参数是否有限，防止 NaN 或无穷值使后续阈值比较失效。
    const std::initializer_list<double> values{
        params.min_light_area, params.min_light_length, params.min_light_aspect_ratio,
        params.max_light_aspect_ratio, params.max_light_tilt_deg, params.max_length_ratio,
        params.max_angle_difference_deg, params.max_center_line_angle_deg,
        params.min_pair_distance_ratio, params.max_pair_distance_ratio,
        params.max_vertical_offset_ratio, params.big_armor_distance_ratio};
    const bool finite = std::all_of(values.begin(), values.end(),
                                   [](double value) { return std::isfinite(value); });
    // 检查像素阈值、比例、角度及取值关系，同时避免评分计算出现除零。
    if (!finite || params.brightness_threshold < 0 || params.brightness_threshold > 255 ||
        params.color_difference_threshold < 1 || params.color_difference_threshold > 255 ||
        params.min_light_area <= 0.0 || params.min_light_length <= 0.0F ||
        params.min_light_aspect_ratio <= 1.0F ||
        params.max_light_aspect_ratio < params.min_light_aspect_ratio ||
        params.max_light_tilt_deg <= 0.0F || params.max_light_tilt_deg >= 90.0F ||
        params.max_length_ratio <= 1.0F || params.max_angle_difference_deg <= 0.0F ||
        params.max_angle_difference_deg >= 90.0F || params.max_center_line_angle_deg <= 0.0F ||
        params.max_center_line_angle_deg >= 90.0F || params.min_pair_distance_ratio <= 0.0F ||
        params.max_pair_distance_ratio <= params.min_pair_distance_ratio ||
        params.max_vertical_offset_ratio <= 0.0F ||
        params.big_armor_distance_ratio <= params.min_pair_distance_ratio ||
        params.big_armor_distance_ratio >= params.max_pair_distance_ratio ||
        (params.target_color != ArmorColor::unknown && params.target_color != ArmorColor::red &&
         params.target_color != ArmorColor::blue)) {
        throw std::invalid_argument("Invalid traditional armor detector parameters");
    }
}

/**
 * @brief 检测单帧装甲板：提取灯条、同色几何配对、评分去重，按横坐标输出结果。
 * @param frame 带帧序号和采集时间的 8 位 BGR/BGRA 图像，不会被修改。
 * @return 保留原帧信息的检测结果；四点使用原图坐标，空图返回空检测列表。
 * @throws std::invalid_argument 非空图像的像素格式不受支持。
 */
DetectionFrame TraditionalArmorDetector::detect(const ImageFrame& frame) {
    // 先保留输入帧信息，即使没有检出目标也能追溯结果对应的图像与时刻。
    DetectionFrame result;
    result.source_sequence = frame.sequence;
    result.capture_time = frame.capture_time;
    if (frame.image.empty()) {
        return result;
    }
    // 仅接受 8 位彩色图像，并将带透明通道的 BGRA 输入统一转换成 BGR。
    if (frame.image.type() != CV_8UC3 && frame.image.type() != CV_8UC4) {
        throw std::invalid_argument("Detector expects an 8-bit BGR or BGRA image");
    }
    cv::Mat bgr = frame.image;
    if (frame.image.channels() == 4) {
        cv::cvtColor(frame.image, bgr, cv::COLOR_BGRA2BGR);
    }
    // 提取按横坐标排序的灯条，再枚举左右组合生成装甲板候选。
    const auto lights = findLights(bgr, params_);
    std::vector<ArmorCandidate> candidates;
    for (std::size_t i = 0; i < lights.size(); ++i) {
        for (std::size_t j = i + 1; j < lights.size(); ++j) {
            const auto& left = lights[i];
            const auto& right = lights[j];
            // 配对必须同色且左右中心可区分，保证后续插值和角度计算有效。
            if (left.color != right.color) {
                continue;
            }
            const float dx = right.center.x - left.center.x;
            const float dy = std::abs(right.center.y - left.center.y);
            if (dx <= 0.0F) {
                continue;
            }
            // 用长度比、倾角差、中心连线角度和归一化间距描述两根灯条的几何关系。
            const float mean_length = (left.length + right.length) / 2.0F;
            const float length_ratio = std::max(left.length, right.length) /
                                       std::min(left.length, right.length);
            const float angle_difference = std::abs(left.tilt_deg - right.tilt_deg);
            const float center_angle = std::atan2(dy, dx) * kRadiansToDegrees;
            const float distance_ratio = std::hypot(dx, dy) / mean_length;
            // 排除长度、方向、位置不匹配，以及跨过同色中间灯条的组合。
            if (length_ratio > params_.max_length_ratio ||
                angle_difference > params_.max_angle_difference_deg ||
                center_angle > params_.max_center_line_angle_deg ||
                dy / mean_length > params_.max_vertical_offset_ratio ||
                distance_ratio < params_.min_pair_distance_ratio ||
                distance_ratio > params_.max_pair_distance_ratio ||
                hasIntermediateLight(lights, i, j, mean_length, params_.max_length_ratio)) {
                continue;
            }

            // 按间距比例粗分大小装甲，并按左上、右上、右下、左下保存四点。
            Detection detection;
            detection.color = left.color;
            detection.armor_type = distance_ratio >= params_.big_armor_distance_ratio
                                       ? ArmorType::big
                                       : ArmorType::small;
            detection.corners = {left.top, right.top, right.bottom, left.bottom};
            const std::vector<cv::Point2f> points(detection.corners.begin(),
                                                detection.corners.end());
            // 包围框限制在原图范围内，避免后续裁剪图像时越界。
            detection.bounding_box = cv::boundingRect(points) & cv::Rect(0, 0, bgr.cols, bgr.rows);
            // 将长度差、方向差和中心连线倾斜程度归一化；误差越小，几何评分越高。
            const float length_error = (length_ratio - 1.0F) / (params_.max_length_ratio - 1.0F);
            detection.confidence = std::clamp(
                1.0F - (length_error + angle_difference / params_.max_angle_difference_deg +
                        center_angle / params_.max_center_line_angle_deg) / 3.0F,
                0.0F, 1.0F);
            candidates.push_back({i, j, detection});
        }
    }

    // 优先保留几何一致性高的配对，每根灯条最多属于一块装甲板。
    std::stable_sort(candidates.begin(), candidates.end(),
                     [](const ArmorCandidate& a, const ArmorCandidate& b) {
                         return a.detection.confidence > b.detection.confidence;
                     });
    // 按评分贪心选择候选，已被选中装甲板使用的灯条不再参与其他配对。
    std::vector<bool> used(lights.size(), false);
    for (const auto& candidate : candidates) {
        if (used[candidate.left] || used[candidate.right]) {
            continue;
        }
        result.detections.push_back(candidate.detection);
        used[candidate.left] = used[candidate.right] = true;
    }
    // 最终按原图中的左右位置排列输出，便于显示和离线检查。
    std::sort(result.detections.begin(), result.detections.end(),
              [](const Detection& a, const Detection& b) {
                  return a.bounding_box.x < b.bounding_box.x;
              });
    return result;
}

}  // namespace rmvision::components
