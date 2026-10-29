#include "components/detector/detector.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <stdexcept>
#include <vector>

namespace rmvision::components {
namespace {

constexpr float kRadiansToDegrees = 180.0F / static_cast<float>(CV_PI);

struct LightBar {
    cv::Point2f center;
    cv::Point2f axis;
    float length;
    float tilt_deg;
    float quality;
    ArmorColor color;
    std::vector<cv::Point> contour;
};

struct ArmorCandidate {
    std::size_t left;
    std::size_t right;
    Detection detection;
};

// 颜色通道亮度和颜色差分共同分割，保留灰度较低的鲜红/鲜蓝灯条。
std::vector<LightBar> findLights(const cv::Mat& bgr, const TraditionalDetectorParams& params,
                                 TraditionalDetectorDiagnostics& diagnostics) {
    std::vector<cv::Mat> channels;
    cv::split(bgr, channels);
    const auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3});
    std::vector<LightBar> lights;
    for (const auto color : {ArmorColor::red, ArmorColor::blue}) {
        const int dominant = color == ArmorColor::red ? 2 : 0;
        const int opposite = color == ArmorColor::red ? 0 : 2;
        cv::Mat difference;
        cv::subtract(channels[dominant], channels[opposite], difference);
        cv::Mat mask = (difference > params.color_difference_threshold) &
                       (channels[dominant] > params.brightness_threshold);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        diagnostics.contours += contours.size();
        for (auto& contour : contours) {
            if (cv::contourArea(contour) < params.min_light_area) {
                ++diagnostics.rejected_area;
                continue;
            }
            const auto rect = cv::minAreaRect(contour);
            cv::Point2f vertices[4];
            rect.points(vertices);
            cv::Point2f axis = vertices[1] - vertices[0];
            auto other_axis = vertices[2] - vertices[1];
            // 选更接近竖直的边，而非无条件选长边，避免横向拖影令主轴翻转 90°。
            if (std::abs(other_axis.y) > std::abs(axis.y)) {
                std::swap(axis, other_axis);
            }
            float length = static_cast<float>(cv::norm(axis));
            const float width = static_cast<float>(cv::norm(other_axis));
            if (width <= 0.0F || length < params.min_light_length) {
                ++diagnostics.rejected_light_geometry;
                continue;
            }
            const float aspect = length / width;
            axis *= 1.0F / length;
            if (axis.y < 0.0F)
                axis = -axis;
            float tilt = std::atan2(axis.x, axis.y) * kRadiansToDegrees;
            if (aspect < params.min_light_aspect_ratio || aspect > params.max_light_aspect_ratio ||
                std::abs(tilt) > params.max_light_tilt_deg) {
                ++diagnostics.rejected_light_geometry;
                continue;
            }
            const float quality =
                std::clamp((aspect - params.min_light_aspect_ratio) /
                               (params.reliable_light_aspect_ratio - params.min_light_aspect_ratio),
                           0.0F, 1.0F);
            if (quality < 1.0F) {
                // 宽亮斑的矩形方向不可靠。暂用竖直跨度，配对时再借助中心连线估计。
                axis = {0.0F, 1.0F};
                length = static_cast<float>(cv::boundingRect(contour).height - 1);
                tilt = 0.0F;
                ++diagnostics.uncertain_lights;
            }
            lights.push_back({rect.center, axis, length, tilt, quality, color, std::move(contour)});
        }
    }
    std::sort(lights.begin(), lights.end(),
              [](const LightBar& a, const LightBar& b) { return a.center.x < b.center.x; });
    diagnostics.lights = lights.size();
    return lights;
}

// 不可靠亮斑沿配对法向投影恢复端点；始终保留低 quality，不能冒充精确角点。
void alignUncertainLight(LightBar& light, const LightBar& original, const cv::Point2f& normal) {
    if (light.quality >= 1.0F)
        return;
    float low = 0.0F, high = 0.0F;
    for (const auto& point : original.contour) {
        const float projection = (cv::Point2f(point) - light.center).dot(normal);
        low = std::min(low, projection);
        high = std::max(high, projection);
    }
    light.center += normal * ((low + high) / 2.0F);
    light.axis = normal;
    light.length = high - low;
    light.tilt_deg = std::atan2(normal.x, normal.y) * kRadiansToDegrees;
}

bool hasIntermediateLight(const std::vector<LightBar>& lights, std::size_t left, std::size_t right,
                          float mean_length, float max_length_ratio) {
    const auto& a = lights[left];
    const auto& b = lights[right];
    for (std::size_t i = left + 1; i < right; ++i) {
        const auto& middle = lights[i];
        if (middle.length < mean_length / max_length_ratio ||
            middle.length > mean_length * max_length_ratio)
            continue;
        const float t = (middle.center.x - a.center.x) / (b.center.x - a.center.x);
        const float expected_y = a.center.y + t * (b.center.y - a.center.y);
        // 灯条在纵向有重叠就应阻止跨越；异色目标也不能成为跨板配对的内部灯条。
        if (std::abs(middle.center.y - expected_y) < (mean_length + middle.length) * 0.5F)
            return true;
    }
    return false;
}

bool validCorners(const std::array<cv::Point2f, 4>& points, cv::Size size) {
    for (const auto& point : points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || point.x < 0.0F ||
            point.y < 0.0F || point.x >= size.width || point.y >= size.height)
            return false;
    }
    const std::vector<cv::Point2f> polygon(points.begin(), points.end());
    return cv::isContourConvex(polygon) && cv::contourArea(polygon) > 1.0 &&
           points[0].y < points[3].y && points[1].y < points[2].y;
}

// 板内中性高亮图案只作辅助评分，不识别数字。比单靠灯条几何更能区分相邻板间空隙。
float patternScore(const cv::Mat& bgr, const LightBar& left, const LightBar& right,
                   int brightness_threshold) {
    // 1.125 = 0.5 * 126/56，将灯条端点扩展到板面高度，仅用于图案取样。
    const cv::Point2f source[]{left.center - left.axis * (left.length * 1.125F),
                               right.center - right.axis * (right.length * 1.125F),
                               right.center + right.axis * (right.length * 1.125F),
                               left.center + left.axis * (left.length * 1.125F)};
    const cv::Point2f destination[]{{0, 0}, {63, 0}, {63, 47}, {0, 47}};
    cv::Mat patch;
    cv::warpPerspective(bgr, patch, cv::getPerspectiveTransform(source, destination), {64, 48},
                        cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    const auto inner = patch(cv::Rect(14, 4, 36, 40));
    cv::Mat foreground_mask = cv::Mat::zeros(inner.size(), CV_8UC1);
    for (int y = 0; y < inner.rows; ++y) {
        const auto* row = inner.ptr<cv::Vec3b>(y);
        for (int x = 0; x < inner.cols; ++x) {
            const auto& pixel = row[x];
            const int low = std::min({pixel[0], pixel[1], pixel[2]});
            const int high = std::max({pixel[0], pixel[1], pixel[2]});
            if (low > brightness_threshold && high - low < 80) {
                foreground_mask.at<unsigned char>(y, x) = 255;
            }
        }
    }
    cv::Mat labels, stats, centroids;
    const int count = cv::connectedComponentsWithStats(foreground_mask, labels, stats, centroids);
    int foreground = 0;
    cv::Rect strokes;
    for (int i = 1; i < count; ++i) {
        const int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area < 3)
            continue;
        strokes |=
            cv::Rect(stats.at<int>(i, cv::CC_STAT_LEFT), stats.at<int>(i, cv::CC_STAT_TOP),
                     stats.at<int>(i, cv::CC_STAT_WIDTH), stats.at<int>(i, cv::CC_STAT_HEIGHT));
        foreground += area;
    }
    // 模糊数字的笔画可能断开，检查整体竖向跨度；单根车身横向反光不能充当图案。
    if (strokes.height < inner.rows * 0.35F || strokes.width < inner.cols * 0.15F ||
        strokes.height < strokes.width * 0.75F)
        return 0.0F;
    return std::min(1.0F, foreground / (static_cast<float>(inner.total()) * 0.15F));
}

std::optional<ArmorCandidate> matchLights(const cv::Mat& bgr, const std::vector<LightBar>& lights,
                                          std::size_t i, std::size_t j,
                                          const TraditionalDetectorParams& params,
                                          TraditionalDetectorDiagnostics& diagnostics) {
    // 始终提取两色作为跨板阻挡信息，颜色配置只决定哪些同色配对可以输出。
    if (lights[i].color != lights[j].color ||
        (params.target_color != ArmorColor::unknown && lights[i].color != params.target_color)) {
        return std::nullopt;
    }
    ++diagnostics.pairs;
    // 只拷贝拟合参数，轮廓在投影时只读；避免为每个配对复制轮廓点。
    LightBar left{lights[i].center,
                  lights[i].axis,
                  lights[i].length,
                  lights[i].tilt_deg,
                  lights[i].quality,
                  lights[i].color,
                  {}};
    LightBar right{lights[j].center,
                   lights[j].axis,
                   lights[j].length,
                   lights[j].tilt_deg,
                   lights[j].quality,
                   lights[j].color,
                   {}};
    const auto delta = right.center - left.center;
    const float distance = static_cast<float>(cv::norm(delta));
    const float roll = std::atan2(delta.y, delta.x) * kRadiansToDegrees;
    if (delta.x <= 0.0F || std::abs(roll) > params.max_center_line_angle_deg) {
        ++diagnostics.rejected_pair_geometry;
        return std::nullopt;
    }
    const cv::Point2f normal(-delta.y / distance, delta.x / distance);
    alignUncertainLight(left, lights[i], normal);
    alignUncertainLight(right, lights[j], normal);
    const float mean_length = (left.length + right.length) / 2.0F;
    const float length_ratio =
        std::max(left.length, right.length) / std::min(left.length, right.length);
    const float angle_difference = std::abs(left.tilt_deg - right.tilt_deg);
    const float distance_ratio = distance / mean_length;
    const bool reliable_pair = left.quality >= 1.0F && right.quality >= 1.0F;
    if (std::min(left.length, right.length) < params.min_light_length ||
        length_ratio > params.max_length_ratio ||
        (reliable_pair && angle_difference > params.max_angle_difference_deg) ||
        (left.quality >= 1.0F &&
         std::abs(left.tilt_deg + roll) > params.max_rectangular_error_deg) ||
        (right.quality >= 1.0F &&
         std::abs(right.tilt_deg + roll) > params.max_rectangular_error_deg) ||
        std::abs(delta.y) / mean_length > params.max_vertical_offset_ratio ||
        distance_ratio < params.min_pair_distance_ratio ||
        distance_ratio > params.max_pair_distance_ratio) {
        ++diagnostics.rejected_pair_geometry;
        return std::nullopt;
    }
    if (hasIntermediateLight(lights, i, j, mean_length, params.max_length_ratio)) {
        ++diagnostics.rejected_intermediate;
        return std::nullopt;
    }
    Detection detection;
    detection.color = left.color;
    detection.armor_type =
        distance_ratio >= params.big_armor_distance_ratio ? ArmorType::big : ArmorType::small;
    detection.corners = {left.center - left.axis * (left.length / 2.0F),
                         right.center - right.axis * (right.length / 2.0F),
                         right.center + right.axis * (right.length / 2.0F),
                         left.center + left.axis * (left.length / 2.0F)};
    if (!validCorners(detection.corners, bgr.size())) {
        ++diagnostics.rejected_corners;
        return std::nullopt;
    }
    detection.corners_valid = true;
    detection.source = DetectionSource::traditional;
    detection.score_kind = DetectionScoreKind::heuristic;
    // 取较差的一侧，防止一根清晰灯条掩盖另一根的端点不确定性。
    detection.corner_quality = std::min(left.quality, right.quality);
    const float mean_quality = (left.quality + right.quality) / 2.0F;
    const float length_error = (length_ratio - 1.0F) / (params.max_length_ratio - 1.0F);
    detection.geometry_score = std::clamp(
        1.0F - (0.35F * length_error + 0.25F * std::abs(roll) / params.max_center_line_angle_deg +
                0.2F * std::min(1.0F, angle_difference / params.max_angle_difference_deg) +
                0.2F * (1.0F - mean_quality)),
        0.0F, 1.0F);
    const float pattern =
        (params.pattern_score_weight > 0.0F || params.min_blob_pattern_score > 0.0F)
            ? patternScore(bgr, left, right, params.pattern_brightness_threshold)
            : 0.0F;
    detection.confidence = (1.0F - params.pattern_score_weight) * detection.geometry_score +
                           params.pattern_score_weight * pattern;
    if (detection.confidence < params.min_pair_score ||
        (!reliable_pair && pattern < params.min_blob_pattern_score)) {
        ++diagnostics.rejected_score;
        return std::nullopt;
    }
    const std::vector<cv::Point2f> points(detection.corners.begin(), detection.corners.end());
    detection.bounding_box = cv::boundingRect(points) & cv::Rect({}, bgr.size());
    return ArmorCandidate{i, j, detection};
}

// 共用灯条之外，也排除不同灯条构成的重叠/交叉配对。
bool overlaps(const Detection& a, const Detection& b) {
    if (a.color != b.color || (a.bounding_box & b.bounding_box).empty())
        return false;
    const std::vector<cv::Point2f> pa(a.corners.begin(), a.corners.end());
    const std::vector<cv::Point2f> pb(b.corners.begin(), b.corners.end());
    std::vector<cv::Point2f> intersection;
    const float area = cv::intersectConvexConvex(pa, pb, intersection);
    return area > 0.25 * std::min(cv::contourArea(pa), cv::contourArea(pb));
}

} // namespace

TraditionalArmorDetector::TraditionalArmorDetector(TraditionalDetectorParams params)
    : params_(params) {
    const std::initializer_list<double> values{params.min_light_area,
                                               params.min_light_length,
                                               params.min_light_aspect_ratio,
                                               params.max_light_aspect_ratio,
                                               params.max_light_tilt_deg,
                                               params.reliable_light_aspect_ratio,
                                               params.max_length_ratio,
                                               params.max_angle_difference_deg,
                                               params.max_center_line_angle_deg,
                                               params.max_rectangular_error_deg,
                                               params.min_pair_distance_ratio,
                                               params.max_pair_distance_ratio,
                                               params.max_vertical_offset_ratio,
                                               params.big_armor_distance_ratio,
                                               params.min_pair_score,
                                               params.pattern_score_weight,
                                               params.min_blob_pattern_score};
    const bool finite = std::all_of(values.begin(), values.end(),
                                    [](double value) { return std::isfinite(value); });
    const auto valid_angle = [](float angle) { return angle > 0.0F && angle < 90.0F; };
    if (!finite || params.brightness_threshold < 0 || params.brightness_threshold > 255 ||
        params.color_difference_threshold < 1 || params.color_difference_threshold > 255 ||
        params.pattern_brightness_threshold < 0 || params.pattern_brightness_threshold > 255 ||
        params.min_light_area <= 0.0 || params.min_light_length <= 0.0F ||
        params.min_light_aspect_ratio <= 0.0F ||
        params.max_light_aspect_ratio < params.reliable_light_aspect_ratio ||
        params.reliable_light_aspect_ratio <= std::max(1.0F, params.min_light_aspect_ratio) ||
        !valid_angle(params.max_light_tilt_deg) || params.max_light_tilt_deg > 45.0F ||
        params.max_length_ratio <= 1.0F || !valid_angle(params.max_angle_difference_deg) ||
        !valid_angle(params.max_center_line_angle_deg) ||
        !valid_angle(params.max_rectangular_error_deg) || params.min_pair_distance_ratio <= 0.0F ||
        params.max_pair_distance_ratio <= params.min_pair_distance_ratio ||
        params.max_vertical_offset_ratio <= 0.0F || params.min_pair_score < 0.0F ||
        params.min_pair_score > 1.0F || params.pattern_score_weight < 0.0F ||
        params.pattern_score_weight >= 1.0F || params.min_blob_pattern_score < 0.0F ||
        params.min_blob_pattern_score > 1.0F ||
        params.big_armor_distance_ratio <= params.min_pair_distance_ratio ||
        params.big_armor_distance_ratio >= params.max_pair_distance_ratio ||
        (params.target_color != ArmorColor::unknown && params.target_color != ArmorColor::red &&
         params.target_color != ArmorColor::blue)) {
        throw std::invalid_argument("Invalid traditional armor detector parameters");
    }
}

DetectionFrame TraditionalArmorDetector::detect(const ImageFrame& frame) {
    TraditionalDetectorDiagnostics diagnostics;
    return detect(frame, diagnostics);
}

DetectionFrame TraditionalArmorDetector::detect(const ImageFrame& frame,
                                                TraditionalDetectorDiagnostics& diagnostics) {
    diagnostics = {};
    DetectionFrame result;
    result.source_sequence = frame.sequence;
    result.capture_time = frame.capture_time;
    if (frame.image.empty())
        return result;
    if (frame.image.type() != CV_8UC3 && frame.image.type() != CV_8UC4) {
        throw std::invalid_argument("Detector expects an 8-bit BGR or BGRA image");
    }
    cv::Mat bgr = frame.image;
    if (frame.image.channels() == 4)
        cv::cvtColor(frame.image, bgr, cv::COLOR_BGRA2BGR);
    const auto lights = findLights(bgr, params_, diagnostics);
    std::vector<ArmorCandidate> candidates;
    for (std::size_t i = 0; i < lights.size(); ++i) {
        for (std::size_t j = i + 1; j < lights.size(); ++j) {
            if (auto candidate = matchLights(bgr, lights, i, j, params_, diagnostics)) {
                candidates.push_back(std::move(*candidate));
            }
        }
    }
    diagnostics.candidates = candidates.size();
    std::stable_sort(candidates.begin(), candidates.end(),
                     [](const ArmorCandidate& a, const ArmorCandidate& b) {
                         return a.detection.confidence > b.detection.confidence;
                     });
    std::vector<bool> used(lights.size(), false);
    for (const auto& candidate : candidates) {
        const bool overlap = std::any_of(
            result.detections.begin(), result.detections.end(),
            [&](const Detection& detection) { return overlaps(candidate.detection, detection); });
        if (used[candidate.left] || used[candidate.right] || overlap) {
            ++diagnostics.suppressed;
            continue;
        }
        result.detections.push_back(candidate.detection);
        used[candidate.left] = used[candidate.right] = true;
    }
    std::sort(
        result.detections.begin(), result.detections.end(),
        [](const Detection& a, const Detection& b) { return a.bounding_box.x < b.bounding_box.x; });
    return result;
}

bool TraditionalArmorDetector::refineCorners(const ImageFrame& frame, Detection& detection) {
    if (frame.image.empty() || !detection.corners_valid ||
        !validCorners(detection.corners, frame.image.size()))
        return false;
    const auto& points = detection.corners;
    const float mean_length = static_cast<float>(
        (cv::norm(points[3] - points[0]) + cv::norm(points[2] - points[1])) / 2.0);
    if (mean_length < params_.min_light_length)
        return false;
    const std::vector<cv::Point2f> polygon(points.begin(), points.end());
    const auto bounds = cv::boundingRect(polygon);
    const int margin = cvCeil(mean_length);
    const auto roi = cv::Rect(bounds.x - margin, bounds.y - margin, bounds.width + 2 * margin,
                              bounds.height + 2 * margin) &
                     cv::Rect({}, frame.image.size());
    ImageFrame local = frame;
    local.image = frame.image(roi);
    const auto result = detect(local);
    const cv::Point2f offset(static_cast<float>(roi.x), static_cast<float>(roi.y));
    const Detection* best = nullptr;
    double best_error = 4.0 * mean_length;
    for (const auto& candidate : result.detections) {
        if (candidate.corner_quality < 1.0F ||
            (detection.color != ArmorColor::unknown && candidate.color != detection.color))
            continue;
        double error = 0.0;
        bool nearby = true;
        for (std::size_t i = 0; i < points.size(); ++i) {
            const double distance = cv::norm(candidate.corners[i] + offset - points[i]);
            nearby = nearby && distance <= mean_length * 0.35;
            error += distance;
        }
        if (nearby && error < best_error) {
            best_error = error;
            best = &candidate;
        }
    }
    if (!best)
        return false;
    for (std::size_t i = 0; i < points.size(); ++i) {
        detection.corners[i] = best->corners[i] + offset;
    }
    detection.bounding_box = best->bounding_box + roi.tl();
    detection.geometry_score = best->geometry_score;
    detection.corner_quality = best->corner_quality;
    detection.corners_valid = true;
    return true;
}

} // namespace rmvision::components
