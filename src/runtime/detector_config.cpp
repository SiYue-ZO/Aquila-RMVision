#include "runtime/detector_config.hpp"

#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <set>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace rmvision::runtime {
namespace {

// 同一字段表用于读取和快照，避免报告与实际运行参数脱节。
template <typename Params, typename Visitor> void visitParams(Params& params, Visitor visitor) {
    visitor("brightness_threshold", params.brightness_threshold);
    visitor("color_difference_threshold", params.color_difference_threshold);
    visitor("min_light_area", params.min_light_area);
    visitor("min_light_length", params.min_light_length);
    visitor("min_light_aspect_ratio", params.min_light_aspect_ratio);
    visitor("max_light_aspect_ratio", params.max_light_aspect_ratio);
    visitor("max_light_tilt_deg", params.max_light_tilt_deg);
    visitor("reliable_light_aspect_ratio", params.reliable_light_aspect_ratio);
    visitor("max_length_ratio", params.max_length_ratio);
    visitor("max_angle_difference_deg", params.max_angle_difference_deg);
    visitor("max_center_line_angle_deg", params.max_center_line_angle_deg);
    visitor("max_rectangular_error_deg", params.max_rectangular_error_deg);
    visitor("min_pair_distance_ratio", params.min_pair_distance_ratio);
    visitor("max_pair_distance_ratio", params.max_pair_distance_ratio);
    visitor("max_vertical_offset_ratio", params.max_vertical_offset_ratio);
    visitor("big_armor_distance_ratio", params.big_armor_distance_ratio);
    visitor("min_pair_score", params.min_pair_score);
    visitor("pattern_score_weight", params.pattern_score_weight);
    visitor("pattern_brightness_threshold", params.pattern_brightness_threshold);
    visitor("min_blob_pattern_score", params.min_blob_pattern_score);
}

} // namespace

components::TraditionalDetectorParams loadDetectorParams(const std::string& path) {
    components::TraditionalDetectorParams params;
    if (path.empty())
        return params;
    cv::FileStorage storage(path, cv::FileStorage::READ);
    if (!storage.isOpened())
        throw std::runtime_error("Cannot open detector config: " + path);
    const auto version = storage["config_version"];
    if (!version.isInt() || static_cast<int>(version) != 2) {
        throw std::invalid_argument("Detector config requires config_version: 2");
    }
    std::set<std::string> allowed{"config_version", "target_color"};
    visitParams(params, [&](const char* name, auto& value) {
        allowed.insert(name);
        const auto node = storage[name];
        if (node.empty())
            return;
        if (!node.isInt() && !node.isReal()) {
            throw std::invalid_argument(std::string("Expected numeric detector parameter: ") +
                                        name);
        }
        const double number = node.real();
        using Value = std::remove_reference_t<decltype(value)>;
        if (!std::isfinite(number) || number < std::numeric_limits<Value>::lowest() ||
            number > std::numeric_limits<Value>::max()) {
            throw std::invalid_argument(std::string("Detector parameter out of range: ") + name);
        }
        if constexpr (std::is_integral_v<Value>) {
            if (std::trunc(number) != number) {
                throw std::invalid_argument(std::string("Expected integer detector parameter: ") +
                                            name);
            }
        }
        value = static_cast<Value>(number);
    });
    if (!storage["target_color"].empty()) {
        const std::string color = static_cast<std::string>(storage["target_color"]);
        if (color == "red")
            params.target_color = ArmorColor::red;
        else if (color == "blue")
            params.target_color = ArmorColor::blue;
        else if (color != "both")
            throw std::invalid_argument("target_color must be both/red/blue");
    }
    std::set<std::string> seen;
    for (const auto& node : storage.root()) {
        const auto name = node.name();
        if (!allowed.contains(name) || !seen.insert(name).second) {
            throw std::invalid_argument("Unknown or duplicate detector parameter: " + name);
        }
    }
    const components::TraditionalArmorDetector validated(params);
    return params;
}

void saveDetectorParams(const std::string& path,
                        const components::TraditionalDetectorParams& params) {
    const components::TraditionalArmorDetector validated(params);
    cv::FileStorage storage(path, cv::FileStorage::WRITE);
    if (!storage.isOpened())
        throw std::runtime_error("Cannot save detector config: " + path);
    storage << "config_version" << 2;
    storage << "target_color"
            << (params.target_color == ArmorColor::red    ? "red"
                : params.target_color == ArmorColor::blue ? "blue"
                                                          : "both");
    visitParams(params, [&](const char* name, const auto& value) { storage << name << value; });
    storage.release();
}

} // namespace rmvision::runtime
