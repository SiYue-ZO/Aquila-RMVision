#include "components/detector/detector.hpp"
#include "components/tracker/tracker.hpp"
#include "runtime/detector_config.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>

namespace {

using rmvision::ArmorColor;
using rmvision::ArmorType;
using rmvision::ImageFrame;
using rmvision::components::TraditionalArmorDetector;
using rmvision::components::TraditionalDetectorParams;

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

ImageFrame makeFrame(const cv::Mat& image) {
    return {.sequence = 42, .image = image, .capture_time = std::chrono::steady_clock::now()};
}

void drawLight(cv::Mat& image, cv::Point2f center, cv::Scalar color, float angle = 0.0F,
               float length = 50.0F) {
    cv::Point2f vertices[4];
    cv::RotatedRect(center, {8.0F, length}, angle).points(vertices);
    std::vector<cv::Point> polygon;
    for (const auto& vertex : vertices) {
        polygon.emplace_back(vertex);
    }
    cv::fillConvexPoly(image, polygon, color);
}

void testSample(const std::string& path) {
    // 固定样例验证数量、颜色、四点位置和输入不变性，防止只在合成图上通过。
    const auto image = cv::imread(path, cv::IMREAD_UNCHANGED);
    require(!image.empty(), "Cannot load sample: " + path);
    const auto original = image.clone();
    const auto frame = makeFrame(image);
    TraditionalArmorDetector detector;
    const auto result = detector.detect(frame);
    require(result.source_sequence == frame.sequence && result.capture_time == frame.capture_time,
            "Frame metadata was lost");
    require(result.detections.size() == 3, "Sample must contain exactly three armors");
    const ArmorColor expected_colors[]{ArmorColor::red, ArmorColor::blue, ArmorColor::red};
    const cv::Point2f expected_centers[]{{197.0F, 218.0F}, {695.0F, 299.0F}, {1149.0F, 211.0F}};
    for (std::size_t i = 0; i < result.detections.size(); ++i) {
        const auto& armor = result.detections[i];
        require(armor.color == expected_colors[i], "Wrong sample color or output order");
        require(armor.target_id == 0, "Detector must not invent a digit or target ID");
        require(armor.confidence >= 0.0F && armor.confidence <= 1.0F, "Invalid heuristic score");
        require(armor.source == rmvision::DetectionSource::traditional &&
                    armor.score_kind == rmvision::DetectionScoreKind::heuristic &&
                    armor.corners_valid && armor.geometry_score >= 0.0F &&
                    armor.geometry_score <= 1.0F && armor.corner_quality >= 0.0F &&
                    armor.corner_quality <= 1.0F,
                "Invalid observation quality metadata");
        const auto& p = armor.corners;
        require(p[0].y < p[3].y && p[1].y < p[2].y && p[0].x + p[3].x < p[1].x + p[2].x,
                "Corners must be TL, TR, BR, BL");
        const cv::Point2f center = (p[0] + p[1] + p[2] + p[3]) * 0.25F;
        require(cv::norm(center - expected_centers[i]) < 12.0, "Sample geometry is misplaced");
        const auto bounds = armor.bounding_box & cv::Rect(0, 0, image.cols, image.rows);
        require(bounds == armor.bounding_box && bounds.area() > 0, "Invalid image bounds");
    }
    require(cv::norm(image, original, cv::NORM_INF) == 0.0, "Detector modified its input");

    for (const auto color : {ArmorColor::red, ArmorColor::blue}) {
        TraditionalDetectorParams params;
        params.target_color = color;
        const auto filtered = TraditionalArmorDetector(params).detect(frame);
        require(filtered.detections.size() == (color == ArmorColor::red ? 2U : 1U),
                "Color filter returned an incorrect count");
        require(std::all_of(filtered.detections.begin(), filtered.detections.end(),
                            [color](const auto& armor) { return armor.color == color; }),
                "Color filter returned the opposite color");
    }
}

void testSyntheticGeometry() {
    TraditionalArmorDetector detector;
    // 模拟过曝灯条的红黄亮芯和蓝青亮芯，白色目标应被颜色差分排除。
    const cv::Scalar red(0, 255, 255);
    const cv::Scalar blue(255, 255, 0);
    cv::Mat image = cv::Mat::zeros(240, 600, CV_8UC3);
    drawLight(image, {60, 100}, red);
    drawLight(image, {160, 100}, red);
    drawLight(image, {360, 130}, blue);
    drawLight(image, {460, 130}, blue);
    const auto result = detector.detect(makeFrame(image));
    require(result.detections.size() == 2, "Separate red and blue pairs must both be detected");

    image.setTo(cv::Scalar());
    drawLight(image, {80, 90}, red, 18.0F, 70.0F);
    drawLight(image, {180, 120}, red, 18.0F, 70.0F);
    require(detector.detect(makeFrame(image)).detections.size() == 1,
            "Parallel tilted lights must remain detectable");

    image.setTo(cv::Scalar());
    drawLight(image, {60, 100}, red);
    drawLight(image, {250, 100}, red);
    const auto big = detector.detect(makeFrame(image));
    require(big.detections.size() == 1 && big.detections.front().armor_type == ArmorType::big,
            "Wide armor pair must be marked as geometrically big");

    image.setTo(cv::Scalar());
    for (float x : {60.0F, 110.0F, 160.0F}) {
        drawLight(image, {x, 100}, red);
    }
    require(detector.detect(makeFrame(image)).detections.size() == 1,
            "Three lights must not produce overlapping pairs sharing a light");

    // 单灯条、异色配对、白色高亮、角度或长度不一致均不能组成装甲板。
    for (int scenario = 0; scenario < 7; ++scenario) {
        image.setTo(cv::Scalar());
        if (scenario == 0) {
            drawLight(image, {80, 100}, red);
        } else if (scenario == 1) {
            drawLight(image, {80, 100}, red);
            drawLight(image, {180, 100}, blue);
        } else if (scenario == 2) {
            drawLight(image, {80, 100}, {255, 255, 255});
            drawLight(image, {180, 100}, {255, 255, 255});
        } else if (scenario == 3) {
            drawLight(image, {80, 100}, red, 90.0F);
            drawLight(image, {180, 100}, red, 90.0F);
        } else if (scenario == 4) {
            drawLight(image, {80, 100}, red, 20.0F);
            drawLight(image, {180, 100}, red, -20.0F);
        } else if (scenario == 5) {
            drawLight(image, {80, 100}, red, 0.0F, 80.0F);
            drawLight(image, {180, 100}, red, 0.0F, 30.0F);
        } else {
            drawLight(image, {80, 60}, red);
            drawLight(image, {180, 140}, red);
        }
        require(detector.detect(makeFrame(image)).detections.empty(),
                "False armor in negative scenario " + std::to_string(scenario));
    }
}

void testColorAndBlur() {
    TraditionalArmorDetector detector;
    cv::Mat image = cv::Mat::zeros(240, 500, CV_8UC3);
    // 鲜红和鲜蓝的灰度都很低，必须仍能形成观测。
    for (const auto& color : {cv::Scalar(0, 0, 255), cv::Scalar(255, 0, 0)}) {
        image.setTo(cv::Scalar());
        drawLight(image, {100, 120}, color);
        drawLight(image, {220, 120}, color);
        const auto result = detector.detect(makeFrame(image));
        require(result.detections.size() == 1, "Saturated colored lights were lost");
        cv::Mat bgra;
        cv::cvtColor(image, bgra, cv::COLOR_BGR2BGRA);
        require(detector.detect(makeFrame(bgra)).detections.size() == 1,
                "BGRA and BGR must produce equivalent observations");
    }

    image.setTo(cv::Scalar());
    drawLight(image, {100, 120}, {0, 0, 255});
    drawLight(image, {220, 120}, {0, 0, 255});
    cv::putText(image, "3", {136, 150}, cv::FONT_HERSHEY_SIMPLEX, 2.5, {255, 255, 255}, 5);
    // 水平方向模糊后，灯条接近方形，长边方向不能直接作为灯条方向。
    cv::GaussianBlur(image, image, {61, 1}, 15.0, 0.0);
    TraditionalDetectorParams params;
    params.brightness_threshold = 40;
    params.color_difference_threshold = 30;
    params.pattern_brightness_threshold = 60;
    rmvision::components::TraditionalDetectorDiagnostics diagnostics;
    const auto blurred = TraditionalArmorDetector(params).detect(makeFrame(image), diagnostics);
    require(blurred.detections.size() == 1 && diagnostics.uncertain_lights == 2,
            "Motion-broadened lights must remain detectable");
    require(blurred.detections.front().corners_valid &&
                blurred.detections.front().corner_quality < 1.0F,
            "Blurred corners must expose their uncertainty");

    image.setTo(cv::Scalar());
    cv::circle(image, {100, 120}, 18, {0, 0, 255}, cv::FILLED);
    cv::circle(image, {220, 120}, 18, {0, 0, 255}, cv::FILLED);
    require(detector.detect(makeFrame(image)).detections.empty(),
            "Two round reflections without armor pattern must not form armor");
    cv::line(image, {140, 120}, {180, 120}, {255, 255, 255}, 3);
    require(detector.detect(makeFrame(image)).detections.empty(),
            "A horizontal chassis reflection must not substitute for an armor pattern");
    cv::rectangle(image, {270, 90, 50, 8}, {0, 0, 255}, cv::FILLED);
    cv::rectangle(image, {370, 90, 50, 8}, {0, 0, 255}, cv::FILLED);
    require(detector.detect(makeFrame(image)).detections.empty(),
            "Horizontal reflections must not form armor");

    image.setTo(cv::Scalar());
    cv::rectangle(image, {85, 102, 31, 37}, {0, 0, 255}, cv::FILLED);
    cv::rectangle(image, {205, 102, 31, 37}, {0, 0, 255}, cv::FILLED);
    cv::putText(image, "3", {137, 145}, cv::FONT_HERSHEY_SIMPLEX, 1.8, {255, 255, 255}, 4);
    const auto broad = detector.detect(makeFrame(image));
    require(broad.detections.size() == 1 && broad.detections.front().corner_quality < 1.0F,
            "Default parameters must accept bright broad lights with armor pattern support");
}

void testPairSelectionAndRefinement() {
    TraditionalArmorDetector detector;
    cv::Mat image = cv::Mat::zeros(260, 500, CV_8UC3);
    for (float x : {100.0F, 200.0F, 300.0F})
        drawLight(image, {x, 120}, {0, 0, 255});
    cv::putText(image, "3", {228, 142}, cv::FONT_HERSHEY_SIMPLEX, 1.5, {255, 255, 255}, 3);
    const auto result = detector.detect(makeFrame(image));
    require(result.detections.size() == 1 && result.detections.front().bounding_box.x > 180,
            "Armor pattern must outrank an equally plausible empty gap");

    auto network = result.detections.front();
    network.source = rmvision::DetectionSource::neural;
    network.score_kind = rmvision::DetectionScoreKind::model;
    network.confidence = 0.93F;
    network.target_id = 3;
    for (auto& point : network.corners)
        point += cv::Point2f(2, 2);
    require(detector.refineCorners(makeFrame(image), network),
            "Nearby clear lights must refine a supplied observation");
    require(network.source == rmvision::DetectionSource::neural &&
                network.score_kind == rmvision::DetectionScoreKind::model &&
                network.confidence == 0.93F && network.target_id == 3 &&
                cv::norm(network.corners.front() - result.detections.front().corners.front()) < 1.0,
            "Refinement changed model metadata or failed to restore original coordinates");
    const auto original = network;
    const auto black = cv::Mat::zeros(image.size(), CV_8UC3);
    require(!detector.refineCorners(makeFrame(black), network) &&
                network.corners == original.corners &&
                network.bounding_box == original.bounding_box &&
                network.confidence == original.confidence &&
                network.corner_quality == original.corner_quality,
            "Failed refinement must preserve the input observation");
    network.corners[0] = {-1.0F, 0.0F};
    require(!detector.refineCorners(makeFrame(image), network),
            "Out-of-frame observations must not be silently clipped during refinement");
}

void testContractsAndConfiguration(const std::filesystem::path& directory) {
    std::filesystem::create_directories(directory);
    const auto config = directory / "detector_config.yml";
    TraditionalDetectorParams params;
    params.target_color = ArmorColor::blue;
    params.brightness_threshold = 191;
    rmvision::runtime::saveDetectorParams(config.string(), params);
    const auto loaded = rmvision::runtime::loadDetectorParams(config.string());
    require(loaded.target_color == params.target_color &&
                loaded.brightness_threshold == params.brightness_threshold &&
                loaded.min_light_aspect_ratio == params.min_light_aspect_ratio &&
                loaded.min_blob_pattern_score == params.min_blob_pattern_score,
            "Resolved parameter snapshot did not round-trip");
    for (const auto* body : {"config_version: 1\n", "config_version: 2\nunknown_key: 1\n",
                             "config_version: 2\nbrightness_threshold: 1.5\n",
                             "config_version: 2\nmax_length_ratio: 1.0\n"}) {
        std::ofstream stream(config);
        stream << "%YAML:1.0\n---\n" << body;
        stream.close();
        bool rejected = false;
        try {
            rmvision::runtime::loadDetectorParams(config.string());
        } catch (const std::exception&) {
            rejected = true;
        }
        require(rejected, "Invalid configuration was silently accepted");
    }
    const auto frame = makeFrame({});
    TraditionalArmorDetector implementation;
    rmvision::components::Detector& backend = implementation;
    rmvision::components::SimpleTracker tracker;
    auto observation = backend.detect(frame);
    const auto state = tracker.update(observation);
    require(observation.source_sequence == frame.sequence &&
                observation.capture_time == frame.capture_time && !state.valid &&
                state.source_sequence == frame.sequence &&
                state.capture_time == frame.capture_time && state.track_id == 0,
            "Empty observations must retain time and sequence through the tracker interface");
    rmvision::components::TraditionalDetectorDiagnostics diagnostics;
    diagnostics.lights = 99;
    implementation.detect(frame, diagnostics);
    require(diagnostics.lights == 0, "Per-frame diagnostics accumulated stale data");
    for (const auto value : {0.65F, std::numeric_limits<float>::quiet_NaN()}) {
        auto invalid = params;
        invalid.reliable_light_aspect_ratio = value;
        bool rejected = false;
        try {
            const TraditionalArmorDetector detector(invalid);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        require(rejected, "Invalid direction reliability threshold was accepted");
    }
}

void testInvalidInputs() {
    TraditionalArmorDetector detector;
    require(detector.detect(makeFrame({})).detections.empty(), "Empty image must return no target");
    require(detector.detect(makeFrame(cv::Mat::zeros(120, 200, CV_8UC3))).detections.empty(),
            "Black image must return no target");
    bool rejected = false;
    try {
        detector.detect(makeFrame(cv::Mat::zeros(120, 200, CV_8UC1)));
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(rejected, "Unsupported pixel formats must be rejected explicitly");
    TraditionalDetectorParams params;
    params.max_length_ratio = 1.0F;
    rejected = false;
    try {
        TraditionalArmorDetector invalid(params);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(rejected, "Invalid parameters must be rejected before processing");
}

} // namespace

int main(int argc, char* argv[]) {
    try {
        require(argc == 2 || argc == 3,
                "Usage: detector_test <2color.png> [test-output-directory]");
        testSample(argv[1]);
        testSyntheticGeometry();
        testColorAndBlur();
        testPairSelectionAndRefinement();
        testContractsAndConfiguration(argc == 3 ? argv[2] : "build/tests/detector_contracts");
        testInvalidInputs();
        std::cout
            << "Detector regression passed: images, blur, distractors, refinement, config and "
               "message contracts\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Detector regression failed: " << error.what() << '\n';
        return 1;
    }
}
