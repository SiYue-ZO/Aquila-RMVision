#include "components/detector/detector.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
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
    return {.sequence = 42,
            .image = image,
            .capture_time = std::chrono::steady_clock::now()};
}

void drawLight(cv::Mat& image, cv::Point2f center, cv::Scalar color,
               float angle = 0.0F, float length = 50.0F) {
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
        require(armor.confidence >= 0.0F && armor.confidence <= 1.0F, "Invalid geometry score");
        const auto& p = armor.corners;
        require(p[0].y < p[3].y && p[1].y < p[2].y &&
                    p[0].x + p[3].x < p[1].x + p[2].x,
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

}  // namespace

int main(int argc, char* argv[]) {
    try {
        require(argc == 2, "Usage: detector_test <2color.png>");
        testSample(argv[1]);
        testSyntheticGeometry();
        testInvalidInputs();
        std::cout << "Detector regression passed: sample, geometry, distractors and input checks\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Detector regression failed: " << error.what() << '\n';
        return 1;
    }
}
