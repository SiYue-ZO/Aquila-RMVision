#include "runtime/application.hpp"

#include "components/detector/detector.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace rmvision::runtime {

int Application::run() {
    ImageFrame frame;
    frame.sequence = 1;
    frame.image = cv::imread(options_.image_path, cv::IMREAD_COLOR);
    frame.capture_time = std::chrono::steady_clock::now();
    if (frame.image.empty()) {
        throw std::runtime_error("Cannot read image: " + options_.image_path);
    }
    const std::filesystem::path output_path(options_.output_path);
    if (std::filesystem::exists(output_path) &&
        std::filesystem::equivalent(options_.image_path, output_path)) {
        throw std::runtime_error("Output must not overwrite the input image");
    }

    components::TraditionalDetectorParams params;
    params.target_color = options_.target_color;
    components::TraditionalArmorDetector detector(params);
    const auto start = std::chrono::steady_clock::now();
    const auto result = detector.detect(frame);
    const auto elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start);

    // 可视化与文件读写放在应用层，检测组件只返回观测结果。
    auto annotated = frame.image.clone();
    std::size_t red_count = 0;
    std::size_t blue_count = 0;
    for (std::size_t i = 0; i < result.detections.size(); ++i) {
        const auto& armor = result.detections[i];
        const bool red = armor.color == ArmorColor::red;
        red ? ++red_count : ++blue_count;
        const auto color = red ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 120, 0);
        const char* name = red ? "red" : "blue";
        const char* type = armor.armor_type == ArmorType::big ? "big" : "small";
        std::vector<cv::Point> polygon;
        for (std::size_t corner = 0; corner < armor.corners.size(); ++corner) {
            const auto point = cv::Point(armor.corners[corner]);
            polygon.push_back(point);
            cv::circle(annotated, point, 4, {0, 255, 255}, cv::FILLED, cv::LINE_AA);
            cv::putText(annotated, std::to_string(corner), point + cv::Point(5, -5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 255}, 1, cv::LINE_AA);
        }
        cv::polylines(annotated, polygon, true, color, 2, cv::LINE_AA);
        const auto label = cv::format("#%zu %s %s score=%.2f", i + 1, name, type, armor.confidence);
        const cv::Point label_position(std::max(5, armor.bounding_box.x),
                                       std::max(25, armor.bounding_box.y - 12));
        cv::putText(annotated, label, label_position, cv::FONT_HERSHEY_SIMPLEX, 0.65,
                    {0, 0, 0}, 4, cv::LINE_AA);
        cv::putText(annotated, label, label_position, cv::FONT_HERSHEY_SIMPLEX, 0.65,
                    color, 2, cv::LINE_AA);
        std::cout << "armor=" << i + 1 << " color=" << name << " type=" << type
                  << " score=" << std::fixed << std::setprecision(3) << armor.confidence
                  << " box=" << armor.bounding_box << '\n';
    }

    if (!output_path.parent_path().empty()) {
        std::filesystem::create_directories(output_path.parent_path());
    }
    if (!cv::imwrite(output_path.string(), annotated)) {
        throw std::runtime_error("Cannot save result: " + output_path.string());
    }
    std::cout << "detections=" << result.detections.size() << " red=" << red_count
              << " blue=" << blue_count << " elapsed_ms=" << elapsed.count() << '\n';
    std::cout << "result=" << std::filesystem::absolute(output_path).string() << '\n';

    // 默认只保存图片，远程终端和无桌面环境也能运行。
    if (options_.show_window) {
        cv::imshow("Aquila armor detection", annotated);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }
    return 0;
}

}  // namespace rmvision::runtime
