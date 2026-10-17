#include "components/detector/detector.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using rmvision::ArmorColor;
using rmvision::ArmorType;

/** @brief 绘制当前帧检测结果，编号仅表示本帧候选顺序。 */
void annotate(cv::Mat& image, const rmvision::DetectionFrame& result, double elapsed_ms) {
    const auto header = cv::format("frame=%llu  armors=%zu  detector=%.2f ms",
        static_cast<unsigned long long>(result.source_sequence), result.detections.size(), elapsed_ms);
    cv::putText(image, header, {20, 36}, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                {0, 0, 0}, 4, cv::LINE_AA);
    cv::putText(image, header, {20, 36}, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                {0, 255, 255}, 2, cv::LINE_AA);
    for (std::size_t i = 0; i < result.detections.size(); ++i) {
        const auto& armor = result.detections[i];
        const bool red = armor.color == ArmorColor::red;
        const auto color = red ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 160, 0);
        std::vector<cv::Point> polygon;
        for (const auto& corner : armor.corners) {
            polygon.emplace_back(corner);
            cv::circle(image, corner, 3, {0, 255, 255}, cv::FILLED, cv::LINE_AA);
        }
        cv::polylines(image, polygon, true, color, 2, cv::LINE_AA);
        const auto label = cv::format("#%zu %s %s %.2f", i + 1, red ? "red" : "blue",
            armor.armor_type == ArmorType::big ? "big" : "small", armor.confidence);
        const cv::Point origin(std::max(0, armor.bounding_box.x),
                               std::max(60, armor.bounding_box.y - 8));
        cv::putText(image, label, origin, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    {0, 0, 0}, 3, cv::LINE_AA);
        cv::putText(image, label, origin, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    color, 1, cv::LINE_AA);
    }
}

/** @brief 逐帧运行现有默认检测器，保存标注视频、观测明细和检测耗时统计。 */
int evaluateVideo(const std::filesystem::path& input, const std::filesystem::path& output) {
    if (!std::filesystem::is_regular_file(input)) {
        throw std::runtime_error("Input video does not exist: " + input.string());
    }
    std::filesystem::create_directories(output);
    const auto video_path = output / "annotated.mp4";
    if (std::filesystem::exists(video_path) && std::filesystem::equivalent(input, video_path)) {
        throw std::runtime_error("Output must not overwrite the input video");
    }
    cv::VideoCapture capture(input.string());
    if (!capture.isOpened()) {
        throw std::runtime_error("Cannot open video: " + input.string());
    }
    const double fps = capture.get(cv::CAP_PROP_FPS);
    const int expected_frames = cvRound(capture.get(cv::CAP_PROP_FRAME_COUNT));
    if (!std::isfinite(fps) || fps <= 0.0) {
        throw std::runtime_error("Video has an invalid frame rate");
    }

    std::ofstream frames_csv(output / "frames.csv");
    std::ofstream observations_csv(output / "detections.csv");
    if (!frames_csv || !observations_csv) {
        throw std::runtime_error("Cannot create CSV reports");
    }
    frames_csv << "frame,time_s,detections,red,blue,small,big,detector_ms\n";
    observations_csv << "frame,index,color,type,geometry_score,x,y,width,height"
                        ",tl_x,tl_y,tr_x,tr_y,br_x,br_y,bl_x,bl_y\n";
    frames_csv << std::fixed << std::setprecision(6);
    observations_csv << std::fixed << std::setprecision(6);

    rmvision::components::TraditionalArmorDetector detector;
    cv::VideoWriter writer;
    cv::Mat image;
    cv::Mat sheet(690, 1440, CV_8UC3, cv::Scalar(15, 15, 15));
    int sample_index = 0;
    std::size_t frame_index = 0;
    std::size_t detected_frames = 0;
    std::size_t total_detections = 0;
    std::size_t red_total = 0;
    std::size_t blue_total = 0;
    std::size_t empty_run = 0;
    std::size_t longest_empty_run = 0;
    std::size_t longest_empty_start = 0;
    std::size_t max_per_frame = 0;
    std::vector<double> timings;
    bool saved_positive = false;
    bool saved_negative = false;
    cv::Size image_size;
    const auto epoch = std::chrono::steady_clock::now();
    const auto wall_start = epoch;

    // 顺序解码全部帧，不跳帧；视频观测时间按原始帧率推进。
    while (capture.read(image)) {
        if (!writer.isOpened()) {
            image_size = image.size();
            writer.open(video_path.string(), cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                        fps, image_size);
            if (!writer.isOpened()) {
                throw std::runtime_error("Cannot create annotated MP4 video");
            }
        }
        if (image.size() != image_size) {
            throw std::runtime_error("Video frame size changed during decoding");
        }
        const double time_s = static_cast<double>(frame_index) / fps;
        rmvision::ImageFrame frame{
            .sequence = frame_index,
            .image = image,
            .capture_time = epoch + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                       std::chrono::duration<double>(time_s))};
        // 仅计时检测调用，视频解码、绘制和编码不纳入算法耗时。
        const auto start = std::chrono::steady_clock::now();
        const auto result = detector.detect(frame);
        const double elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
        timings.push_back(elapsed_ms);

        std::size_t red = 0, blue = 0, small = 0, big = 0;
        for (std::size_t i = 0; i < result.detections.size(); ++i) {
            const auto& armor = result.detections[i];
            armor.color == ArmorColor::red ? ++red : ++blue;
            armor.armor_type == ArmorType::big ? ++big : ++small;
            observations_csv << frame_index << ',' << i << ','
                << (armor.color == ArmorColor::red ? "red" : "blue") << ','
                << (armor.armor_type == ArmorType::big ? "big" : "small") << ','
                << armor.confidence << ',' << armor.bounding_box.x << ',' << armor.bounding_box.y
                << ',' << armor.bounding_box.width << ',' << armor.bounding_box.height;
            for (const auto& point : armor.corners) {
                observations_csv << ',' << point.x << ',' << point.y;
            }
            observations_csv << '\n';
        }
        const auto count = result.detections.size();
        frames_csv << frame_index << ',' << time_s << ',' << count << ',' << red << ',' << blue
                   << ',' << small << ',' << big << ',' << elapsed_ms << '\n';
        total_detections += count;
        red_total += red;
        blue_total += blue;
        max_per_frame = std::max(max_per_frame, count);
        if (count > 0) {
            ++detected_frames;
            empty_run = 0;
        } else if (++empty_run > longest_empty_run) {
            longest_empty_run = empty_run;
            longest_empty_start = frame_index + 1 - empty_run;
        }

        annotate(image, result, elapsed_ms);
        writer.write(image);
        if ((count > 0 && !saved_positive) || (count == 0 && !saved_negative)) {
            const auto filename = count > 0 ? "first_detected.jpg" : "first_no_detection.jpg";
            if (!cv::imwrite((output / filename).string(), image)) {
                throw std::runtime_error("Cannot save representative frame");
            }
            count > 0 ? saved_positive = true : saved_negative = true;
        }

        // 均匀抽取十二帧生成预览，完整结果仍保存在原帧率视频中。
        const int sample_frame = cvRound(std::max(0, expected_frames - 1) * sample_index / 11.0);
        if (sample_index < 12 && frame_index >= static_cast<std::size_t>(sample_frame)) {
            cv::Mat thumb;
            const double scale = std::min(360.0 / image.cols, 200.0 / image.rows);
            cv::resize(image, thumb, {}, scale, scale);
            const int x = (sample_index % 4) * 360;
            const int y = (sample_index / 4) * 230;
            thumb.copyTo(sheet(cv::Rect(x, y, thumb.cols, thumb.rows)));
            cv::putText(sheet, cv::format("%.2fs  count=%zu", time_s, count), {x + 4, y + 220},
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 255, 255}, 1, cv::LINE_AA);
            ++sample_index;
        }
        ++frame_index;
        if (frame_index % 100 == 0) {
            std::cout << "processed=" << frame_index << '/' << expected_frames << std::endl;
        }
    }
    if (frame_index == 0) {
        throw std::runtime_error("Video did not produce any decoded frames");
    }
    writer.release();
    frames_csv.close();
    observations_csv.close();
    if (!frames_csv || !observations_csv ||
        !cv::imwrite((output / "contact_sheet.jpg").string(), sheet)) {
        throw std::runtime_error("Cannot finish evaluation reports");
    }

    double total_ms = 0.0;
    for (const auto value : timings) {
        total_ms += value;
    }
    std::sort(timings.begin(), timings.end());
    const auto p95_index = static_cast<std::size_t>(std::ceil(timings.size() * 0.95)) - 1;
    const auto wall_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - wall_start).count();
    std::ofstream summary(output / "summary.txt");
    if (!summary) {
        throw std::runtime_error("Cannot create summary");
    }
    summary << std::fixed << std::setprecision(3)
        << "input=" << input.string() << '\n'
        << "parameters=TraditionalDetectorParams defaults (unchanged)\n"
        << "resolution=" << image_size.width << 'x' << image_size.height << '\n'
        << "fps=" << fps << '\n'
        << "metadata_frames=" << expected_frames << '\n'
        << "processed_frames=" << frame_index << '\n'
        << "duration_s=" << frame_index / fps << '\n'
        << "detected_frames=" << detected_frames << '\n'
        << "detected_frame_percent=" << 100.0 * detected_frames / frame_index << '\n'
        << "total_detections=" << total_detections << '\n'
        << "red_detections=" << red_total << '\n'
        << "blue_detections=" << blue_total << '\n'
        << "max_detections_per_frame=" << max_per_frame << '\n'
        << "longest_no_detection_frames=" << longest_empty_run << '\n'
        << "longest_no_detection_start_s=" << longest_empty_start / fps << '\n'
        << "longest_no_detection_duration_s=" << longest_empty_run / fps << '\n'
        << "detector_mean_ms=" << total_ms / frame_index << '\n'
        << "detector_p95_ms=" << timings[p95_index] << '\n'
        << "wall_time_s=" << wall_seconds << '\n'
        << "note=Detected-frame percentage is not precision or recall; no ground-truth labels.\n";
    summary.close();
    if (!summary) {
        throw std::runtime_error("Cannot finish summary");
    }
    std::cout << "finished: " << frame_index << " frames, " << detected_frames
              << " frames with detections, mean=" << total_ms / frame_index << " ms\n"
              << "reports=" << std::filesystem::absolute(output).string() << '\n';
    return 0;
}

}  // namespace

/** @brief 接收输入视频和输出目录，出错时返回非零退出码。 */
int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: detector_video_test <input-video> <output-directory>\n";
        return 1;
    }
    try {
        return evaluateVideo(argv[1], argv[2]);
    } catch (const std::exception& error) {
        std::cerr << "Video evaluation failed: " << error.what() << '\n';
        return 1;
    }
}
