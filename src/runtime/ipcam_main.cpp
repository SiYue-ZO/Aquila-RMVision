#pragma once

#include "components/camera/ipcam_source.hpp"
#include "components/detector/detector.hpp"

#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <csignal>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

volatile std::sig_atomic_t stop_requested = 0;

void requestStop(int) {
    stop_requested = 1;
}

cv::Mat annotate(const rmvision::ImageFrame& frame, const rmvision::DetectionFrame& result,
                 double elapsed_ms, const rmvision::components::IpCamFrameSource& camera,
                 const std::vector<rmvision::components::IpCamPoseResult>& poses, bool pnp_enabled) {
    auto image = frame.image.clone();
    for (std::size_t index = 0; index < result.detections.size(); ++index) {
        const auto& armor = result.detections[index];
        const bool red = armor.color == rmvision::ArmorColor::red;
        const cv::Scalar color = red ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 120, 0);
        std::vector<cv::Point> polygon;
        for (std::size_t i = 0; i < armor.corners.size(); ++i) {
            const cv::Point point(armor.corners[i]);
            polygon.push_back(point);
            cv::circle(image, point, 4, {0, 255, 255}, cv::FILLED, cv::LINE_AA);
            cv::putText(image, std::to_string(i), point + cv::Point(5, -5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 255}, 1, cv::LINE_AA);
        }
        cv::polylines(image, polygon, true, color, 2, cv::LINE_AA);
        if (pnp_enabled) {
            const auto& pose = poses[index];
            std::string label = cv::format("#%zu PnP failed", index);
            if (pose.valid) {
                camera.drawPnPAxes(image, pose);
                label = cv::format("#%zu XYZ=(%.2f,%.2f,%.2f)m D=%.2fm", index,
                                   pose.tvec.at<double>(0), pose.tvec.at<double>(1),
                                   pose.tvec.at<double>(2), pose.distance_m);
            }
            // 详细位姿按行显示，避免相邻目标的文字相互覆盖；框旁只放对应索引。
            const cv::Point origin(10, 55 + static_cast<int>(index) * 20);
            cv::putText(image, cv::format("#%zu", index),
                        {std::max(0, armor.bounding_box.x),
                         std::max(15, armor.bounding_box.y - 12)},
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 0}, 1, cv::LINE_AA);
            cv::putText(image, label, origin, cv::FONT_HERSHEY_SIMPLEX, 0.45,
                        {0, 0, 0}, 3, cv::LINE_AA);
            cv::putText(image, label, origin, cv::FONT_HERSHEY_SIMPLEX, 0.45,
                        pose.valid ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255),
                        1, cv::LINE_AA);
        }
    }
    const auto status = cv::format("frame=%llu armors=%zu detect=%.1f ms",
                                   static_cast<unsigned long long>(frame.sequence),
                                   result.detections.size(), elapsed_ms);
    cv::putText(image, status, {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.65,
                {0, 0, 0}, 4, cv::LINE_AA);
    cv::putText(image, status, {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.65,
                {0, 255, 0}, 1, cv::LINE_AA);
    if (pnp_enabled && result.detections.empty()) {
        cv::putText(image, "PnP: no target", {10, 55}, cv::FONT_HERSHEY_SIMPLEX,
                    0.55, {0, 255, 255}, 1, cv::LINE_AA);
    }
    return image;
}

}  // namespace

int main(int argc, char* argv[]) {
    const char* keys =
        "{help h ? | | Show usage}"
        "{url | | Required: MJPEG/RTSP URL or local video for replay}"
        "{timeout | 5000 | Open/read timeout in milliseconds}"
        "{color | both | Armor color: both, red or blue}"
        "{show | true | Show preview; press Q or Esc to stop}"
        "{frames | 0 | Stop after N frames; 0 means continuous}"
        "{pnp | false | Test PnP using parameters in ipcam_source.cpp}"
        "{log-every | 30 | Log every N frames; 1 logs every frame}"
        "{output | | Optional path to save the last annotated frame}";
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about("Aquila RMVision - temporary IPCam armor detection");
    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    try {
        rmvision::components::IpCamOptions camera_options;
        camera_options.url = parser.get<std::string>("url");
        camera_options.timeout_ms = parser.get<int>("timeout");
        const bool show = parser.get<bool>("show");
        const int frame_limit = parser.get<int>("frames");
        const auto output = parser.get<std::string>("output");
        const auto color = parser.get<std::string>("color");
        const bool pnp_enabled = parser.get<bool>("pnp");
        const int log_every = parser.get<int>("log-every");
        if (!parser.check()) {
            parser.printErrors();
            return 1;
        }
        if (camera_options.url.empty()) {
            throw std::invalid_argument(
                "Missing camera address: specify --url=http://<IP>:<PORT>/video");
        }
        if (frame_limit < 0) {
            throw std::invalid_argument("frames must be non-negative");
        }
        if (log_every <= 0) {
            throw std::invalid_argument("log-every must be positive");
        }
        rmvision::components::TraditionalDetectorParams params;
        if (color == "red") {
            params.target_color = rmvision::ArmorColor::red;
        } else if (color == "blue") {
            params.target_color = rmvision::ArmorColor::blue;
        } else if (color != "both") {
            throw std::invalid_argument("Invalid color: use both, red or blue");
        }

        rmvision::components::IpCamFrameSource camera(std::move(camera_options));
        rmvision::components::TraditionalArmorDetector detector(params);
        if (pnp_enabled) {
            std::cout << "PnP test enabled: camera parameters in ipcam_source.cpp; "
                         "model=0.140x0.125m; camera XYZ: right/down/forward\n";
        }
        std::signal(SIGINT, requestStop);
        std::signal(SIGTERM, requestStop);
        camera.open();
        constexpr const char* window = "Aquila IPCam - Q/Esc to quit";
        cv::Mat last_annotated;
        std::uint64_t processed = 0;
        std::uint64_t pnp_solved = 0;
        std::uint64_t pnp_failed = 0;
        std::uint64_t pnp_no_target = 0;
        bool stream_failed = false;
        while (!stop_requested &&
               (frame_limit == 0 || processed < static_cast<std::uint64_t>(frame_limit))) {
            const auto frame = camera.next();
            if (!frame) {
                stream_failed = !stop_requested;
                break;
            }
            const auto start = std::chrono::steady_clock::now();
            const auto result = detector.detect(*frame);
            const double elapsed_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
            std::vector<rmvision::components::IpCamPoseResult> poses;
            if (pnp_enabled) {
                poses = camera.solvePoses(result);
                pnp_no_target += result.detections.empty();
                for (const auto& pose : poses) {
                    pose.valid ? ++pnp_solved : ++pnp_failed;
                }
            }
            ++processed;
            if (show || !output.empty()) {
                last_annotated = annotate(*frame, result, elapsed_ms, camera, poses, pnp_enabled);
            }
            if (processed == 1 || processed % static_cast<std::uint64_t>(log_every) == 0) {
                std::cout << "frame=" << frame->sequence << " size=" << frame->image.cols
                          << 'x' << frame->image.rows << " detections=" << result.detections.size()
                          << " elapsed_ms=" << elapsed_ms << std::endl;
                if (pnp_enabled && poses.empty()) {
                    std::cout << "  PnP: no target\n";
                }
                for (std::size_t i = 0; i < poses.size(); ++i) {
                    const auto& pose = poses[i];
                    std::cout << "  target=" << i;
                    if (pose.valid) {
                        std::cout << " tvec_m=" << pose.tvec.t()
                                  << " distance_m=" << pose.distance_m
                                  << " rvec_rad=" << pose.rvec.t() << '\n';
                    } else {
                        std::cout << " PnP failed: " << pose.error << '\n';
                    }
                }
            }
            if (show) {
                cv::imshow(window, last_annotated);
                const int key = cv::waitKey(1) & 0xff;
                if (key == 27 || key == 'q' || key == 'Q' ||
                    cv::getWindowProperty(window, cv::WND_PROP_VISIBLE) < 1) {
                    break;
                }
            }
        }
        if (show) {
            cv::destroyAllWindows();
        }
        if (!output.empty() && !last_annotated.empty()) {
            const std::filesystem::path output_path(output);
            if (!output_path.parent_path().empty()) {
                std::filesystem::create_directories(output_path.parent_path());
            }
            if (!cv::imwrite(output, last_annotated)) {
                throw std::runtime_error("Cannot save result: " + output);
            }
            std::cout << "result=" << std::filesystem::absolute(output_path).string() << '\n';
        }
        std::cout << "processed_frames=" << processed << '\n';
        if (pnp_enabled) {
            std::cout << "pnp_solved=" << pnp_solved << " pnp_failed=" << pnp_failed
                      << " pnp_no_target=" << pnp_no_target << '\n';
        }
        if (stream_failed) {
            throw std::runtime_error("IPCam stream ended or timed out; check connection and restart");
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "IPCam failed: " << error.what() << '\n';
        return 1;
    }
}
