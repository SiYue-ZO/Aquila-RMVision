#pragma once

#include "components/camera/ipcam_source.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <array>
#include <vector>

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void testInvalidOptions() {
    for (const auto& options : {rmvision::components::IpCamOptions{"", 5000},
                                rmvision::components::IpCamOptions{"unused", 0},
                                rmvision::components::IpCamOptions{"unused", -1}}) {
        bool rejected = false;
        try {
            rmvision::components::IpCamFrameSource camera(options);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        require(rejected, "Invalid camera options must be rejected");
    }
}

void testReplay(const std::filesystem::path& path) {
    std::filesystem::create_directories(path.parent_path());
    // 使用真实 MJPEG 编解码链路验证采集，不依赖网络、GUI 或提交视频素材。
    cv::VideoWriter writer(path.string(), cv::CAP_FFMPEG,
                           cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, {64, 48});
    require(writer.isOpened(), "Cannot create MJPEG fixture; check FFmpeg support");
    for (int i = 0; i < 3; ++i) {
        writer.write(cv::Mat(48, 64, CV_8UC3, cv::Scalar(20 + i * 60, 30, 40)));
    }
    writer.release();

    rmvision::components::IpCamFrameSource camera({path.string(), 1000});
    bool rejected = false;
    try {
        camera.next();
    } catch (const std::logic_error&) {
        rejected = true;
    }
    require(rejected, "Reading an unopened camera must fail");
    camera.open();
    rmvision::ImageFrame first;
    auto previous_time = std::chrono::steady_clock::now();
    for (std::uint64_t i = 0; i < 3; ++i) {
        const auto frame = camera.next();
        require(frame.has_value(), "Replay ended before all frames were decoded");
        require(frame->sequence == i, "Incorrect source sequence");
        require(frame->capture_time >= previous_time &&
                    frame->capture_time <= std::chrono::steady_clock::now(),
                "Capture timestamp must reflect local frame receipt");
        require(frame->image.type() == CV_8UC3 && frame->image.size() == cv::Size(64, 48),
                "Detector requires BGR pixels with original dimensions");
        require(cv::norm(cv::mean(frame->image) - cv::Scalar(20 + i * 60, 30, 40)) < 10,
                "MJPEG frames decoded in the wrong order or color format");
        if (i == 0) {
            first = *frame;
        }
        previous_time = frame->capture_time;
    }
    require(cv::mean(first.image)[0] < 30, "Later reads must not overwrite earlier frames");
    require(!camera.next(), "End of stream must return nullopt");
    camera.open();
    const auto restarted = camera.next();
    require(restarted && restarted->sequence == 0, "Reopening must reset the sequence");
}

const std::array<double, 9> intrinsics{800, 0, 320, 0, 800, 240, 0, 0, 1};
const std::vector<double> distortion(5, 0.0);
const std::vector<cv::Point3f> object_points{
    {-0.07F, 0.0625F, 0}, {0.07F, 0.0625F, 0},
    {0.07F, -0.0625F, 0}, {-0.07F, -0.0625F, 0}};

std::vector<cv::Point2f> projectArmor(double x) {
    std::vector<cv::Point2f> points;
    // 装甲板 +Y 朝上，相机 +Y 朝下，绕 X 旋转 pi 后正对相机。
    cv::projectPoints(object_points, cv::Vec3d(CV_PI, 0, 0), cv::Vec3d(x, 0, 1),
                      cv::Mat(intrinsics).reshape(1, 3), distortion, points);
    return points;
}

void drawArmor(cv::Mat& image, double x) {
    const auto points = projectArmor(x);
    for (const int side : {0, 1}) {
        const auto top = points[side];
        const auto bottom = points[3 - side];
        cv::rectangle(image, cv::Point(cvRound(top.x) - 3, cvRound(top.y)),
                      cv::Point(cvRound(bottom.x) + 3, cvRound(bottom.y)),
                      {0, 255, 255}, cv::FILLED);
    }
}

void writeFixture(const std::filesystem::path& directory) {
    std::filesystem::create_directories(directory);
    cv::VideoWriter writer((directory / "pnp_fixture.avi").string(), cv::CAP_FFMPEG,
                           cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, {640, 480});
    require(writer.isOpened(), "Cannot create PnP replay video");
    // 依次覆盖无目标、单目标、双目标，检查实时入口不会对空 detections 调用 back()。
    for (int count = 0; count <= 2; ++count) {
        cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
        if (count == 1) {
            drawArmor(image, 0.0);
        } else if (count == 2) {
            drawArmor(image, -0.15);
            drawArmor(image, 0.15);
        }
        writer.write(image);
    }
}

void testPnP(const std::filesystem::path& directory) {
    writeFixture(directory);
    rmvision::components::IpCamFrameSource camera({(directory / "pnp_fixture.avi").string(), 1000});
    camera.open();
    require(camera.next().has_value(), "Cannot read PnP replay frame");
    rmvision::DetectionFrame detections;
    require(camera.solvePoses(detections).empty(), "No target must skip PnP");
    for (const double x : {-0.15, 0.15}) {
        const auto points = projectArmor(x);
        rmvision::Detection detection;
        for (std::size_t i = 0; i < points.size(); ++i) {
            detection.corners[i] = points[i];
        }
        detections.detections.push_back(detection);
    }
    const auto poses = camera.solvePoses(detections);
    require(poses.size() == 2, "Both targets must be solved");
    for (std::size_t i = 0; i < poses.size(); ++i) {
        const auto& pose = poses[i];
        require(pose.valid, "Known pose could not be solved");
        const cv::Mat expected = (cv::Mat_<double>(3, 1) << (i == 0 ? -0.15 : 0.15), 0, 1);
        require(cv::norm(pose.tvec - expected) < 1e-4, "Recovered XYZ differs from known pose");
        std::vector<cv::Point2f> reprojected;
        cv::projectPoints(object_points, pose.rvec, pose.tvec,
                          cv::Mat(intrinsics).reshape(1, 3), distortion, reprojected);
        require(cv::norm(projectArmor(i == 0 ? -0.15 : 0.15), reprojected) < 0.01,
                "Recovered rotation has reprojection error");
    }
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        require(argc == 2, "Usage: ipcam_source_test <generated-video-path>");
        testInvalidOptions();
        testReplay(argv[1]);
        testPnP(std::filesystem::path(argv[1]).parent_path());
        std::cout << "IPCam source regression passed: MJPEG, metadata, EOF, reopen and PnP\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "IPCam source regression failed: " << error.what() << '\n';
        return 1;
    }
}
