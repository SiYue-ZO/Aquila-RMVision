#include "runtime/application.hpp"

#include "components/camera/frame_source.hpp"
#include "components/detector/detector.hpp"
#include "components/tracker/tracker.hpp"

#include <iostream>

namespace rmvision::runtime {

int Application::run() {
    components::DemoFrameSource camera(frame_count_);
    components::DemoDetector detector;
    components::SimpleTracker tracker;

    std::cout << "Aquila RMVision minimal component demo\n";

    while (const auto frame = camera.next()) {
        const auto detections = detector.detect(*frame);
        const auto track = tracker.update(detections);

        std::cout << "frame=" << frame->sequence
                  << " detections=" << detections.detections.size();
        if (track.valid) {
            std::cout << " target=" << track.target_id << " center=(" << track.center.x << ','
                      << track.center.y << ") confidence=" << track.confidence;
        }
        std::cout << '\n';
    }

    std::cout << "demo finished\n";
    return 0;
}

}  // namespace rmvision::runtime
