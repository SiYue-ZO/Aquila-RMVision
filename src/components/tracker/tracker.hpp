#pragma once

#include "utils/messages.hpp"

namespace rmvision::components {

class Tracker {
public:
    virtual ~Tracker() = default;
    virtual TrackState update(const DetectionFrame& detections) = 0;
};

class SimpleTracker final : public Tracker {
public:
    TrackState update(const DetectionFrame& detections) override;
};

}  // namespace rmvision::components
