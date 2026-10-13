#pragma once

#include "utils/messages.hpp"

namespace rmvision::components {

class Detector {
public:
    virtual ~Detector() = default;
    virtual DetectionFrame detect(const ImageFrame& frame) = 0;
};

class DemoDetector final : public Detector {
public:
    DetectionFrame detect(const ImageFrame& frame) override;
};

}  // namespace rmvision::components
