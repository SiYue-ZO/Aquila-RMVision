#pragma once

#include "utils/messages.hpp"

#include <optional>

namespace rmvision::components {

class FrameSource {
public:
    virtual ~FrameSource() = default;
    virtual std::optional<ImageFrame> next() = 0;
};

class DemoFrameSource final : public FrameSource {
public:
    explicit DemoFrameSource(int frame_count) : frame_count_(frame_count) {}

    std::optional<ImageFrame> next() override;

private:
    int frame_count_{};
    int next_frame_{};
};

}  // namespace rmvision::components
