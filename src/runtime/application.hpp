#pragma once

namespace rmvision::runtime {

class Application {
public:
    explicit Application(int frame_count) : frame_count_(frame_count) {}

    int run();

private:
    int frame_count_{};
};

}  // namespace rmvision::runtime
