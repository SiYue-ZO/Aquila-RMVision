#pragma once

#include "utils/messages.hpp"

#include <string>
#include <utility>

namespace rmvision::runtime {

struct ApplicationOptions {
    std::string image_path{"tests/media/2color.png"};
    std::string output_path{"build/2color_detected.png"};
    ArmorColor target_color{ArmorColor::unknown};
    bool show_window{false};
};

class Application {
public:
    explicit Application(ApplicationOptions options = {}) : options_(std::move(options)) {}

    int run();

private:
    ApplicationOptions options_;
};

}  // namespace rmvision::runtime
