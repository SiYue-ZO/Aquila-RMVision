#include "runtime/application.hpp"

#include <exception>
#include <iostream>
#include <opencv2/core/utility.hpp>

int main(int argc, char* argv[]) {
    const char* keys = "{help h ? | | Show usage}"
                       "{@image | tests/media/2color.png | Input image path}"
                       "{@output | build/2color_detected.png | Annotated image path}"
                       "{color | | Override armor color: both, red or blue}"
                       "{config | | Traditional detector YAML config}"
                       "{show | false | Display the result and wait for a key}";
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about("Aquila RMVision - traditional OpenCV armor detector");
    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    rmvision::runtime::ApplicationOptions options;
    options.image_path = parser.get<std::string>(0);
    options.output_path = parser.get<std::string>(1);
    options.show_window = parser.get<bool>("show");
    options.detector_config = parser.get<std::string>("config");
    const auto color = parser.get<std::string>("color");
    options.override_target_color = !color.empty();
    if (!parser.check()) {
        parser.printErrors();
        return 1;
    }
    if (color == "red") {
        options.target_color = rmvision::ArmorColor::red;
    } else if (color == "blue") {
        options.target_color = rmvision::ArmorColor::blue;
    } else if (!color.empty() && color != "both") {
        std::cerr << "Invalid color: use both, red or blue\n";
        return 1;
    }

    try {
        return rmvision::runtime::Application(std::move(options)).run();
    } catch (const std::exception& error) {
        std::cerr << "Detection failed: " << error.what() << '\n';
        return 1;
    }
}
