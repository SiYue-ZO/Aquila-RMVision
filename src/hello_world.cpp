#include <opencv2/core.hpp>
#include <opencv2/core/version.hpp>

#include <iostream>

int main() {
    const cv::Mat sample(2, 2, CV_8UC1, cv::Scalar(255));

    std::cout << "Aquila RMVision: hello, world!\n";
    std::cout << "OpenCV version: " << CV_VERSION << '\n';
    std::cout << "Sample matrix: " << sample.rows << "x" << sample.cols << '\n';
    return 0;
}
