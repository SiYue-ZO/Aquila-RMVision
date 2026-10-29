#pragma once

#include "components/detector/detector.hpp"

#include <string>

namespace rmvision::runtime {

// 配置与文件读写位于运行层，检测器本身不访问文件系统。
components::TraditionalDetectorParams loadDetectorParams(const std::string& path);
void saveDetectorParams(const std::string& path,
                        const components::TraditionalDetectorParams& params);

} // namespace rmvision::runtime
