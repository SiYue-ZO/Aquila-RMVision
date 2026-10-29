# Aquila RMVision

Aquila 战队 RoboMaster 视觉组的装甲板识别与自瞄研究仓库。

当前已实现第一个算法组件：**传统 OpenCV 装甲板检测器**。`rmvision_demo` 从图片读取一帧，完成红蓝灯条分割、几何配对和去重，并保存带四点标注的结果；保留 Hello World 作为构建验证入口。真实相机、数字识别、PnP、弹道解算和串口通信尚未接入。

## 当前环境与要求

- CMake 3.20 或更高版本
- 支持 C++20 的编译器（Ubuntu 22.04/24.04 的 GCC/Clang、Windows MSVC 均可）
- Make 或 Ninja（仓库预设默认使用 Unix Makefiles）
- OpenCV 4（core、imgproc、imgcodecs、highgui、videoio 模块）
- YOLO/ONNX Runtime 暂不参与当前基座构建；硬件和部署平台确认后再锁版本

## 构建流程

CMake 不直接编译 C++。它先读取 `CMakeLists.txt`，检查编译器并生成构建规则；之后再由 Make 等底层工具完成实际编译。仓库使用“源码目录”和“构建目录”分离的方式，生成文件全部放在 `build/` 下，不会混入 `src/`。

### 第一次构建

在仓库根目录执行：

```bash
# Ubuntu/Debian 示例：安装 OpenCV C++ 开发包
sudo apt install libopencv-dev

cmake --preset debug
cmake --build --preset debug
./build/debug/bin/rmvision_hello

# 在仓库根目录运行图片检测，默认输入 tests/media/2color.png
./build/debug/bin/rmvision_demo
```

上述命令分别表示：

1. `cmake --preset debug`：读取 `CMakePresets.json` 中的 `debug` 配置，检查 C++ 编译器和 OpenCV 4，并在 `build/debug/` 生成构建文件。该配置使用 Debug 模式，并打开编译命令导出。
2. `cmake --build --preset debug`：调用底层构建工具，编译 Hello World、检测器库和图片检测应用，链接 OpenCV。
3. `./build/debug/bin/rmvision_hello`：运行 Hello World，输出 OpenCV 版本和示例矩阵尺寸。
4. `./build/debug/bin/rmvision_demo`：检测 `tests/media/2color.png`，输出检测数量和几何评分，结果保存到 `build/2color_detected.png`。

预设默认使用系统的 Unix Makefiles，因此 Linux 上不需要额外安装 Ninja。Windows 或其他生成器可以通过本机的 `CMakeUserPresets.json` 覆盖，不要修改团队共享的预设。

### 日常增量构建

只修改了 `.cpp` 或 `.h` 文件时，通常直接执行：

```bash
cmake --build --preset debug
./build/debug/bin/rmvision_hello
```

只会重新编译受影响的文件。修改 `CMakeLists.txt` 或 `CMakePresets.json` 后，重新执行配置命令：

```bash
cmake --preset debug
cmake --build --preset debug
```

### Release 构建

Release 版本会启用编译器优化，适合后续做性能测试：

```bash
cmake --preset release
cmake --build --preset release
./build/release/bin/rmvision_hello
```

Debug 和 Release 使用不同的构建目录，可以同时保留。所有构建出的可执行文件都位于对应构建目录的 `bin/` 下，不需要在 `CMakeFiles/` 等中间目录里查找。

### 安装到固定目录

如果希望得到一个更干净、适合交给其他人运行的目录，可以执行：

```bash
cmake --install build/debug --prefix install/debug
./install/debug/bin/rmvision_hello
```

`build/debug/bin/` 是构建产物位置，`install/debug/bin/` 是安装产物位置。后续添加配置文件、模型和运行脚本时，也可以统一放入安装目录，形成一个完整的可运行包。

### 不使用预设的写法

如果暂时不熟悉 CMake Preset，也可以把参数直接写在命令行中：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
./build/bin/rmvision_hello
```

其中：

- `-S .` 指定源码目录为当前目录。
- `-B build` 指定构建目录为 `build/`。
- `-DCMAKE_BUILD_TYPE=Debug` 选择 Debug 构建类型。
- `cmake --build build` 执行实际编译。

### 清理并重新构建

通常不需要清理构建目录。如果遇到缓存了旧编译器、旧生成器或旧依赖路径的问题，可以删除对应的构建目录后重新配置：

```bash
python3 -c 'import shutil; shutil.rmtree("build/debug", ignore_errors=True)'
cmake --preset debug
cmake --build --preset debug
```

不要删除源码目录，也不要把 `build/` 提交到 Git。

### 常见问题

- **找不到 C++ 编译器**：安装 GCC/G++ 或 Clang，并重新执行 `cmake --preset debug`。(msvc也行)
- **找不到 OpenCV**：确认已安装 OpenCV 4 的 C++ 开发包；Ubuntu/Debian 可执行 `sudo apt install -y libopencv-dev`，然后重新执行 `cmake --preset debug`。
- **生成器不匹配**：同一个构建目录不能在 Ninja 和 Unix Makefiles 之间直接切换，删除对应的 `build/<配置>/` 后重新配置。
- **程序路径不存在**：先确认 `cmake --build --preset debug` 成功，再运行 `./build/debug/bin/rmvision_hello`。
- **修改后结果没有变化**：确认修改的是当前仓库目录中的文件，并重新执行配置和构建命令。

## 传统装甲板检测器

在仓库根目录执行，输入、输出路径也可以换成绝对路径：

```bash
./build/debug/bin/rmvision_demo tests/media/2color.png build/2color_detected.png

# 仅检测指定颜色；显示窗口是可选的，不加 --show 时无需桌面环境
./build/debug/bin/rmvision_demo tests/media/2color.png build/red_detected.png --color=red
./build/debug/bin/rmvision_demo tests/media/2color.png build/blue_detected.png --color=blue --show
```

`tests/media/2color.png` 的基线结果为 **3 块装甲板：2 红、1 蓝**。图中 `#1/#2/#3` 仅表示本帧输出顺序，四点 `0/1/2/3` 分别表示左上、右上、右下、左下灯条端点。

实现位于 `src/components/detector/`，沿用 `Detector::detect(ImageFrame)` 接口：

1. 使用目标颜色通道亮度与红蓝差分提取灯条，避免鲜红/鲜蓝被灰度权重压暗。
2. 清晰灯条保留拟合方向；宽亮斑借助配对中心连线估计方向，并降低角点质量。
3. 同色灯条按尺度、方向和间距配对，结合板内图案评分，排除跨灯条和重叠配对。
4. 输出原图坐标下的四点、包围框、观测来源、评分种类与角点质量。

默认参数与 `config/traditional.yaml` 一致，也可由运行层读取独立配置，检测组件不执行文件读写、不创建窗口和线程：

```bash
./build/debug/bin/rmvision_demo tests/media/2color.png build/configured.png \
  --config=config/traditional.yaml
```

配置中的 `config_version: 2` 表示亮度阈值作用于目标颜色通道，旧版灰度阈值不可直接套用；显式 `--color` 覆盖配置颜色。未知键、非法类型和越界参数会报错。实际相机的曝光、光照和距离变化后仍需重新验证参数。

`DetectionFrame` 保留输入帧序号和采集时间，包括空检测帧。`target_id=0` 表示数字类别未知，追踪身份使用独立 `track_id`。`confidence` 是几何与图案组成的启发式排序分数，`geometry_score` 单独记录几何一致性；不能与未来 YOLO 的模型分数直接比较。`corner_quality` 标记灯条方向可靠程度，`corners_valid` 仅表示四点几何有效；低质量四点不能作为精确 PnP 的保证。

后续 YOLO 可实现相同 `Detector` 接口，并按需调用 `TraditionalArmorDetector::refineCorners`：只采用附近的清晰灯条，失败保留原始观测，成功也不改写网络类别和模型分数。接口说明和当前限制见 [传统检测器优化说明](docs/traditional-detector.md)。

开启并运行离线回归：

```bash
cmake --preset debug -DRMVISION_BUILD_TESTS=ON
cmake --build --preset debug
ctest --test-dir build/debug --output-on-failure
```

回归覆盖实际红蓝图片、颜色筛选、鲜红/鲜蓝、运动模糊、宽亮斑、图案配对、圆形与横向反光、角点修正成功和失败、配置校验，以及空观测经追踪接口传递的时间信息；保留原有几何和非法输入检查。窗口显示不参与自动测试。

### 视频离线测试

开启 `RMVISION_BUILD_TESTS` 并构建后，在仓库根目录执行：

```bash
./build/debug/bin/detector_video_test \
  'tests/media/旋转装甲板.mp4' build/video_test/traditional_v2 \
  --config=config/traditional.yaml --threads=1
```

该工具顺序处理全部帧，无需显示窗口；不传 `--config` 则使用默认参数。默认使用一个 OpenCV 线程以便对比，不属于常规 CTest 回归。输出目录包含：

- `annotated.mp4`：保留输入分辨率和帧率的检测标注视频。
- `summary.txt`：检出帧数、最长连续未检出区间，以及检测调用的平均和 P95 耗时。
- `frames.csv`、`detections.csv`：逐帧计数、耗时，以及每个候选的颜色、类型、启发式分数、几何分数、角点质量和四点坐标。
- `diagnostics.csv`：轮廓、灯条、配对候选及各阶段拒绝数量。
- `parameters.yml`：本次运行实际使用的完整参数，可直接作为后续实验配置。
- `contact_sheet.jpg`、`first_detected.jpg`、`first_no_detection.jpg`：均匀抽帧预览及代表帧。

旋转视频共 647 帧。旧版检测器有结果 24 帧（3.71%），最长连续空检测 246 帧（4.10 秒）；v2 默认配置有结果 340 帧（52.55%），最长连续空检测 14 帧（0.234 秒）。静态样图仍输出 2 红、1 蓝。该视频用于开发调参，尚不是独立验证集；当前指标只描述候选输出连续性。

“有检出结果的帧占比”不等于准确率或召回率，后两者需要人工真值标注。耗时仅统计检测调用，不包含解码、绘制和编码；虚拟机下的 Debug/Release 运行结果也不能直接代表部署性能。重复执行会覆盖同一输出目录的报告，需要比较实验时应使用不同目录。

## 仓库导航

```text
src/                    组件代码和两个可执行入口：Hello World、图片检测应用
models/                 模型元数据约定，不提交模型二进制
data/raw, data/processed/数据说明与后续数据集
docs/research.md        调研资料、技术决策和待确认问题
docs/development-plan.md分阶段开发计划与验收标准
references/             论文、规则、硬件手册索引
```

## 协作约定

每个可审查的改动使用独立分支和小提交；代码格式采用 `.clang-format`，提交前至少完成本地配置、编译和 Hello World 运行。算法、参数和数据集版本必须能追溯；大模型和原始视频不得直接塞进普通 Git 历史。

## 进一步阅读

- [调研报告](docs/research.md)
- [开发计划](docs/development-plan.md)
- [资料索引](references/README.md)
