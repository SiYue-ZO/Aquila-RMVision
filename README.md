# Aquila RMVision

Aquila 战队 RoboMaster 视觉组的装甲板识别与自瞄研究仓库。

当前仓库处于**基座阶段**：只提供现代 C++ / CMake 的可运行 Hello World、工程约定、调研结论和开发计划，尚未实现相机采集、装甲板检测、YOLO 推理、数字识别、PnP、跟踪、弹道解算或串口通信。

## 当前环境与要求

- CMake 3.20 或更高版本
- 支持 C++20 的编译器（Ubuntu 22.04/24.04 的 GCC/Clang、Windows MSVC 均可）
- Make 或 Ninja（仓库预设默认使用 Unix Makefiles）
- OpenCV 4（当前 Hello World 已使用）
- YOLO/ONNX Runtime 暂不参与当前基座构建；硬件和部署平台确认后再锁版本

## 构建流程

CMake 不直接编译 C++。它先读取 `CMakeLists.txt`，检查编译器并生成构建规则；之后再由 Make 等底层工具完成实际编译。仓库使用“源码目录”和“构建目录”分离的方式，生成文件全部放在 `build/` 下，不会混入 `src/`。

### 第一次构建

在仓库根目录执行：

```bash
# Ubuntu/Debian 示例：安装 OpenCV C++ 开发包
sudo apt update
sudo apt install -y libopencv-dev

cmake --preset debug
cmake --build --preset debug
./build/debug/bin/rmvision_hello
```

三条命令分别表示：

1. `cmake --preset debug`：读取 `CMakePresets.json` 中的 `debug` 配置，检查 C++ 编译器和 OpenCV 4，并在 `build/debug/` 生成构建文件。该配置使用 Debug 模式，并打开编译命令导出。
2. `cmake --build --preset debug`：调用底层构建工具，编译 `src/hello_world.cpp`，链接 OpenCV，生成 `build/debug/bin/rmvision_hello`。
3. `./build/debug/bin/rmvision_hello`：运行程序，输出 Hello World、OpenCV 版本和示例矩阵尺寸。

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

- **找不到 C++ 编译器**：安装 GCC/G++ 或 Clang，并重新执行 `cmake --preset debug`。
- **找不到 OpenCV**：确认已安装 OpenCV 4 的 C++ 开发包；Ubuntu/Debian 可执行 `sudo apt install -y libopencv-dev`，然后重新执行 `cmake --preset debug`。
- **生成器不匹配**：同一个构建目录不能在 Ninja 和 Unix Makefiles 之间直接切换，删除对应的 `build/<配置>/` 后重新配置。
- **程序路径不存在**：先确认 `cmake --build --preset debug` 成功，再运行 `./build/debug/bin/rmvision_hello`。
- **修改后结果没有变化**：确认修改的是当前仓库目录中的文件，并重新执行配置和构建命令。

## 仓库导航

```text
src/                    当前唯一程序：Hello World
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
