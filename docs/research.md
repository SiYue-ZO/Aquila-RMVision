# Aquila RMVision 仓库搭建

## 结论

先建立一个只依赖 OpenCV 核心模块的 CMake 基座，让所有成员可以在没有相机、GPU 和模型的情况下完成配置、编译、运行和协作。视觉代码进入仓库后，按“输入 → 检测 → 识别 → 几何解算 → 跟踪/预测 → 输出”的数据流拆成库或组件；硬件适配和算法核心隔离。模型训练留在 Python 工具链，部署边界固定为导出的 ONNX 文件，具体后端在目标计算平台实际部署再考虑。

## 资料与得到的事实

| 资料 | 调研要点 | 对本仓库的影响 |
| --- | --- | --- |
| [CMake Tutorial](https://cmake.org/cmake/help/latest/guide/tutorial/index.html) | 现代 CMake 以 target 为中心，配置和构建可以分离，Preset 适合固化常用配置 | 使用目标级编译选项、`CMakePresets.json`，不把 IDE 工程文件作为真相 |
| [OpenCV Linux 安装说明](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html) | OpenCV 在不同平台和安装方式下版本、路径可能不同，CMake `find_package` 是常见集成方式 | 当前基座要求 OpenCV 4 的 `core` 组件；后续按模块增加组件 |
| [Ultralytics ONNX 导出](https://docs.ultralytics.com/modes/export/) | YOLO 模型可导出为 ONNX，并配置输入尺寸、精度、设备等导出参数 | 训练与部署解耦；仓库记录导出参数和模型校验和，不把 `.pt/.onnx` 默认提交 |
| [ONNX Runtime C++](https://onnxruntime.ai/docs/get-started/with-cpp.html) | C++ 部署需要显式处理运行时、执行提供者和模型输入输出 | 先定义推理接口，再为 CPU、CUDA、TensorRT 等后端做适配，避免业务代码绑定单一 SDK |
| [RoboMaster 官方资源入口](https://www.robomaster.com/zh-CN) | 比赛规则、机器人约束和赛季版本由官方发布，不能用社区经验替代 | 在冻结相机、弹丸速度、装甲尺寸和通信协议前，必须锁定当年规则版本 |
| [rm_armor_detection_demo](https://github.com/23YDY/rm_armor_detection_demo) | 社区新人项目采用 CMake、`include/src/config/data/output/docs`，并把视频和产物排除出 Git | 采用相似的易上手目录，但暂不复制其算法实现 |



## 推荐技术边界

### C++ 与构建

基座使用 C++20、CMake 3.20+，开启 `-Wall -Wextra -Wpedantic`。如果目标工控机或嵌入式板卡的编译器无法稳定支持 C++20，应把标准降到 C++17，并在全队统一，而不是混用。依赖通过 CMake target 传递，版本在构建说明和锁定文件中记录。

### 图像与推理

OpenCV 负责 `cv::Mat`、颜色空间、几何运算、相机/视频输入和调试可视化。YOLO 训练、数据增强和评估使用 Python；部署只接收模型文件和明确的预处理/后处理契约。推理接口至少要约定输入布局（BGR/RGB、NCHW）、归一化、letterbox、类别顺序、置信度/NMS 和坐标还原。

OpenCV DNN 上手快、依赖少，适合最初验证；ONNX Runtime 更适合明确控制 CPU/CUDA/TensorRT 执行提供者和性能。不能仅凭文档选择后端，应在目标板上用同一个模型、输入尺寸和数据集比较延迟、帧率、内存及稳定性。

### 运行时数据流

后续建议按以下边界逐步实现：

```text
Camera/Input → FrameSource → ArmorDetector → NumberRecognizer
             → PoseSolver(PnP) → Tracker/Prediction → AimCommand/Telemetry
```

每层通过小接口传递带时间戳的数据结构；相机参数、敌方颜色、模型路径、阈值、串口地址放在 YAML/TOML 等配置中，不硬编码在算法类里。第一版可以单进程、单线程；只有测量证明需要时再引入线程池、队列或 ROS2

## 必须在开发前确认的问题

1. 目标计算平台、操作系统、GPU/推理加速器和允许的编译器版本。
2. 相机型号、分辨率、帧率、曝光控制、内参/畸变标定流程和相机 SDK 许可。
3. 当年赛事规则中的装甲板尺寸、颜色、编号、通信与安全限制。
4. 自瞄输出协议：串口/网络格式、坐标系、时间戳、云台延迟和频率。
5. 数据集采集、标注、分割、版本发布和模型权重存储方案。
6. 是否需要 ROS 2、多进程、录包回放或仿真；这些都应由需求和测量驱动。

本仓库现检测器、训练脚本、推理封装、PnP、跟踪、弹道预测或硬件驱动，也没有把社区仓库代码复制进来。社区项目只能用于了解目录和工程实践；比赛规则以当年 RoboMaster 官方文件为准。
