# 临时 IPCam 相机

程序不保存默认相机地址，每次测试通过 `--url=IP:端口` 指定，例如 `--url=192.168.1.100:8080`。
程序自动补全为 `http://IP:端口/video`；也支持完整视频 URL 和本地视频回放。
电脑和手机需能通过局域网互通，手机需保持相机服务开启。

## 构建与运行

需要带 FFmpeg 视频采集后端的 OpenCV；沿用项目现有依赖，无需手机相机专用 SDK。
在仓库根目录运行：

```bash
cmake --preset debug
cmake --build --preset debug

# 手动填写本次相机地址，实时检测并预览；Q、Esc、关闭窗口或 Ctrl+C 退出
./build/debug/bin/rmvision_ipcam --url="<IP>:<端口>"

# 只检测蓝色装甲板
./build/debug/bin/rmvision_ipcam --url="<IP>:<端口>" --color=blue

# 无桌面环境验证：读取并检测 30 帧，保存最后一帧标注
./build/debug/bin/rmvision_ipcam --url="<IP>:<端口>" --show=false --frames=30 --output=build/ipcam_last.jpg

# 自定义相机 IP:端口和连接/读取超时（毫秒）
./build/debug/bin/rmvision_ipcam --url="<IP>:<端口>" --timeout=5000
```

未提供 `--url` 或地址为空时，程序提示填写地址并退出，不尝试连接相机。
`--frames=0` 默认持续采集；`--output` 默认不保存，指定后退出时保存最后一个已处理帧。
OpenCV 命令行参数请使用 `--名称=值` 形式，关闭窗口必须写 `--show=false`。
每 30 帧输出一次检测数量和检测调用耗时；耗时不含网络传输、解码和绘制。
断流或读帧超时会输出错误并返回非零退出码，需检查服务后重新启动程序。
连接和读帧默认超时各为 5 秒；Ctrl+C 会在当前采集或处理调用返回后退出。

## PnP 实时测试

PnP 测试直接写在原有 `src/components/camera/ipcam_source.hpp/.cpp` 中，
不需要新增配置文件或独立测试程序。通过 `--pnp=true` 开启：

```bash
./build/debug/bin/rmvision_ipcam \
  --url="<IP>:<端口>" --pnp=true --log-every=1
```

相机参数在 `ipcam_source.cpp` 顶部集中修改：

- `kCalibrationWidth` / `kCalibrationHeight`：相机标定时的图像分辨率。
- `kCameraMatrix`：行优先的 3×3 内参 `[fx, 0, cx; 0, fy, cy; 0, 0, 1]`。
- `kDistCoeffs`：5 个畸变系数 `[k1, k2, p1, p2, k3]`。

当前默认值是合成测试用的 640×480、焦距 800 像素和零畸变，只用于跑通流程；
真实测距请替换为手机相机实测标定值，然后重新构建。内参会按视频帧尺寸缩放，
这仅适用于同一视场的图像缩放；裁剪、数字变焦或切换镜头后应重新标定。

每个目标分别调用你写的 `Armor_PnPsolver`。画面显示 XYZ、距离和长度为 5 cm 的
装甲板坐标轴（红 X、绿 Y、蓝 Z），终端输出：

- `tvec_m=[X,Y,Z]`：装甲板中心在相机坐标系中的位置，单位米，X 向右、Y 向下、Z 向前。
- `distance_m`：到相机光心的直线距离；Z 是沿光轴的深度。
- `rvec_rad`：装甲板到相机的 Rodrigues 旋转向量，单位弧度，不是 yaw/pitch/roll。
- `target`：本帧从左到右的目标索引，不是跟踪 ID。

`--log-every=1` 每帧输出，默认每 30 帧输出（首帧也输出）。无目标时跳过求解，
单个目标失败时报告状态并继续。退出时汇总成功目标次数 `pnp_solved`、失败目标次数
`pnp_failed`、无目标帧数 `pnp_no_target`。

当前模型尺寸来自 `detector.hpp` 中的 `Armor_length=0.140`、`Armor_height=0.125`，
三维点对应左右灯条端点，顺序为左上、右上、右下、左下。测距前需核对实物对应尺寸；
目前大小装甲板共用这一模型，尺寸不匹配会造成距离偏差。

执行下文离线测试后，也可使用生成的视频测试（默认合成相机参数下，深度约 1 m）：

```bash
./build/debug/bin/rmvision_ipcam \
  --url=build/debug/tests/pnp_fixture.avi --pnp=true \
  --frames=3 --show=false --log-every=1 --output=build/pnp_last.jpg
```

三帧依次为无目标、单目标、双目标，预期汇总为
`pnp_solved=3 pnp_failed=0 pnp_no_target=1`。

## 实现边界

- `src/components/camera/ipcam_source.hpp/.cpp`：新增 `IpCamFrameSource`，实现现有
  `FrameSource::next()`，通过 OpenCV FFmpeg 采集 BGR 图像；`open()` 显式打开相机，
  `solvePoses()` 调用 PnP 并返回逐目标位姿，`drawPnPAxes()` 绘制坐标轴。
- `src/runtime/ipcam_main.cpp`：独立实时入口，连接帧源、传统检测器和可选 PnP，负责预览与保存。
- 原图片检测入口 `rmvision_demo` 继续可用，检测器参数保持原有默认值。

帧序号从 0 递增；时间戳是本机读到解码帧时的 `steady_clock` 时间，并非手机曝光时间。
临时网络相机会受到手机编码、网络和后端缓冲的影响，当前需自行提供标定结果，未做曝光控制或
端到端延迟补偿。用它检查采集与检测链路后，真实相机可实现相同的 `FrameSource` 接口替换。

## 离线验证

```bash
cmake --preset debug -DRMVISION_BUILD_TESTS=ON
cmake --build --preset debug
ctest --test-dir build/debug --output-on-failure
```

`ipcam_source_regression` 在构建目录生成三帧 MJPEG 视频，通过真实 FFmpeg 解码验证帧内容、
BGR 格式、序号、时间戳、图像内存独立性、流结束与重新打开；`ipcam_app` 使用同一视频验证
完整的无窗口检测与保存流程。测试无需手机和网络，不向仓库提交视频素材。
PnP 数值校验与合成视频生成已合并进原有 `ipcam_source_test.cpp`，使用源码中的默认合成参数
检查已知投影的位姿恢复；`ipcam_pnp_app` 使用同一测试程序生成的视频验证无目标、单目标、
多目标的完整求解、输出计数和标注保存。替换成真实相机参数后，需同步调整测试的合成内参。
也可使用 `--url=/完整路径/本地视频.avi --show=false --frames=3` 回放；帧数请勿超过视频长度，
当前临时相机入口把提前到达文件末尾同样视为断流并返回非零退出码。
