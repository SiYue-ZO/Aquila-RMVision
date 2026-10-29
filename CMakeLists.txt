cmake_minimum_required(VERSION 3.20)

project(AquilaRMVision VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(GNUInstallDirs)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

option(RMVISION_BUILD_TESTS "Build offline detector regression tests" OFF)

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs highgui videoio)

# 检测器独立成库，应用和离线测试复用同一份算法。
add_library(rmvision_detector
    src/components/detector/detector.cpp
)
target_compile_features(rmvision_detector PUBLIC cxx_std_20)
target_include_directories(rmvision_detector PUBLIC src ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rmvision_detector PUBLIC opencv_core opencv_imgproc)
target_compile_options(rmvision_detector PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

# 配置和追踪实现独立于检测算法；未来 YOLO 后端可单独增加可选构建目标。
add_library(rmvision_detector_config src/runtime/detector_config.cpp)
target_link_libraries(rmvision_detector_config PUBLIC rmvision_detector)

add_library(rmvision_tracker src/components/tracker/tracker.cpp)
target_include_directories(rmvision_tracker PUBLIC src ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rmvision_tracker PUBLIC opencv_core)
foreach(target rmvision_detector_config rmvision_tracker)
    target_compile_options(${target} PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
        $<$<CXX_COMPILER_ID:MSVC>:/W4>
    )
endforeach()

add_executable(rmvision_hello
    src/hello_world.cpp
)

add_executable(rmvision_demo
    src/runtime/main.cpp
    src/runtime/application.cpp
)

target_compile_features(rmvision_hello PRIVATE cxx_std_20)
target_include_directories(rmvision_hello PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rmvision_hello PRIVATE opencv_core)
target_compile_options(rmvision_hello PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

target_compile_features(rmvision_demo PRIVATE cxx_std_20)
target_include_directories(rmvision_demo PRIVATE src ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rmvision_demo PRIVATE rmvision_detector_config opencv_imgcodecs opencv_highgui)
target_compile_options(rmvision_demo PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

install(TARGETS rmvision_hello
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(TARGETS rmvision_demo
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(RMVISION_BUILD_TESTS)
    enable_testing()
    add_executable(detector_test tests/detector_test.cpp)
    target_link_libraries(detector_test PRIVATE rmvision_detector_config rmvision_tracker opencv_imgcodecs)
    target_compile_options(detector_test PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
        $<$<CXX_COMPILER_ID:MSVC>:/W4>
    )
    add_test(NAME detector_regression
        COMMAND detector_test "${CMAKE_CURRENT_SOURCE_DIR}/tests/media/2color.png"
                "${CMAKE_CURRENT_BINARY_DIR}/tests/detector_contracts")
    add_test(NAME detector_app
        COMMAND rmvision_demo "${CMAKE_CURRENT_SOURCE_DIR}/tests/media/2color.png"
                "${CMAKE_CURRENT_BINARY_DIR}/tests/2color_detected.png")
    add_test(NAME detector_app_config
        COMMAND rmvision_demo "${CMAKE_CURRENT_SOURCE_DIR}/tests/media/2color.png"
                "${CMAKE_CURRENT_BINARY_DIR}/tests/2color_configured.png"
                "--config=${CMAKE_CURRENT_SOURCE_DIR}/config/traditional.yaml")

    # 视频评估手动执行，避免将本地视频和耗时编码加入常规回归。
    add_executable(detector_video_test tests/detector_video_test.cpp)
    target_link_libraries(detector_video_test PRIVATE
        rmvision_detector_config opencv_videoio opencv_imgcodecs)
    target_compile_options(detector_video_test PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
        $<$<CXX_COMPILER_ID:MSVC>:/W4>
    )
endif()
