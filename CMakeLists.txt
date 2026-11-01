cmake_minimum_required(VERSION 3.20)

project(AquilaRMVision VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(GNUInstallDirs)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

option(RMVISION_BUILD_TESTS "Build offline detector regression tests" OFF)

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs highgui videoio calib3d)

# 检测器独立成库，应用和离线测试复用同一份算法。
add_library(rmvision_detector
    src/components/detector/detector.cpp
)
target_compile_features(rmvision_detector PUBLIC cxx_std_20)
target_include_directories(rmvision_detector PUBLIC src ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rmvision_detector PUBLIC opencv_core opencv_imgproc opencv_calib3d)
target_compile_options(rmvision_detector PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

add_executable(rmvision_hello
    src/hello_world.cpp
)

add_executable(rmvision_demo
    src/runtime/main.cpp
    src/runtime/application.cpp
)

# 临时 IPCam 入口单独构建，沿用 FrameSource 和已有检测器。
add_library(rmvision_ipcam_source src/components/camera/ipcam_source.cpp)
target_include_directories(rmvision_ipcam_source PUBLIC src ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rmvision_ipcam_source PUBLIC rmvision_detector opencv_videoio)
target_compile_options(rmvision_ipcam_source PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

add_executable(rmvision_ipcam src/runtime/ipcam_main.cpp)
target_link_libraries(rmvision_ipcam PRIVATE
    rmvision_ipcam_source rmvision_detector opencv_imgcodecs opencv_highgui)
target_compile_options(rmvision_ipcam PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
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
target_link_libraries(rmvision_demo PRIVATE rmvision_detector opencv_imgcodecs opencv_highgui)
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

install(TARGETS rmvision_ipcam
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(RMVISION_BUILD_TESTS)
    enable_testing()
    add_executable(ipcam_source_test tests/ipcam_source_test.cpp)
    target_link_libraries(ipcam_source_test PRIVATE rmvision_ipcam_source)
    target_compile_options(ipcam_source_test PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
        $<$<CXX_COMPILER_ID:MSVC>:/W4>
    )
    set(ipcam_fixture "${CMAKE_CURRENT_BINARY_DIR}/tests/ipcam_fixture.avi")
    add_test(NAME ipcam_source_regression COMMAND ipcam_source_test "${ipcam_fixture}")
    set_tests_properties(ipcam_source_regression PROPERTIES FIXTURES_SETUP ipcam_video)
    add_test(NAME ipcam_app COMMAND rmvision_ipcam
        "--url=${ipcam_fixture}" --frames=3 --show=false
        "--output=${CMAKE_CURRENT_BINARY_DIR}/tests/ipcam_last.jpg")
    set_tests_properties(ipcam_app PROPERTIES FIXTURES_REQUIRED ipcam_video TIMEOUT 15)

    add_test(NAME ipcam_pnp_app COMMAND rmvision_ipcam
        "--url=${CMAKE_CURRENT_BINARY_DIR}/tests/pnp_fixture.avi"
        --pnp=true
        --frames=3 --show=false --log-every=1
        "--output=${CMAKE_CURRENT_BINARY_DIR}/tests/pnp_last.jpg")
    set_tests_properties(ipcam_pnp_app PROPERTIES
        FIXTURES_REQUIRED ipcam_video TIMEOUT 15
        PASS_REGULAR_EXPRESSION "pnp_solved=3 pnp_failed=0 pnp_no_target=1")

    add_executable(detector_test tests/detector_test.cpp)
    target_link_libraries(detector_test PRIVATE rmvision_detector opencv_imgcodecs)
    target_compile_options(detector_test PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
        $<$<CXX_COMPILER_ID:MSVC>:/W4>
    )
    add_test(NAME detector_regression
        COMMAND detector_test "${CMAKE_CURRENT_SOURCE_DIR}/tests/media/2color.png")
    add_test(NAME detector_app
        COMMAND rmvision_demo "${CMAKE_CURRENT_SOURCE_DIR}/tests/media/2color.png"
                "${CMAKE_CURRENT_BINARY_DIR}/tests/2color_detected.png")

    # 视频评估手动执行，避免将本地视频和耗时编码加入常规回归。
    add_executable(detector_video_test tests/detector_video_test.cpp)
    target_link_libraries(detector_video_test PRIVATE
        rmvision_detector opencv_videoio opencv_imgcodecs)
    target_compile_options(detector_video_test PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
        $<$<CXX_COMPILER_ID:MSVC>:/W4>
    )
endif()
