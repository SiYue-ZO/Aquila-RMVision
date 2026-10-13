cmake_minimum_required(VERSION 3.20)

project(AquilaRMVision VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(GNUInstallDirs)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

option(RMVISION_BUILD_TESTS "Build tests (reserved for a later milestone)" OFF)

find_package(OpenCV 4 REQUIRED COMPONENTS core)

add_executable(rmvision_hello
    src/hello_world.cpp
)

add_executable(rmvision_demo
    src/runtime/main.cpp
    src/runtime/application.cpp
    src/components/camera/frame_source.cpp
    src/components/detector/detector.cpp
    src/components/tracker/tracker.cpp
)

target_compile_features(rmvision_hello PRIVATE cxx_std_20)
target_include_directories(rmvision_hello PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rmvision_hello PRIVATE ${OpenCV_LIBS})
target_compile_options(rmvision_hello PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

target_compile_features(rmvision_demo PRIVATE cxx_std_20)
target_include_directories(rmvision_demo PRIVATE src ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rmvision_demo PRIVATE ${OpenCV_LIBS})
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
    message(STATUS "Tests are reserved for the first algorithm milestone")
endif()
