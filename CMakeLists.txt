cmake_minimum_required(VERSION 3.10)
project(StereoVisualOdometry VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -Wall")

find_package(OpenCV 4.0 REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(vo_core STATIC
    src/feature_tracker.cpp
    src/triangulation.cpp
    src/motion_estimator.cpp
    src/visual_odometry.cpp
)

# Compile the synthetic dashboard test
add_executable(vo_test tests/main.cpp)
target_link_libraries(vo_test vo_core ${OpenCV_LIBS})

# Compile the LIVE WEBCAM test
add_executable(webcam_live tests/webcam_live.cpp)
target_link_libraries(webcam_live vo_core ${OpenCV_LIBS})
