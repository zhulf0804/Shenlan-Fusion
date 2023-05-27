cmake_minimum_required(VERSION 3.10)

set(CMAKE_BUILD_TYPE release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen 3.0 REQUIRED)

add_subdirectory(thirdparty/apriltags)

include_directories(
        ${Boost_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        include
        thirdparty/apriltags/include)
 
add_executable(camera_calib
        src/IntrinsicCalib.cpp
        src/CameraCalibration.cpp
        src/CostFunctionFactory.cpp
        src/PinholeCamera.cpp
        src/Chessboard.cpp
        src/Transform.cpp
 )

target_link_libraries(camera_calib PRIVATE
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS}
        ${CERES_LIBRARIES}
        apriltags
)
