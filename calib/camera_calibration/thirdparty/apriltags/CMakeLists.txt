cmake_minimum_required(VERSION 2.8.3)
project(apriltags)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
include_directories(${Boost_INCLUDE_DIRS})

find_package(Eigen 3.0 REQUIRED )
include_directories(${EIGEN_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIR})

aux_source_directory(src APRILTAGS_SRC)
include_directories(include)

add_library(apriltags SHARED ${APRILTAGS_SRC})
target_link_libraries(apriltags ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS})
