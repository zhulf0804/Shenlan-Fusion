cmake_minimum_required(VERSION 3.12)

find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(Eigen3 REQUIRED)

find_package(PCL 1.8 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})

add_library(gicp src/gicp/fast_gicp.cpp
                 src/gicp/fast_gicp_st.cpp 
                 src/gicp/lsq_registration.cpp)

add_executable(lidar2lidar src/lidar2lidar.cpp src/calib_icp.cpp)

target_link_libraries(lidar2lidar Eigen3::Eigen ${PCL_LIBRARIES} gicp)
target_compile_features(lidar2lidar PRIVATE cxx_std_14)

add_executable(pcd_convert src/convert.cpp)
target_link_libraries(pcd_convert Eigen3::Eigen ${PCL_LIBRARIES})
