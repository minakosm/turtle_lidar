cmake_minimum_required(VERSION 3.10)
project(turtle_lidar_test VERSION 0.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS NO)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-fopenmp")
set(CMAKE_CXX_FLAGS_DEBUG "-Wall -Wextra -g -fopenmp")
set(CMAKE_CXX_FLAGS_RELEASE "-fopenmp -O3 -ffast-math")

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(jsoncpp REQUIRED)
# find_package(ament_index_cpp)
# find_package(turtle_common REQUIRED)
# find_package(turtle_interfaces REQUIRED)
#find_package(Boost REQUIRED COMPONENTS property_tree)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include
                    ${CMAKE_CURRENT_SOURCE_DIR}/../include/PointcloudProcessing
                    ${CMAKE_CURRENT_SOURCE_DIR}/../include/OusterDriver
                    ${CMAKE_CURRENT_SOURCE_DIR}/../include/SimDriver
                    ${CMAKE_CURRENT_SOURCE_DIR}/../include/OusterOffline
                    ${CMAKE_CURRENT_SOURCE_DIR}/../modules/ouster_client/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/../modules/hclust-cpp
                    ${CMAKE_CURRENT_SOURCE_DIR}/../modules/dbscan
                    ${CMAKE_CURRENT_SOURCE_DIR}/../modules/parallel_dbscan
                    ${CMAKE_CURRENT_SOURCE_DIR}/../modules/hpdbscan/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/../modules/gaussian-naive-bayes-classifier)

# Ouster client library section
# add_library(ouster_client SHARED modules/ouster_client/src/client.cpp 
#                                  modules/ouster_client/src/types.cpp
#                                  modules/ouster_client/src/netcompat.cpp
#                                  modules/ouster_client/src/lidar_scan.cpp)
# target_link_libraries(ouster_client PUBLIC jsoncpp_lib Eigen3::Eigen)
# target_include_directories(ouster_client SYSTEM PRIVATE ${jsoncpp_INCLUDE_DIRS})

# FastCluster (Hierarchical Clustering library for C++)
add_library(FastCluster SHARED ../modules/hclust-cpp/fastcluster.cpp)

# DBSCAN (DBSCAN Clustering library for C++)
add_library(DBSCAN SHARED ../modules/dbscan/dbscan.cpp)

# HPDBSCAN
add_library(hpdbscan SHARED ../modules/hpdbscan/src/hpdbscan.cpp)
find_package(OpenMP 4.0 REQUIRED)
target_link_libraries(hpdbscan PRIVATE ${OpenMP_CXX_FLAGS})
target_compile_options(hpdbscan PRIVATE ${OpenMP_CXX_FLAGS})
find_package(HDF5 1.8.10 REQUIRED)
include_directories("${HDF5_INCLUDE_DIRS}")
target_link_libraries(hpdbscan PRIVATE "${HDF5_LIBRARIES}")

# Gaussian Naive Bayes Classifier library for C++
add_library(GNBC SHARED ../modules/gaussian-naive-bayes-classifier/bayes.cpp)

# LidarSettings (INI file reader)
add_library(LidarSettings SHARED ../src/settings.cpp)

# Main library used for pointcloud processing (Input: Pointcloud -> Output: Cone positions)
add_library(PointcloudProcessing SHARED ../src/PointcloudProcessing.cpp)
target_link_libraries(PointcloudProcessing Eigen3::Eigen)
target_link_libraries(PointcloudProcessing DBSCAN)
target_link_libraries(PointcloudProcessing hpdbscan)
target_link_libraries(PointcloudProcessing "${HDF5_LIBRARIES}")
target_link_libraries(PointcloudProcessing FastCluster)
target_link_libraries(PointcloudProcessing LidarSettings)
target_link_libraries(PointcloudProcessing GNBC)

# Test executable #1
add_executable(test test.cpp)
target_link_libraries(test PointcloudProcessing)
