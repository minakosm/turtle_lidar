cmake_minimum_required(VERSION 3.10)
project(lidarPipeline VERSION 0.92)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS NO)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-fopenmp")
set(CMAKE_CXX_FLAGS_DEBUG "-Wall -Wextra -g -fopenmp")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fopenmp")

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(jsoncpp REQUIRED)
find_package(Threads REQUIRED)
#find_package(Boost REQUIRED COMPONENTS property_tree)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/PointcloudProcessing
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/LidarDriver
                    ${CMAKE_CURRENT_SOURCE_DIR}/modules/ouster_driver/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/modules/hclust-cpp
                    ${CMAKE_CURRENT_SOURCE_DIR}/modules/dbscan
                    ${CMAKE_CURRENT_SOURCE_DIR}/modules/gaussian-naive-bayes-classifier)

# Ouster client library section
add_library(ouster_client SHARED modules/ouster_driver/src/os1.cpp 
                                 modules/ouster_driver/src/os1_util.cpp)
target_link_libraries(ouster_client jsoncpp_lib)
target_include_directories(ouster_client SYSTEM PRIVATE ${jsoncpp_INCLUDE_DIRS})

# FastCluster (Hierarchical Clustering library for C++)
add_library(FastCluster SHARED modules/hclust-cpp/fastcluster.cpp)

# DBSCAN (DBSCAN Clustering library for C++)
add_library(DBSCAN SHARED modules/dbscan/dbscan.cpp)

# Gaussian Naive Bayes Classifier library for C++
add_library(GNBC SHARED modules/gaussian-naive-bayes-classifier/bayes.cpp)

# Settings (INI file reader)
add_library(Settings SHARED src/settings.cpp)

# Main library used for pointcloud processing (Input: Pointcloud -> Output: Cone positions)
add_library(PointcloudProcessing SHARED src/PointcloudProcessing.cpp)
target_link_libraries(PointcloudProcessing Eigen3::Eigen)
target_link_libraries(PointcloudProcessing DBSCAN)
target_link_libraries(PointcloudProcessing FastCluster)
target_link_libraries(PointcloudProcessing Settings)
target_link_libraries(PointcloudProcessing GNBC)

# Lidar driver library section
add_library(LidarDriver SHARED src/LidarDriver.cpp)
target_link_libraries(LidarDriver ouster_client)
target_link_libraries(LidarDriver Threads::Threads)
target_link_libraries(LidarDriver PointcloudProcessing)

# Test executable #1
add_executable(testExample src/testExample.cpp)
target_link_libraries(testExample PointcloudProcessing)

# Test executable #2
add_executable(testExample2 src/testExample2.cpp)
target_link_libraries(testExample2 PointcloudProcessing)

# Test executable LidarDriver
add_executable(testLidar src/testLidarDriver.cpp)
target_link_libraries(testLidar LidarDriver)
