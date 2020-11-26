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
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fopenmp -funroll-loops")

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
#find_package(Boost REQUIRED COMPONENTS property_tree)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/PointcloudProcessing
                    ${CMAKE_CURRENT_SOURCE_DIR}/modules/hclust-cpp
                    ${CMAKE_CURRENT_SOURCE_DIR}/modules/dbscan
                    ${CMAKE_CURRENT_SOURCE_DIR}/modules/gaussian-naive-bayes-classifier)

add_library(FastCluster SHARED modules/hclust-cpp/fastcluster.cpp)

add_library(DBSCAN SHARED modules/dbscan/dbscan.cpp)

add_library(GNBC SHARED modules/gaussian-naive-bayes-classifier/bayes.cpp)

add_library(Settings SHARED src/settings.cpp)

add_library(PointcloudProcessing SHARED src/PointcloudProcessing.cpp)
target_link_libraries(PointcloudProcessing Eigen3::Eigen)
target_link_libraries(PointcloudProcessing DBSCAN)
target_link_libraries(PointcloudProcessing FastCluster)
target_link_libraries(PointcloudProcessing Settings)
target_link_libraries(PointcloudProcessing GNBC)

add_executable(testExample src/testExample.cpp)
target_link_libraries(testExample PointcloudProcessing)

add_executable(testExample2 src/testExample2.cpp)
target_link_libraries(testExample2 PointcloudProcessing)
