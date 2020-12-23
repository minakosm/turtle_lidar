cmake_minimum_required(VERSION 3.5)
project(turtle_lidar VERSION 1.00)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS NO)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-fopenmp")
set(CMAKE_CXX_FLAGS_DEBUG "-Wall -Wextra -g -fopenmp")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fopenmp")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)
find_package(jsoncpp REQUIRED)
#find_package(Boost REQUIRED COMPONENTS property_tree)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/PointcloudProcessing
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/OusterDriver
                    ${CMAKE_CURRENT_SOURCE_DIR}/modules/ouster_client/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/modules/hclust-cpp
                    ${CMAKE_CURRENT_SOURCE_DIR}/modules/dbscan
                    ${CMAKE_CURRENT_SOURCE_DIR}/modules/gaussian-naive-bayes-classifier)

# Ouster client library section
add_library(ouster_client SHARED modules/ouster_client/src/os1.cpp 
                                modules/ouster_client/src/os1_util.cpp)
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
add_library(OusterDriver SHARED src/OusterDriver.cpp)
target_link_libraries(OusterDriver ouster_client)
target_link_libraries(OusterDriver Threads::Threads)
target_link_libraries(OusterDriver PointcloudProcessing)
ament_target_dependencies(OusterDriver rclcpp std_msgs sensor_msgs)

# Test executable #1
add_executable(testExample src/testExample.cpp)
target_link_libraries(testExample PointcloudProcessing)

# Test executable #2
add_executable(testExample2 src/testExample2.cpp)
target_link_libraries(testExample2 PointcloudProcessing)

# ROS2 Ouster Driver Executable
add_executable(ROS2OusterDriver src/main_driver.cpp)
target_link_libraries(ROS2OusterDriver OusterDriver)
ament_target_dependencies(ROS2OusterDriver rclcpp std_msgs sensor_msgs)

install(TARGETS
    ROS2OusterDriver
    DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_ouster_client HAS_LIBRARY_TARGET)
ament_export_libraries(ouster_client)

install(
  TARGETS ouster_client
  EXPORT export_ouster_client
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_FastCluster HAS_LIBRARY_TARGET)
ament_export_libraries(FastCluster)

install(
  TARGETS FastCluster
  EXPORT export_FastCluster
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_DBSCAN HAS_LIBRARY_TARGET)
ament_export_libraries(DBSCAN)

install(
  TARGETS DBSCAN
  EXPORT export_DBSCAN
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_GNBC HAS_LIBRARY_TARGET)
ament_export_libraries(GNBC)

install(
  TARGETS GNBC
  EXPORT export_GNBC
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_Settings HAS_LIBRARY_TARGET)
ament_export_libraries(Settings)

install(
  TARGETS Settings
  EXPORT export_Settings
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_PointcloudProcessing HAS_LIBRARY_TARGET)
ament_export_libraries(PointcloudProcessing)

install(
  TARGETS PointcloudProcessing
  EXPORT export_PointcloudProcessing
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_OusterDriver HAS_LIBRARY_TARGET)
ament_export_libraries(OusterDriver)

install(
  TARGETS OusterDriver
  EXPORT export_OusterDriver
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
