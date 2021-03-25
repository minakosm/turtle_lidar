cmake_minimum_required(VERSION 3.5)
project(turtle_lidar VERSION 0.97)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS NO)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-fopenmp")
set(CMAKE_CXX_FLAGS_DEBUG "-Wall -Wextra -g -fopenmp")
set(CMAKE_CXX_FLAGS_RELEASE "-fopenmp -O3")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(ament_index_cpp)
find_package(turtle_common REQUIRED)
find_package(turtle_interfaces REQUIRED)
#find_package(Boost REQUIRED COMPONENTS property_tree)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/PointcloudProcessing
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/OusterDriver
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/SimDriver
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/OusterOffline
                    ${CMAKE_CURRENT_SOURCE_DIR}/modules/ouster_client/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/modules/hclust-cpp
                    ${CMAKE_CURRENT_SOURCE_DIR}/modules/dbscan
                    ${CMAKE_CURRENT_SOURCE_DIR}/modules/gaussian-naive-bayes-classifier)

# Ouster client library section
add_library(ouster_client SHARED modules/ouster_client/src/client.cpp 
                                 modules/ouster_client/src/types.cpp
                                 modules/ouster_client/src/netcompat.cpp
                                 modules/ouster_client/src/lidar_scan.cpp)
target_link_libraries(ouster_client PUBLIC jsoncpp_lib Eigen3::Eigen)
target_include_directories(ouster_client SYSTEM PRIVATE ${jsoncpp_INCLUDE_DIRS})

# FastCluster (Hierarchical Clustering library for C++)
add_library(FastCluster SHARED modules/hclust-cpp/fastcluster.cpp)

# DBSCAN (DBSCAN Clustering library for C++)
add_library(DBSCAN SHARED modules/dbscan/dbscan.cpp)

# Gaussian Naive Bayes Classifier library for C++
add_library(GNBC SHARED modules/gaussian-naive-bayes-classifier/bayes.cpp)

# LidarSettings (INI file reader)
add_library(LidarSettings SHARED src/settings.cpp)

# Main library used for pointcloud processing (Input: Pointcloud -> Output: Cone positions)
add_library(PointcloudProcessing SHARED src/PointcloudProcessing.cpp)
target_link_libraries(PointcloudProcessing Eigen3::Eigen)
target_link_libraries(PointcloudProcessing DBSCAN)
target_link_libraries(PointcloudProcessing FastCluster)
target_link_libraries(PointcloudProcessing LidarSettings)
target_link_libraries(PointcloudProcessing GNBC)

# Ouster Lidar driver library section
add_library(OusterDriver SHARED src/OusterDriver.cpp)
target_link_libraries(OusterDriver ouster_client)
target_link_libraries(OusterDriver Threads::Threads)
target_link_libraries(OusterDriver PointcloudProcessing)
ament_target_dependencies(OusterDriver rclcpp std_msgs sensor_msgs ament_index_cpp turtle_interfaces)

# Simulation Lidar driver library section
add_library(SimDriver SHARED src/SimDriver.cpp)
target_link_libraries(SimDriver PointcloudProcessing)
ament_target_dependencies(SimDriver rclcpp std_msgs sensor_msgs nav_msgs ament_index_cpp turtle_common)

# Ouster Offline (for rosbags) library section
add_library(OusterOffline SHARED src/OusterOffline.cpp)
target_link_libraries(OusterOffline PointcloudProcessing)
ament_target_dependencies(OusterOffline rclcpp std_msgs sensor_msgs ament_index_cpp)

# # Test executable #1
# add_executable(testExample src/testExample.cpp)
# target_link_libraries(testExample PointcloudProcessing)

# # Test executable #2
# add_executable(testExample2 src/testExample2.cpp)
# target_link_libraries(testExample2 PointcloudProcessing)

# ROS2 Ouster Driver Executable
add_executable(ouster_driver src/main_ouster.cpp)
target_link_libraries(ouster_driver OusterDriver)
ament_target_dependencies(ouster_driver rclcpp)

# ROS2 Simulation Driver Executable
add_executable(sim_driver src/main_sim.cpp)
target_link_libraries(sim_driver SimDriver)
ament_target_dependencies(sim_driver rclcpp)

# ROS2 Ouster Offline Executable (for running Pointcloud Processing on rosbags)
add_executable(ouster_offline src/main_ouster_offline.cpp)
target_link_libraries(ouster_offline OusterOffline)
ament_target_dependencies(ouster_offline rclcpp)

# ROS2 Test PTP Sync
add_executable(testPTP src/testPTP.cpp)
target_link_libraries(testPTP Eigen3::Eigen ouster_client)
ament_target_dependencies(testPTP rclcpp ament_index_cpp)

install(TARGETS
    ouster_driver
    sim_driver
    ouster_offline
    testPTP
    DESTINATION lib/${PROJECT_NAME}
)

install(
    TARGETS SimDriver OusterDriver OusterOffline PointcloudProcessing ouster_client DBSCAN GNBC LidarSettings FastCluster
    EXPORT export_turtle_lidar_libraries
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/lidarConfig.ini 
              ${CMAKE_CURRENT_SOURCE_DIR}/example/simConeTrainDataX.txt 
              ${CMAKE_CURRENT_SOURCE_DIR}/example/simConeTrainDataY.txt
	      ${CMAKE_CURRENT_SOURCE_DIR}/example/ousterConeTrainDataX.txt
	      ${CMAKE_CURRENT_SOURCE_DIR}/example/ousterConeTrainDataY.txt
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
