cmake_minimum_required(VERSION 3.8)
project(turtle_lidar)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS NO)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(ament_index_cpp)
find_package(turtle_common REQUIRED)
find_package(turtle_interfaces REQUIRED)
#find_package(HDF5 1.8.10 REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/OusterDriver
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/PointcloudProcessing
                    ${CMAKE_CURRENT_SOURCE_DIR}/modules/ouster_client/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/OusterOffline)
                    #${HDF5_INCLUDE_DIRS})

#Ouster client library section
add_library(ouster_client SHARED modules/ouster_client/src/client.cpp 
                                 modules/ouster_client/src/types.cpp
                                 modules/ouster_client/src/netcompat.cpp
                                 modules/ouster_client/src/lidar_scan.cpp)
target_link_libraries(ouster_client PUBLIC jsoncpp_lib Eigen3::Eigen)
target_include_directories(ouster_client SYSTEM PRIVATE ${jsoncpp_INCLUDE_DIRS})

add_library(LidarSettings SHARED src/settings.cpp)

add_library(PointcloudProcessing SHARED src/pointcloudProcessing.cpp)
target_link_libraries(PointcloudProcessing Eigen3::Eigen)
#target_link_libraries(PointcloudProcessing ${HDF5_LIBRARIES})
target_link_libraries(PointcloudProcessing LidarSettings)

add_library(OusterOffline SHARED src/OusterOffline.cpp)
target_link_libraries(OusterOffline PointcloudProcessing)
ament_target_dependencies(OusterOffline rclcpp std_msgs sensor_msgs ament_index_cpp)


add_library(OusterDriver SHARED src/OusterDriver.cpp)
target_link_libraries(OusterDriver ouster_client)
target_link_libraries(OusterDriver Threads::Threads)
target_link_libraries(OusterDriver PointcloudProcessing)
ament_target_dependencies(OusterDriver rclcpp std_msgs sensor_msgs ament_index_cpp turtle_interfaces)

add_executable(ouster_driver src/main_ouster.cpp)
target_link_libraries(ouster_driver OusterDriver)
ament_target_dependencies(ouster_driver rclcpp)

add_executable(ouster_offline src/main_ouster_offline.cpp)
target_link_libraries(ouster_offline OusterOffline)
ament_target_dependencies(ouster_offline rclcpp)



install(TARGETS 
        ouster_driver
        ouster_offline
        DESTINATION lib/${PROJECT_NAME})

install(
    TARGETS OusterDriver OusterOffline ouster_client PointcloudProcessing LidarSettings
    EXPORT export_turtle_lidar_libraries
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(FILES 
        lidarConfig.ini 
        DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
