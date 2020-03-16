cmake_minimum_required(VERSION 3.10)
project(lidarPipeline VERSION 0.1)

#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED YES)
#set(CMAKE_CXX_EXTENSIONS NO)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra -fopenmp")
set(CMAKE_CXX_FLAGS_DEBUG "-Wall -Wextra -g -fopenmp")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fopenmp -ffast-math")

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
#find_package(Boost REQUIRED COMPONENTS property_tree)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_library(Utils SHARED src/utils.cpp)
target_link_libraries(Utils Eigen3::Eigen)

add_library(ReadINI SHARED src/readINI.cpp)

add_library(GroundRemoval SHARED src/groundRemoval.cpp)
target_link_libraries(GroundRemoval Eigen3::Eigen)
target_link_libraries(GroundRemoval Utils)

add_executable(TestExample src/testExample.cpp)
target_link_libraries(TestExample GroundRemoval)
target_link_libraries(TestExample ReadINI)
