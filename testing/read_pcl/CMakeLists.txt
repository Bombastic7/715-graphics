cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(read_pcl)

find_package(PCL 1.8 REQUIRED)
find_package(Qt5Widgets)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (read_pcl read_pcl.cpp)
target_link_libraries (read_pcl ${PCL_LIBRARIES})
