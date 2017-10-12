cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(region_growing)

find_package(PCL 1.7 REQUIRED)
find_package(Qt5Widgets)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (region_growing region_growing.cpp ${common_SOURCE_DIR}/include/common.h)
target_link_libraries (region_growing ${PCL_LIBRARIES})
