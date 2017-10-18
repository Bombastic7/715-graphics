cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(sebastian_making_noise)

find_package(PCL 1.8 REQUIRED)
find_package(Qt5Widgets)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (noise noise.cpp)
target_link_libraries (noise ${PCL_LIBRARIES})
