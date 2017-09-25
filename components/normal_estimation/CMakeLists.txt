cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(normal_estimation)

find_package(PCL 1.7 REQUIRED)
find_package(Qt5Widgets)

include_directories(${PCL_INCLUDE_DIRS} ${common_SOURCE_DIR}/include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (normal_estimation normal_estimation.cpp)
target_link_libraries (normal_estimation ${PCL_LIBRARIES})
