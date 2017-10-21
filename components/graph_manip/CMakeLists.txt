cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(graph_manip C CXX)

find_package(PCL 1.7 REQUIRED)
find_package(Qt5Widgets)

include_directories(${PCL_INCLUDE_DIRS} ${common_SOURCE_DIR}/include ${face_segmentation_SOURCE_DIR}/include)
link_directories(${PCL_LIBRARY_DIRS} ${common_BUILD_DIR})
add_definitions(${PCL_DEFINITIONS})

add_executable (graph_manip graph_manip.cpp ${common_SOURCE_DIR}/include/common.h ${common_SOURCE_DIR}/include/parse.h ${face_segmentation_SOURCE_DIR}/include/face_segmentation.hpp)
target_link_libraries (graph_manip ${PCL_LIBRARIES} ${Boost_LIBRARIES} common)
