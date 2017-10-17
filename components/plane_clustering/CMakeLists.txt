cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(plane_clustering C CXX)

find_package(PCL 1.7 REQUIRED)
find_package(Qt5Widgets)
find_package(Boost COMPONENTS program_options REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${common_SOURCE_DIR}/include ${face_graph_SOURCE_DIR}/include)
link_directories(${PCL_LIBRARY_DIRS} ${common_BUILD_DIR})
add_definitions(${PCL_DEFINITIONS})

add_executable (plane_clustering plane_clustering.cpp ${common_SOURCE_DIR}/include/common.h ${common_SOURCE_DIR}/include/parse.h ${face_graph_SOURCE_DIR}/include/face_graph.hpp)
target_link_libraries (plane_clustering ${PCL_LIBRARIES} ${Boost_LIBRARIES} common)
