cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(common C CXX)

include_directories(${common_SOURCE_DIR}/include)
add_library(${PROJECT_NAME} STATIC parse.cpp)
