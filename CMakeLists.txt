cmake_minimum_required(VERSION 2.8.3)

project(circle_judge)

add_compile_options(-std=c++14 -O2)

find_package(PCL 1.1 REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(EXECUTABLE_OUTPUT_PATH ..)


add_executable(test test.cpp)


