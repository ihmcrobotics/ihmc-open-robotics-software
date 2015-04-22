project(minipcl)
cmake_minimum_required(VERSION 2.8)


set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})

add_subdirectory(recognition)
add_subdirectory(common) # io/pcd_io is moved to here
#add_subdirectory(filters)

