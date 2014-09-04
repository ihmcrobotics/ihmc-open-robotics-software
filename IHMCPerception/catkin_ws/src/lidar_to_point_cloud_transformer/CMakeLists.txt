cmake_minimum_required(VERSION 2.8.3)
project(lidar_to_point_cloud_transformer)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  laser_geometry
  sensor_msgs
  tf
  pcl_ros
)

## System dependencies are found with CMake's conventions
find_package(Eigen REQUIRED)
find_package(Kindr REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
#  LIBRARIES elevation_mapping
    CATKIN_DEPENDS sensor_msgs tf laser_geometry
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Kindr_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(
  ${PROJECT_NAME}
  src/lidar_to_point_cloud_transformer_node.cpp
  src/LidarToPointCloudTransformer.cpp
  src/PointCloudAssembler.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
