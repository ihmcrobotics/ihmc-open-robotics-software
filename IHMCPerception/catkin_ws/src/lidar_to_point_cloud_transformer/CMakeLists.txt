cmake_minimum_required(VERSION 2.8.3)
project(lidar_to_point_cloud_transformer)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  laser_geometry
  sensor_msgs
  tf
  pcl_ros
  moveit_ros_perception
  moveit_ros_planning
  geometric_shapes
  tf_conversions
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
    CATKIN_DEPENDS sensor_msgs tf laser_geometry moveit_ros_perception geometric_shapes moveit_ros_planning tf_conversions
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
  ${moveit_ros_perception_INCLUDE_DIRS}
  ${geometric_shapes_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(
  ${PROJECT_NAME}
  src/lidar_to_point_cloud_transformer_node.cpp
  src/LidarToPointCloudTransformer.cpp
  src/PointCloudAssembler.cpp
  src/RobotSelfFilter.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)



# Add gtest based cpp test target and link libraries
find_package(rostest REQUIRED)
add_rostest_gtest(${PROJECT_NAME}-test test/sphere.test test/RobotSelfFilter_test.cpp src/RobotSelfFilter.cpp)
target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
