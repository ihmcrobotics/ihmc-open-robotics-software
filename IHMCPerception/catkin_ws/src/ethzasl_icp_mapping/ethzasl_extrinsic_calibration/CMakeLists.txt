cmake_minimum_required(VERSION 2.8.3)
project(ethzasl_extrinsic_calibration)

find_package(catkin REQUIRED COMPONENTS roscpp rospy tf)

catkin_package(
	CATKIN_DEPENDS roscpp rospy tf
	DEPENDS eigen
)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

add_executable(tf_logger src/tf_logger.cpp)
target_link_libraries(tf_logger ${catkin_LIBRARIES})
add_executable(optimize src/optimize.cpp)
target_link_libraries(tf_logger ${catkin_LIBRARIES})

