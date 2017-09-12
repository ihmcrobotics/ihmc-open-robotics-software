cmake_minimum_required(VERSION 2.8.3)
project(image_encoding_converter)

find_package(catkin REQUIRED COMPONENTS
	image_transport
	roscpp
	sensor_msgs
)

catkin_package()

include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable(image_encoding_converter src/EncodingConverter.cpp)
target_link_libraries(image_encoding_converter ${catkin_LIBRARIES})