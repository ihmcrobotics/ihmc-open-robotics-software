cmake_minimum_required(VERSION 2.8.3)
project(osrf_msgs)

find_package(catkin REQUIRED message_generation std_msgs)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  JointCommands.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  message_generation
  std_msgs
)
