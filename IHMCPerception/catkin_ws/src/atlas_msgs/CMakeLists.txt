cmake_minimum_required(VERSION 2.8.3)
project(atlas_msgs)

if ($ENV{ROS_DISTRO} STREQUAL "fuerte")
  # VERY IMPORTANT: For Fuerte, tell CMake not to strip off the RPATH from libs and executables during installation.
  # This is needed to resolve dependencies on ROS libraries that live in various places deep in /opt/ros/fuerte/stacks.
  # Starting with Groovy, we can allow CMake to strip RPATH (its default behavior) because we use only catkinized
  # packages from ROS.  Those packages' libraries are installed to /opt/ros/<distro>/lib, which we can assume will
  # be in LD_LIBRARY_PATH at run-time.
  set (CMAKE_INSTALL_RPATH_USE_LINK_PATH 1)
endif()

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  actionlib_msgs
  osrf_msgs
#  sandia_hand_msgs
  control_msgs
  message_generation
  )

add_message_files(
  DIRECTORY msg
  FILES
  AtlasBehaviorFeedback.msg
  AtlasBehaviorManipulateFeedback.msg
  AtlasBehaviorManipulateParams.msg
  AtlasBehaviorPelvisServoParams.msg
  AtlasBehaviorStandFeedback.msg
  AtlasBehaviorStandParams.msg
  AtlasBehaviorStepData.msg
  AtlasBehaviorStepFeedback.msg
  AtlasBehaviorStepParams.msg
  AtlasBehaviorWalkFeedback.msg
  AtlasBehaviorWalkParams.msg
  AtlasCommand.msg
  AtlasPositionData.msg
  AtlasSimInterfaceCommand.msg
  AtlasSimInterfaceState.msg
  AtlasState.msg
  ControllerStatistics.msg
  ForceTorqueSensors.msg
  SModelRobotInput.msg
  SModelRobotOutput.msg
  SynchronizationStatistics.msg
  Test.msg
  VRCScore.msg
  )

add_service_files(DIRECTORY srv FILES
  AtlasFilters.srv
  GetJointDamping.srv
  ResetControls.srv
  SetJointDamping.srv
  )

#add_action_files(DIRECTORY action FILES
#  WalkDemo.action
# )

generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  actionlib_msgs
  osrf_msgs
#  sandia_hand_msgs
  control_msgs
  )

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  actionlib_msgs
  osrf_msgs
#  sandia_hand_msgs
  message_generation
  )
