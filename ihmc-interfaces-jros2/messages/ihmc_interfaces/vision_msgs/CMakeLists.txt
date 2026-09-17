cmake_minimum_required(VERSION 3.5)
project(vision_msgs)

if(NOT WIN32)
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  msg/BoundingBox2D.msg
  msg/BoundingBox2DArray.msg
  msg/BoundingBox3D.msg
  msg/BoundingBox3DArray.msg
  msg/Classification.msg
  msg/Detection2DArray.msg
  msg/Detection2D.msg
  msg/Detection3DArray.msg
  msg/Detection3D.msg
  msg/LabelInfo.msg
  msg/ObjectHypothesis.msg
  msg/ObjectHypothesisWithPose.msg
  msg/VisionClass.msg
  msg/Point2D.msg
  msg/Pose2D.msg
  msg/VisionInfo.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs
  ADD_LINTER_TESTS)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(vision_msgs_test test/main.cpp)
  add_dependencies(vision_msgs_test ${PROJECT_NAME})
  target_link_libraries(vision_msgs_test
    ${geometry_msgs_TARGETS}
    ${std_msgs_TARGETS}
  )
  # TODO(sloretz) rosidl_generate_interfaces() should make using generated messages in same project simpler
  target_include_directories(vision_msgs_test PUBLIC
    include
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  )
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_package()
