cmake_minimum_required(VERSION 2.8.3)
project(sm_common)


find_package(catkin_simple REQUIRED)
catkin_simple()

find_package(Boost REQUIRED COMPONENTS system)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x -D__STRICT_ANSI__")

include_directories(${Boost_INCLUDE_DIRS})


cs_add_library(${PROJECT_NAME}
  src/hash_id.cpp
  src/progress_info.cpp
)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

#############
## Testing ##
#############
# Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

catkin_add_gtest(${PROJECT_NAME}-test
  test/test_main.cpp
  test/maths.cpp
  test/serialization_macros.cpp
  test/numerical_comparisons.cpp
  test/hash_id.cpp
)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} pthread)

cs_install()
cs_export(CFG_EXTRAS export_flags.cmake)
