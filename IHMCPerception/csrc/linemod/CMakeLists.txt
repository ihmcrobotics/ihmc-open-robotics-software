project(linemod_interface)
cmake_minimum_required(VERSION 3.1)

add_subdirectory(minipcl)

#Eigen
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/minipcl/cmake)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})

#Boost
set(Boost_USE_STATIC_LIBS   ON)
find_package(Boost COMPONENTS filesystem system timer REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

#minipcl headers
INCLUDE_DIRECTORIES(minipcl/recognition/include)
INCLUDE_DIRECTORIES(minipcl/common/include)
INCLUDE_DIRECTORIES(minipcl/filters/include)
INCLUDE_DIRECTORIES(minipcl/features/include)
INCLUDE_DIRECTORIES(minipcl/search/include)
INCLUDE_DIRECTORIES(minipcl/kdtree/include)
INCLUDE_DIRECTORIES(minipcl/octree/include)


#main target
add_library(linemod_interface SHARED linemod_interface.cpp)
target_link_libraries(linemod_interface pcl_recognition  ${Boost_LIBRARIES})
install(TARGETS linemod_interface DESTINATION ${CMAKE_SOURCE_DIR}/../../resources/us/ihmc/ihmcPerception/linemod)

#unit-test
enable_testing() 
add_executable(linemod_test linemod_interface.cpp)
target_link_libraries(linemod_test pcl_recognition  ${Boost_LIBRARIES})
add_test(template_cloud_match linemod_test ${PROJECT_SOURCE_DIR}/test_template.pcd ${PROJECT_SOURCE_DIR}/test_template.sqmmt)
