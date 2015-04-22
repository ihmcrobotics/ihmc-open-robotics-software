project(linemod_interface)
cmake_minimum_required(VERSION 2.8)

add_subdirectory(minipcl)

#Eigen
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/minipcl/cmake)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})

#Boost
if(WIN32)
	set(Boost_USE_STATIC_LIBS   ON)
endif(WIN32)
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
	set(CMAKE_CXX_FLAGS "-fPIC" CACHE STRING ""  FORCE)
	set(CMAKE_C_FLAGS  "-fPIC" CACHE STRING "" FORCE)
endif()

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()
find_package(Boost COMPONENTS filesystem system thread date_time chrono iostreams REQUIRED)
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
set_target_properties(linemod_interface PROPERTIES COMPILE_FLAGS "-fPIC")
target_link_libraries(linemod_interface pcl_recognition pcl_common ${Boost_LIBRARIES})
install(TARGETS linemod_interface DESTINATION ${CMAKE_SOURCE_DIR}/../../resources/us/ihmc/ihmcPerception/linemod)

#unit-test
enable_testing() 
add_executable(linemod_test linemod_interface.cpp)
target_link_libraries(linemod_test pcl_recognition pcl_common  ${Boost_LIBRARIES})
add_test(template_cloud_match linemod_test ${PROJECT_SOURCE_DIR}/test_template.pcd ${PROJECT_SOURCE_DIR}/test_template.sqmmt)
