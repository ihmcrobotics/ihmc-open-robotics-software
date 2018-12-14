cmake_minimum_required(VERSION 3.0)

project(NativeCommonOps)

find_package(JNI REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(${JNI_INCLUDE_DIRS})

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -s")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(SOURCE_FILES NativeCommonOps.cpp)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../src/main/resources)

add_library(NativeCommonOps SHARED ${SOURCE_FILES})
target_link_libraries (NativeCommonOps Eigen3::Eigen)

install(TARGETS NativeCommonOps RUNTIME DESTINATION ${PROJECT_SOURCE_DIR}/../src/main/resources LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/../src/main/resources)
