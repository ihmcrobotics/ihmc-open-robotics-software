cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 17)

project(external-control CXX)

set(Eigen3_DIR "${CMAKE_INSTALL_PREFIX}/include/eigen3")
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(SOURCES
        ${CONTROL_SOURCES}
        src/main/cpp/external-control.hpp
        src/main/cpp/external-control.cpp
        src/main/cpp/types.hpp)

add_library(external-wrapper SHARED ${SOURCES})
target_include_directories(external-wrapper PUBLIC
        src/main/cpp)

install(TARGETS external-wrapper
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)
install(DIRECTORY src/main/cpp/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include FILES_MATCHING PATTERN "*.hpp")
