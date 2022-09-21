cmake_minimum_required(VERSION 3.4)
project(factor_graph_node)

add_compile_options(-std=c++17)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/modules/" ${CMAKE_MODULE_PATH})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

find_package(GTSAM REQUIRED)
#find_package(Eigen3 3.3 REQUIRED NO_MODULE)

#include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${GTSAM_INCLUDE_DIR})

set(FG_SOURCES
    factor_graph/factor_graph_handler.cpp
    factor_graph/factor_graph_handler.h
    external/factor_graph_external.cpp
    external/factor_graph_external.h
    launcher/application.cpp
)


add_executable(FactorGraphNodeHeadless ${FG_SOURCES})
target_link_libraries(FactorGraphNodeHeadless
    gtsam
 #   eigen

)
target_include_directories(FactorGraphNodeHeadless PUBLIC
    factor_graph/
    external/
)