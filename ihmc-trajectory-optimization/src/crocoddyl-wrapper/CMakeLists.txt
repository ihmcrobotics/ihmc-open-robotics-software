cmake_minimum_required(VERSION 3.4)
project(crocoddyl)

add_compile_options(-std=c++17)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/modules/" ${CMAKE_MODULE_PATH})
list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror")

set(CMAKE_BUILD_TYPE Debug)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

set(INSTALL_LIB_DIR "${CMAKE_INSTALL_PREFIX}/lib" CACHE PATH "Installation directory for libraries")
set(INSTALL_INC_DIR "${CMAKE_INSTALL_PREFIX}/include" CACHE PATH "Installation directory for headers")

set(CROCODDYL_SOURCES

    cpp/external/crocoddyl_external.h
    cpp/external/crocoddyl_external.cpp
)

add_library(crocoddyl-wrapper SHARED ${CROCODDYL_SOURCES})

target_include_directories(crocoddyl-wrapper PUBLIC
     /usr/include/
     cpp/headers/
     cpp/launcher/
     cpp/external/
)

add_executable(DDPSolver cpp/launcher/ddp_solver.cpp)

target_include_directories(DDPSolver PUBLIC
    /usr/include/
    cpp/launcher/
    cpp/headers/
)


