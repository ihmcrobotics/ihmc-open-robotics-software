cmake_minimum_required(VERSION 3.4)
project(mpc)

add_compile_options(-std=c++17)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/modules/" ${CMAKE_MODULE_PATH})
list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror")

set(CMAKE_BUILD_TYPE Debug)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

set(INSTALL_LIB_DIR "${CMAKE_INSTALL_PREFIX}/lib" CACHE PATH "Installation directory for libraries")
set(INSTALL_INC_DIR "${CMAKE_INSTALL_PREFIX}/include" CACHE PATH "Installation directory for headers")

set(MPC_SOURCES

    external/mpc_external.h
    external/mpc_external.cpp
)

add_library(mpc-wrapper SHARED ${MPC_SOURCES})

target_include_directories(mpc-wrapper PUBLIC
     /usr/include/
     headers/
     launcher/
     external/
)

install(FILES ${MPC_SOURCES} DESTINATION "${INSTALL_INC_DIR}")

add_executable(DDPSolver launcher/ddp_solver.cpp)

target_include_directories(DDPSolver PUBLIC
    /usr/include/
    launcher/
    headers/
)


