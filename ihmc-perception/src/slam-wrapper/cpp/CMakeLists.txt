cmake_minimum_required(VERSION 3.4)
project(slam-wrapper C CXX)

add_compile_options(-std=c++17 -fPIC)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/modules/" ${CMAKE_MODULE_PATH})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

set(INSTALL_LIB_DIR "${CMAKE_INSTALL_PREFIX}/lib" CACHE PATH "Installation directory for libraries")
set(INSTALL_INC_DIR "${CMAKE_INSTALL_PREFIX}/include" CACHE PATH "Installation directory for headers")

find_package(GTSAM REQUIRED)
#find_package(Eigen3 3.3 REQUIRED NO_MODULE)

#include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${GTSAM_INCLUDE_DIR})

set(SW_SOURCES
    factorGraph/FactorGraphHandler.cpp
    factorGraph/FactorGraphHandler.h
    external/FactorGraphExternal.cpp
    external/FactorGraphExternal.h
)

add_library(slam-wrapper SHARED ${SW_SOURCES})

target_link_libraries(slam-wrapper
    gtsam
 #   eigen
)

target_include_directories(slam-wrapper PUBLIC
    factorGraph/
    external/
)

install(TARGETS slam-wrapper
        ARCHIVE DESTINATION "${INSTALL_LIB_DIR}"
        LIBRARY DESTINATION "${INSTALL_LIB_DIR}" )
install(FILES ${SW_SOURCES} DESTINATION "${INSTALL_INC_DIR}")


# +++++++++++++++++++++++++++++++++++++++ C++ Tester +++++++++++++++++++++++++++++++++++++++++++++++++++++++

add_executable(FactorGraphHeadless 
    ${SW_SOURCES}
    launcher/Application.cpp
)

target_link_libraries(FactorGraphHeadless
    gtsam
 #   eigen
)

target_include_directories(FactorGraphHeadless PUBLIC
    factorGraph/
    external/
)
