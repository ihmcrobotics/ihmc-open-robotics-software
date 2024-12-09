cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 17)

project(external-control CXX)

set(INSTALL_LIB_DIR "${CMAKE_INSTALL_PREFIX}/lib" CACHE PATH "Installation directory for libraries")
set(INSTALL_INC_DIR "${CMAKE_INSTALL_PREFIX}/include" CACHE PATH "Installation directory for headers")

find_package(Catch2 2 REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


file(GLOB CONTACT_SOURCES
        "src/main/cpp/contact/*.hpp"
        "src/main/cpp/contact/*.cpp")
file(GLOB COST_SOURCES
        "src/main/cpp/cost/*.hpp"
        "src/main/cpp/cost/*.cpp")
file(GLOB PROBLEM_SOURCES
        "src/main/cpp/problem/*.hpp"
        "src/main/cpp/problem/*.cpp")
set(SOURCES
        ${CONTACT_SOURCES}
        ${PROBLEM_SOURCES}
        ${COST_SOURCES}
        src/main/cpp/frame.hpp
        src/main/cpp/stage.hpp
        src/main/cpp/stage.cpp
        src/main/cpp/types.hpp)

add_library(external-wrapper SHARED ${SOURCES})
target_include_directories(external-wrapper PUBLIC
        src/main/cpp)
target_link_libraries(external-wrapper PUBLIC
        Eigen3::Eigen)

install(TARGETS external-wrapper
        EXPORT targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)
install(DIRECTORY
        src/main/cpp/
        DESTINATION "${INSTALL_INC_DIR}"
        FILES_MATCHING PATTERN "*.hpp")