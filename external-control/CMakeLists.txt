cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 17)

project(external-control CXX)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

file(GLOB_RECURSE SOURCES
    "src/main/cpp/*.cpp"
    "src/main/cpp/*.hpp"
)

add_library(external-control SHARED ${SOURCES})
target_include_directories(external-control PUBLIC
        src/main/cpp)

install(TARGETS external-control
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)
install(DIRECTORY src/main/cpp/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include FILES_MATCHING PATTERN "*.hpp")

target_link_libraries(external-control Eigen3::Eigen zmq)
