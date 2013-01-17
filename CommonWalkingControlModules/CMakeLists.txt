cmake_minimum_required(VERSION 2.6)
project(CommonWalkingControlModules)

#Compiles with -Os when using gcc, as recommended by http://cvxgen.com/speed.html
set(CMAKE_BUILD_TYPE MinSizeRel)


find_package(JNI REQUIRED)

add_subdirectory(csrc)
