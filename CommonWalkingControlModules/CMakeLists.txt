cmake_minimum_required(VERSION 2.6)
project(CommonWalkingControlModules)

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX
    ${CMAKE_SOURCE_DIR} CACHE PATH "CommonWalkingControlModules install prefix" FORCE
    )
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

#Compiles with -Os when using gcc, as recommended by http://cvxgen.com/speed.html
set(CMAKE_BUILD_TYPE MinSizeRel)


find_package(JNI REQUIRED)

add_subdirectory(csrc)
