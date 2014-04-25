cmake_minimum_required(VERSION 2.6)
project(CommonWalkingControlModules)

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX
    ${CMAKE_SOURCE_DIR} CACHE PATH "CommonWalkingControlModules install prefix" FORCE
    )
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

#Compiles with -Os when using gcc, as recommended by http://cvxgen.com/speed.html
set(CMAKE_BUILD_TYPE MinSizeRel)



set(JDK "C:/Program Files/Java/jdk1.7.0_45")
set(JAVA_AWT_INCLUDE_PATH "${JDK}/include")
set(JAVA_AWT_LIBRARY "${JDK}/jawt.lib")
set(JAVA_INCLUDE_PATH "${JDK}/include")
set(JAVA_INCLUDE_PATH2 "${JDK}/include/win32")
set(JAVA_JVM_LIBRARY "${JDK}/lib/jvm.lib")
find_package(JNI REQUIRED)

add_subdirectory(csrc)
