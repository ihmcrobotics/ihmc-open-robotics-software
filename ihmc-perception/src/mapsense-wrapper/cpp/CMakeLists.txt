cmake_minimum_required(VERSION 3.4)
project(mapsense)

add_compile_options(-std=c++17)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/modules/" ${CMAKE_MODULE_PATH})
list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror")

set(CMAKE_BUILD_TYPE Debug)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

set(INSTALL_LIB_DIR "${CMAKE_INSTALL_PREFIX}/lib" CACHE PATH "Installation directory for libraries")
set(INSTALL_INC_DIR "${CMAKE_INSTALL_PREFIX}/include" CACHE PATH "Installation directory for headers")

message("* Video module available")
add_definitions(-DSENSORS_MOD_AVAILABLE)
add_definitions(-DVIDEO_MOD_AVAILABLE)

find_package(LIBUSB 1 REQUIRED)
message(STATUS "libusb: include dir at ${LibUSB_INCLUDE_DIRS}")
message(STATUS "libusb: libraries at ${LibUSB_LIBRARIES}")
include_directories(${LibUSB_INCLUDE_DIRS})
set(SRC_FULL ${SRC_FULL} ${SRC_VIDEO})
set(HDR_FULL ${HDR_FULL} ${HEADERS_VIDEO})
set(DEP_LIBS ${DEP_LIBS}
     ${LibUSB_LIBRARIES}
)


find_package(HIDAPI REQUIRED)
find_package(OpenCV REQUIRED)


set(ZED_SOURCES

    external/zed_open_driver_external.h
    external/zed_open_driver_external.cpp

    driver/zed_open_driver.h
    driver/zed_open_driver.cpp

    headers/defines.hpp
    headers/sensorcapture_def.hpp
    headers/sensorcapture.hpp
    headers/videocapture_def.hpp
    headers/videocapture.hpp
)

add_library(zed-driver SHARED ${ZED_SOURCES})

target_link_libraries(zed-driver
    zed_open_capture
)

target_include_directories(zed-driver PUBLIC
     /usr/include/
     /usr/include/hidapi/
     headers/
     driver/
     launcher/
     external/
)

install(TARGETS zed-driver
        ARCHIVE DESTINATION "${INSTALL_LIB_DIR}"
        LIBRARY DESTINATION "${INSTALL_LIB_DIR}" )
install(FILES ${ZED_SOURCES} DESTINATION "${INSTALL_INC_DIR}")
target_compile_definitions(zed-driver PUBLIC -DVIDEO_MOD_AVAILABLE=1)
target_compile_definitions(zed-driver PUBLIC -DSENSORS_MOD_AVAILABLE=1)

# ZED Player
add_executable(ZEDPlayer launcher/zed_player.cpp)
target_link_libraries(ZEDPlayer
    ${OpenCV_LIBS}
    zed-driver
)
target_include_directories(ZEDPlayer PUBLIC
    /usr/include/
    launcher/
    headers/
    ${OpenCV_INCLUDE_DIRS}
)


