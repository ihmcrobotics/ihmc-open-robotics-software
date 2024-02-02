cmake_minimum_required(VERSION 3.4)
project(mapsense)

add_compile_options(-std=c++17)
add_compile_options(-Wno-unused-parameter)

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
find_package(Eigen3 REQUIRED NO_MODULE)


# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# +++++++++++++++++++++++++++++++++++++++++++++++  Visual Odometry Sources ++++++++++++++++++++++++++++++++
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


set(MS_SOURCES

    feature_detection/visual_odometry.h
    feature_detection/visual_odometry.cpp

    geometry/point_landmark.h
    geometry/camera_model.h
    geometry/camera_model.cpp
    geometry/keyframe.h

    launcher/application_state.h
    launcher/application_state.cpp

    external/visual_odometry_external.h
    external/visual_odometry_external.cpp

    core/core.h

    tools/buffer_tools.h
    tools/buffer_tools.cpp

    tools/file_tools.h
    tools/file_tools.cpp

    tools/opencv_tools.h
    tools/opencv_tools.cpp
)

set(ZOD_SOURCES

    external/zed_open_driver_external.h
    external/zed_open_driver_external.cpp

    drivers/zed_open_driver.h
    drivers/zed_open_driver.cpp

    drivers/defines.hpp
    drivers/sensorcapture_def.hpp
    drivers/sensorcapture.hpp
    drivers/videocapture_def.hpp
    drivers/videocapture.hpp
)

# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# +++++++++++++++++++++++++++++++++++++++++++++++  Visual Odometry ++++++++++++++++++++++++++++++++++++++++
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


add_library(visual-odometry SHARED ${MS_SOURCES})

target_link_libraries(visual-odometry
    tbb
    Eigen3::Eigen
    ${OpenCV_LIBS}
)

target_include_directories(visual-odometry PUBLIC
     /usr/include/
     /usr/include/opencv4/
     feature_detection/
     geometry/
     launcher/
     external/
     core/
     tools/
)

install(TARGETS visual-odometry
        ARCHIVE DESTINATION "${INSTALL_LIB_DIR}"
        LIBRARY DESTINATION "${INSTALL_LIB_DIR}" )
install(FILES ${MS_SOURCES} DESTINATION "${INSTALL_INC_DIR}")

# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# +++++++++++++++++++++++++++++++++++++++++++++++  SensorDrivers ++++++++++++++++++++++++++++++++++++++++**
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


add_library(zed-driver SHARED ${ZOD_SOURCES})

target_link_libraries(zed-driver
    zed_open_capture
)

target_include_directories(zed-driver PUBLIC
     /usr/include/
     /usr/include/hidapi/
     drivers/
     launcher/
     external/
)

install(TARGETS zed-driver
        ARCHIVE DESTINATION "${INSTALL_LIB_DIR}"
        LIBRARY DESTINATION "${INSTALL_LIB_DIR}" )
install(FILES ${ZOD_SOURCES} DESTINATION "${INSTALL_INC_DIR}")
target_compile_definitions(zed-driver PUBLIC -DVIDEO_MOD_AVAILABLE=1)
target_compile_definitions(zed-driver PUBLIC -DSENSORS_MOD_AVAILABLE=1)


# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# +++++++++++++++++++++++++++++++++++++++++++++++  Launchers       ++++++++++++++++++++++++++++++++++++++++
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

# Visual Odometry Player
add_executable(VisualOdometryNativeDemoLauncher launcher/vo_demo_launcher.cpp launcher/vo_demo_launcher.h)
target_link_libraries(VisualOdometryNativeDemoLauncher
    visual-odometry
)
target_include_directories(VisualOdometryNativeDemoLauncher PUBLIC
    /usr/include/
    feature_detection/
    geometry/
    launcher/
)


# ZED Player
add_executable(ZEDPlayer launcher/zed_player.cpp)
target_link_libraries(ZEDPlayer
    ${OpenCV_LIBS}
    zed-driver
)
target_include_directories(ZEDPlayer PUBLIC
    /usr/include/
    launcher/
    drivers/
    ${OpenCV_INCLUDE_DIRS}
)


