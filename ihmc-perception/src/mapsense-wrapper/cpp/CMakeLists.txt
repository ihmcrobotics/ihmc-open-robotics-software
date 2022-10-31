cmake_minimum_required(VERSION 3.4)
project(mapsense)

add_compile_options(-std=c++17)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/modules/" ${CMAKE_MODULE_PATH})
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

set(INSTALL_LIB_DIR "${CMAKE_INSTALL_PREFIX}/lib" CACHE PATH "Installation directory for libraries")
set(INSTALL_INC_DIR "${CMAKE_INSTALL_PREFIX}/include" CACHE PATH "Installation directory for headers")

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
# +++++++++++++++++++++++++++++++++++++++++++++++  Launchers       ++++++++++++++++++++++++++++++++++++++++
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


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


