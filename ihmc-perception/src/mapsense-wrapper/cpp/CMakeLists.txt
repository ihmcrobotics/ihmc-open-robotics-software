cmake_minimum_required(VERSION 3.4)
project(mapsense)

add_compile_options(-std=c++17)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/modules/" ${CMAKE_MODULE_PATH})
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
# set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
# set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")


find_package(OpenCV REQUIRED)
find_package(OpenCL REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)


# include_directories(
#         /usr/include/CL/
# )

set(MS_SOURCES

    feature_detection/map_frame.cpp
    feature_detection/map_frame.h
    feature_detection/map_frame_processor.cpp
    feature_detection/map_frame_processor.h
    feature_detection/planar_region_calculator.cpp
    feature_detection/planar_region_calculator.h
    feature_detection/planar_region_processor.cpp
    feature_detection/planar_region_processor.h

    
    geometry/region_ring.h
    geometry/region_ring.cpp
    geometry/planar_region.h
    geometry/planar_region.cpp
    geometry/rigid_body_transform.h
    geometry/rigid_body_transform.cpp
    geometry/geom_tools.h
    geometry/geom_tools.cpp
    geometry/hull_tools.h
    geometry/hull_tools.cpp
    geometry/line_tools.h
    geometry/line_tools.cpp
    geometry/point_cloud.h
    geometry/point_cloud.cpp
    geometry/line_2d.h
    geometry/line_2d.cpp


    opencl/opencl_manager.cpp
    opencl/opencl_manager.h


    launcher/mapsense_headless_launcher.cpp
    launcher/application_state.h
    launcher/application_state.cpp
)


add_executable(MapsenseHeadless ${MS_SOURCES})
target_link_libraries(MapsenseHeadless
    tbb
    Eigen3::Eigen
    ${OpenCV_LIBS}
    /usr/lib/x86_64-linux-gnu/libOpenCL.so
 #   eigen
)
target_include_directories(MapsenseHeadless PUBLIC
    
    /usr/include/

    feature_detection/
    geometry/
    launcher/
    opencl/
)