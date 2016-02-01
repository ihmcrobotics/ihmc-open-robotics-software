cmake_minimum_required(VERSION 2.8.3)
project(ihmc_gazebo)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
)


find_package(Boost REQUIRED system thread)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
#message("Gazebo includes: ${GAZEBO_INCLUDE_DIRS}")
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
 
#add_library(${PROJECT_NAME} src/.cpp)
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_subdirectory(gazebo_plugin)

file(COPY model/polaris_ranger_xp900_no_roll_cage DESTINATION $ENV{HOME}/.gazebo/models)
file(COPY meshes_unplugged DESTINATION $ENV{HOME}/.gazebo/models/ihmc_gazebo)

include(cmake/rospack.cmake)

get_rospack_flags(roscpp)
include_directories(${roscpp_INCLUDE_DIRS})
message("roscpp includes: ${roscpp_INCLUDE_DIRS}")
link_directories(${roscpp_LIBRARY_DIRS})
set(CMAKE_INSTALL_RPATH "${roscpp_LIBRARY_DIRS}")

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)

