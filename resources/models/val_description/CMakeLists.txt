cmake_minimum_required(VERSION 2.8.3)
project(val_description)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  xacro 
)

set(urdf_files "")

# Xacro files
file(GLOB model_xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/robots/xacro/*/*.xacro)

# Robot files
file(GLOB urdf_xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/robots/*.xacro)

# Create urdf dir
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/urdf)

foreach(it ${urdf_xacro_files})
  get_filename_component(basepath ${it} PATH)
  get_filename_component(basename ${it} NAME_WE)

  message("processing urdf file :" ${basepath}/${basename})

  set(expanded_file "${CMAKE_CURRENT_SOURCE_DIR}/urdf/${basename}.urdf")
  message("produce expanded file :" ${expanded_file})

  xacro_add_xacro_file(${it} ${expanded_file})
  
  set(urdf_files ${urdf_files} ${expanded_file})
endforeach(it)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES val_description
#  CATKIN_DEPENDS xacro
#  DEPENDS system_lib
)

add_custom_target(val_description_media_files ALL DEPENDS ${urdf_files})