cmake_minimum_required(VERSION 2.8.3)
project(rhino_beetle_description)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  xacro
)

catkin_package(
 CATKIN_DEPENDS xacro
)

