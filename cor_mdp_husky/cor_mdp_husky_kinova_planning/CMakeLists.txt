cmake_minimum_required(VERSION 3.0.2)
project(cor_mdp_husky_kinova_planning)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  moveit_msgs
  std_msgs
  actionlib_msgs
  actionlib
  kortex_driver
  std_srvs
  check_for_humans
)



## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cor_mdp_husky_kinova_planning
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include ${PROJECT_SOURCE_DIR}/src)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
    ${catkin_INCLUDE_DIRS}
)
