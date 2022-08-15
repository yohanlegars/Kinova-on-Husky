cmake_minimum_required(VERSION 3.0.2)
project(gazebo_attach_model_plugin)


find_package(catkin REQUIRED)
find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS} include)

# set a attach_model_plugin variable
set(attach_model_plugin AttachModelPlugin)

# specify the exported library
catkin_package(
	LIBRARIES ${attach_model_plugin}
)

# build AttachModelPlugin library
add_library(${attach_model_plugin} SHARED src/AttachModelPlugin.cc)
target_link_libraries(${attach_model_plugin} ${GAZEBO_LIBRARIES})
