cmake_minimum_required(VERSION 3.0.2)
project(duot06_bringup)

find_package(catkin REQUIRED)

catkin_package(
)

include_directories(
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
