cmake_minimum_required(VERSION 2.8.3)
project(rbvogui_sim_bringup)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
