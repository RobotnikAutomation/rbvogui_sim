cmake_minimum_required(VERSION 2.8.3)
project(rbvogui_gazebo)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_srvs
    std_msgs
    tf
    gazebo_ros
)


###################################
## catkin specific configuration ##
###################################
catkin_package(

)

###########
## Build ##
###########


include_directories(
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############


install(
  DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
