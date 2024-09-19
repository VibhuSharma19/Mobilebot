cmake_minimum_required(VERSION 3.0.2)
project(mobilebot)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf2_ros
  message_generation
  sensor_msgs
  nav_msgs
  tf2
)




## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
  add_service_files(
    FILES
    GetTransform.srv
  )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
  generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    
  )


###################################
## catkin specific configuration ##
####################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need


catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES mobilebot
   CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs tf2_ros sensor_msgs nav_msgs tf2 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/mobilebot.cpp
# )

