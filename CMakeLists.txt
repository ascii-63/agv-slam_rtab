cmake_minimum_required(VERSION 3.0.2)
project(agv-slam_rtabmap)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2_ros
  geometry_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES agv-slam_rtabmap
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib
  src/agv-slam_rtabmap.cpp
)
target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
  src/agv-slam_rtabmap_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
