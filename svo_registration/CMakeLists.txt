cmake_minimum_required(VERSION 3.0.2)
project(svo_registration)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  svo_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs message_runtime svo_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  svo_registration/scripts/online_registration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

