cmake_minimum_required(VERSION 3.0.2)
project(satellite_simulator)

find_package(Ceres REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  geometry_msgs
  std_msgs
)

find_package(GeographicLib REQUIRED)
find_package(Eigen3 REQUIRED yaml-cpp REQUIRED)


add_message_files(
  FILES
  DoubleMsg.msg
  EmitterMsg.msg
  DoubleArrayMsg.msg
  IntMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  
  geometry_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES satellite_simulator
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
  ${GeographicLib_INCLUDE_DIRS}
)

add_executable(gnss_simulator src/main.cpp src/gnss.cpp src/satellite_param.cpp src/emitter.cpp src/frame_handler.cpp)
target_link_libraries(gnss_simulator ${catkin_LIBRARIES} pthread yaml-cpp ${GeographicLib_LIBRARIES} ${CERES_LIBRARIES})


install(PROGRAMS
    script/gps_error_publisher.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
