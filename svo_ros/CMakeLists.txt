project(svo_ros)
cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_BUILD_TYPE Debug)

message("CMAKE_PREFIX_PATH is ${CMAKE_PREFIX_PATH}")

find_package(catkin_simple REQUIRED)
find_package(rostest REQUIRED)
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

find_package(OpenCV REQUIRED)
find_package(GeographicLib REQUIRED) # new !
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  std_msgs
  geometry_msgs
)
catkin_simple()

set(USE_GTSAM false)
include(SvoSetup)
if(USE_GLOBAL_MAP OR USE_GTSAM)
  find_package(GTSAMCMakeTools REQUIRED)
  find_package(GTSAM REQUIRED)
  set(GTSAM_LIBRARIES "gtsam")
endif()
message(STATUS "GTSAM include dir: ${GTSAM_INCLUDE_DIR}")
message(STATUS "GTSAM library: ${GTSAM_LIBRARIES}")
include_directories(
  ${GTSAM_INCLUDE_DIR}
  ${GeographicLib_INCLUDE_DIRS} # new !
  ${catkin_INCLUDE_DIRS}
  )

set(HEADERS
  include/svo_ros/svo_interface.h
  include/svo_ros/svo_factory.h
  include/svo_ros/visualizer.h
  include/svo_ros/svo_nodelet.h
  include/svo_ros/csv_dataset_reader.h
  include/svo_ros/ceres_backend_factory.h
  include/svo_ros/svo_node_base.h
)

set(SOURCES
  src/ceres_backend_factory.cpp
  src/svo_interface.cpp
  src/svo_factory.cpp
  src/svo_node_base.cpp
  src/visualizer.cpp
  src/csv_dataset_reader.cpp
)

# not used anymore
if(USE_GTSAM)
    ADD_DEFINITIONS(-DSVO_USE_GTSAM_BACKEND)
    list(APPEND HEADERS
      include/svo_ros/backend_visualizer.h
      include/svo_ros/backend_factory.h)
    list(APPEND SOURCES
      src/backend_visualizer.cpp
      src/backend_factory.cpp)
endif()

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(
  ${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${svo_backend_LIBRARIES} 
  ${GTSAM_LIBRARIES}
  ${GeographicLib_LIBRARIES} # new !
  )

cs_add_executable(svo_node src/svo_node.cpp)
target_link_libraries(svo_node ${PROJECT_NAME})

cs_add_library(svo_nodelet src/svo_nodelet.cpp)
target_link_libraries(svo_nodelet ${PROJECT_NAME})

cs_install()
install(FILES svo_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



