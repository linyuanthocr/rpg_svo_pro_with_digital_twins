# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "cmake_modules;cv_bridge;image_transport;nodelet;roscpp;nav_msgs;pcl_ros;sensor_msgs;std_msgs;tf;visualization_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lsvo_ros;-lsvo_nodelet".split(';') if "-lsvo_ros;-lsvo_nodelet" != "" else []
PROJECT_NAME = "svo_ros"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.1.0"
