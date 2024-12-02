# EmitterMsg.msg
std_msgs/Header header  # Standard ROS message header
geometry_msgs/Point position_ecef  # Position in ECEF frame
geometry_msgs/Point position_geo  # Position in lat long alt frame
std_msgs/Float64 range  # Additional range (float) data
std_msgs/Float64 elevation_angle  # Additional range (float) data
std_msgs/Float64 azimuth_angle  # Additional range (float) data
std_msgs/Int32 idx_satellite  # Index of the satellite