#!/usr/bin/env python

"""
From a elios3 bag need to : 
- extract the raw images
- convert IMU
- create GPS from the LidAR
- add GPS ???
"""

import rospy
import rosbag
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import rosbag
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Point, Quaternion

def convert_to_standard_imu(custom_imu_msg):
    standard_imu_msg = Imu()
    standard_imu_msg.header = custom_imu_msg.imu.header
    standard_imu_msg.orientation = custom_imu_msg.imu.orientation
    standard_imu_msg.orientation_covariance = custom_imu_msg.imu.orientation_covariance
    standard_imu_msg.angular_velocity = custom_imu_msg.imu.angular_velocity
    standard_imu_msg.angular_velocity_covariance = custom_imu_msg.imu.angular_velocity_covariance
    standard_imu_msg.linear_acceleration = custom_imu_msg.imu.linear_acceleration
    standard_imu_msg.linear_acceleration_covariance = custom_imu_msg.imu.linear_acceleration_covariance
    return standard_imu_msg

input_bag_path = '/home/roxane/elios3_2706/EL300825315585_01645_214_1flight/Short_Flight_Raw.bag' #/home/roxane/DatasetForPres/Real_World_V2/raw_flight_data.bag' 
output_bag_path = '/home/roxane/elios3_2706/EL300825315585_01645_214_1flight/Short_Flight.bag'  

# Image compress to raw
bridge = CvBridge()
excluded_topics = ['/sensors', '/camera_1', '/camera_0', '/control', '/alert', '/kalman_scan2map_node', '/os_node', '/obc_communication_node', '/communication', '/dvs', '/flight/', '/vio_module/health_telemetry',  '/robot/devices', '/vio/odometry_fast', '/camera_2/camera_state']

with rosbag.Bag(output_bag_path, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(input_bag_path).read_messages():
        if topic == "/camera_2/image_raw/compressed" and msg._type == "sensor_msgs/CompressedImage":
            cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
            image_msg = bridge.cv2_to_imgmsg(cv_image, encoding="mono8")
            image_msg.header = msg.header  # Preserve timestamp and frame_id
            outbag.write("/camera_2/image_raw", image_msg, t)
            print(t.to_sec())
        elif topic == '/vio/odometry':
            pose_msg = PoseStamped()
            pose_msg.header = msg.header
            pose_msg.header.frame_id = 'world'
            pose_msg.pose = msg.pose.pose
            outbag.write('/gps_lidar', pose_msg, t)
        elif topic == '/vio_module/imu_filtered':
            imu_msg = convert_to_standard_imu(msg)
            outbag.write(topic, imu_msg, t)
        elif not any(topic.startswith(excluded) for excluded in excluded_topics):
            outbag.write(topic, msg, t)
