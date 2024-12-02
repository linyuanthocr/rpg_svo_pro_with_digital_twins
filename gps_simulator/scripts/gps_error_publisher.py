#!/usr/bin/env python
import rospy
import numpy as np
import pickle
import joblib
import yaml
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, Int32
from satellite_simulator.msg import DoubleArrayMsg, IntMsg
import os 

"""
Script for estimating GPS multipath error and satellite visibility in function of the height

This script implements a ROS node that uses Gaussian Mixture Models (GMMs) to estimate 
GPS pseudorange errors (caused by multipath) based on the receiver's height. Additionally, 
it predicts the number of visible satellites using a Gaussian Process (GP) classifier.

"""

class GPSNoiseModelLoader:
    def __init__(self, model_path: str):
        with open(model_path, 'rb') as file:
            self.model_data = pickle.load(file)
        self.models = self.model_data['models']
        self.height_ranges = self.model_data['height_ranges']

    def generate_error_estimate(self, height: float) -> float:
        for model, (min_height, max_height) in zip(self.models, self.height_ranges):
            if min_height <= height < max_height:
                sample = model.sample(1)[0]
                return sample[0][0] + np.random.normal(0, 1)
        raise ValueError("Height out of range for the available models.")


class HeightErrorNode:
    def __init__(self):
        rospy.init_node('height_error_estimator')

        config_path = rospy.get_param('~config_path')
        folder = rospy.get_param("gps_noise_model_folder")

        with open(config_path, 'r') as config_file:
            config = yaml.safe_load(config_file)

        self.multipath_model_path = os.path.join(folder, config['gps_noise_model_multipath'])
        self.n_sat_model_path = os.path.join(folder, config['gps_noise_model_number_of_sat'])
        self.n_sat_scaler_path = os.path.join(folder, config['gps_noise_model_number_of_sat_scaler'])
        self.pseudorange_error_topic = config['pseudorange_error_topic']
        self.number_of_satellite_topic = config['number_of_satellite_topic']
        self.ground_truth_topic = config['ground_truth_topic']
        self.n_sat_min = config['n_sat_class_0']
        self.n_sat_max = config['n_class']

        # the models must be previously computed
        self.gmm_multipath_model = GPSNoiseModelLoader(self.multipath_model_path)
        self.n_sat_gp_classifier = joblib.load(self.n_sat_model_path)
        self.n_sat_scaler = joblib.load(self.n_sat_scaler_path)

        self.error_publisher = rospy.Publisher(self.pseudorange_error_topic, DoubleArrayMsg, queue_size=1)
        self.nsat_publisher = rospy.Publisher(self.number_of_satellite_topic, IntMsg, queue_size=1)
        self.ground_truth_subscriber = rospy.Subscriber(self.ground_truth_topic, PoseStamped, self.ground_truth_callback)

        self.last_time_called = rospy.Time.now().to_sec()
        self.callback_interval = 0.5 

    def ground_truth_callback(self, msg: PoseStamped):
        current_time = rospy.Time.now().to_sec()
        if current_time - self.last_time_called < self.callback_interval:
            return
        self.last_time_called = current_time

        height = msg.pose.position.z
        errors = self.generate_multipath_errors(height)
        nsat = self.predict_num_satellites(height)

        self.publish_errors(errors)
        self.publish_nsat(nsat)

    def generate_multipath_errors(self, height: float):
        return [self.gmm_multipath_model.generate_error_estimate(height) for _ in range(32)]

    def predict_num_satellites(self, height: float) -> int:
        scaled_height = self.n_sat_scaler.transform([[height]])[0, 0]
        keys = np.array(list(self.n_sat_gp_classifier.keys()))
        min_key, max_key = keys.min(), keys.max()

        if min_key <= scaled_height <= max_key:
            nearest_key = keys[np.argmin(np.abs(keys - scaled_height))]
            class_probs = self.n_sat_gp_classifier[nearest_key]
            return np.random.choice(a=np.arange(len(class_probs)), p=class_probs) + self.n_sat_min
        return self.n_sat_max if scaled_height > max_key else self.n_sat_min

    def publish_errors(self, errors):
        error_msg = DoubleArrayMsg(
            header=Header(stamp=rospy.Time.now(), frame_id='error_frame'),
            data=errors
        )
        self.error_publisher.publish(error_msg)

    def publish_nsat(self, nsat: int):
        nsat_msg = IntMsg(
            header=Header(stamp=rospy.Time.now(), frame_id='nsat_frame'),
            data=Int32(data=nsat)
        )
        self.nsat_publisher.publish(nsat_msg)

if __name__ == '__main__':
    try:
        HeightErrorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
