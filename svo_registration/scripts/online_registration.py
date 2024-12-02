#!/usr/bin/env python
import argparse
import rospy
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped
import transforms3d as t3d
import tf.transformations
import math
from threading import Lock
import threading
import yaml
from svo_msgs.msg import PoseWithCloud
from std_srvs.srv import Empty, EmptyResponse

DEFAULT_VOXEL_SIZE = 0.1
WORLD_FRAME_ID = "world"

class CityMapRegistrator:

    def __init__(self, config_file, reference_map):
        self.lock = Lock()
        print(f"READING FROM {reference_map}")
        self.load_config(config_file,reference_map)
        self.init_ros_nodes()
        self.republish_service = rospy.Service('/visualize_city_twin', Empty, self.republish_city_twin)
        self.registration_estimate = np.eye(4)

    def republish_city_twin(self, request):
        rospy.loginfo("Re-publishing point clouds on demand.")
        self.publish_city_twin()
        return EmptyResponse()
    
    def publish_city_twin(self):
        point_cloud_msg = self.pc_to_ros_pointcloud2(self.city_twin)
        point_cloud_msg.header.stamp = rospy.Time.now()
        point_cloud_msg.header.frame_id = WORLD_FRAME_ID
        self.city_twin_publisher.publish(point_cloud_msg)

    def init_ros_nodes(self):
        rospy.init_node('map_registration_node', anonymous=True)

        self.w_svo_pose_point_est_sub = rospy.Subscriber("/svo/w_backend_pose_points", PoseWithCloud, self.svo_reg_callback, queue_size=1)
        self.registered_point_cloud_pub = rospy.Publisher("/svo/w_backend_points_registered", PointCloud2, queue_size=1)
        self.registration_T_w_b_pub = rospy.Publisher("/svo/w_registrated_pose", PoseWithCovarianceStamped, queue_size=1)
        self.city_twin_publisher = rospy.Publisher('/city_twin', PointCloud2, queue_size=1, latch=True)

    def load_config(self, config_file,reference_map):
        try :
            with open(config_file, 'r') as file:
                config = yaml.safe_load(file)['registration_settings']
        except FileNotFoundError:
            rospy.logerr(f"Configuration file not found: {config_file}")
            rospy.signal_shutdown("Shutting down due to missing config file.")
        except yaml.YAMLError as e:
            rospy.logerr(f"Error loading YAML configuration: {str(e)}")
            rospy.signal_shutdown("Shutting down due to YAML error.")
    
        self.valid = True
        self.scale_factor_position = config['scale_factor_position']
        self.scale_factor_rotation = config['scale_factor_rotation']
        self.max_correspondence_distance = config['max_correspondence_distance']
        self.inliers_rmse_thr = config['inliers_rmse_thr']
        self.max_translation_distance = config['max_translation_distance']
        self.power = config['information_matrix_power']
        self.max_rotation_distance = config['max_rotation_distance']
        self.size_bounding_box = config['size_bounding_box']

        self.city_twin = self.load_and_filter_point_cloud(reference_map)
        self.city_twin = self.city_twin.voxel_down_sample(voxel_size=1)

    def load_and_filter_point_cloud(self, ply_file_path):
        try :
            point_cloud = o3d.io.read_point_cloud(ply_file_path)
        except FileNotFoundError:
            rospy.logerr(f"Reference map not found: {ply_file_path}")

        points = np.asarray(point_cloud.points)

        # Filter out points below z=0 (underground)
        mask = points[:, 2] >= 0
        filtered_points = points[mask]

        # Create a new point cloud with filtered points
        filtered_point_cloud = o3d.geometry.PointCloud()
        filtered_point_cloud.points = o3d.utility.Vector3dVector(filtered_points)

        # If normals exist, filter and assign them to the new point cloud
        if point_cloud.has_normals():
            normals = np.asarray(point_cloud.normals)
            filtered_normals = normals[mask]
            filtered_point_cloud.normals = o3d.utility.Vector3dVector(filtered_normals)

        return filtered_point_cloud
   
    def calculate_information_matrix(self, source_points, target_normals):
        source_points = np.asarray(source_points)
        target_normals = np.asarray(target_normals)
        Apn = np.zeros((6,6))
        arr_normal = np.zeros((3,1))

        for a_i, n_i in zip(source_points, target_normals):
            arr_normal[0] += n_i[0]
            arr_normal[1] += n_i[1]
            arr_normal[2] += n_i[2]
            cross_prod = np.cross(a_i, n_i)
            cross_prod = cross_prod / np.linalg.norm(cross_prod)
            H_i = np.vstack([-cross_prod.reshape(-1, 1), -n_i.reshape(-1, 1)]).T
            Apn += H_i.T @ H_i
            
        # rotation and position were inversed
        Apn_correct_order = np.zeros((6,6))
        Apn_correct_order[:3,:3] = Apn[3:,3:]
        Apn_correct_order[3:,3:] = Apn[:3,:3]
        Apn_correct_order[:3,3:] = Apn[3:,:3]
        Apn_correct_order[3:,:3] = Apn[:3,3:]
        Apn_correct_order = np.linalg.matrix_power(Apn_correct_order, self.power)

        return Apn_correct_order

    def transformation_from_pose(self, pose_msg):

        q = pose_msg.pose.orientation
        quaternion = [q.w, q.x, q.y, q.z]
        rot_matrix = t3d.quaternions.quat2mat(quaternion)

        position = [pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z]

        transformation = np.eye(4)
        transformation[:3, :3] = rot_matrix
        transformation[:3, 3] = position

        return transformation

    def create_pose_with_covariance_stamped(self, registration_est, informations, stamp, frame_id=WORLD_FRAME_ID):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = stamp 
        pose_msg.header.frame_id = frame_id

        rotation_matrix = registration_est[:3, :3]
        translation_vector = registration_est[:3, 3]
        
        quaternion = tf.transformations.quaternion_from_matrix(
            np.vstack([np.hstack([rotation_matrix, [[0], [0], [0]]]), [0, 0, 0, 1]])
        )
        
        norm = np.linalg.norm(quaternion)
        if norm != 1:
            quaternion /= norm

        pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, pose_msg.pose.pose.position.z = translation_vector
        pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w = quaternion
        pose_msg.pose.covariance = tuple(informations.flatten())

        return pose_msg
    
    def pc_to_ros_pointcloud2(self, pointcloud):
        points = np.asarray(pointcloud.points, dtype=np.float32)
        header = Header(frame_id=WORLD_FRAME_ID) 
        ros_cloud = point_cloud2.create_cloud_xyz32(header, points)
        return ros_cloud
    
    def filter_city_twin(self, T_world_body):
        # Bound the target point cloud to limit optimization time.
        min_point, max_point = T_world_body[:3, 3] - self.size_bounding_box, T_world_body[:3, 3] + self.size_bounding_box

        ref_points_np = np.asarray(self.city_twin.points)
        ref_normals_np = np.asarray(self.city_twin.normals)

        mask = np.all((ref_points_np >= min_point) & (ref_points_np <= max_point), axis=1)
        target_point_cloud = o3d.geometry.PointCloud()
        target_point_cloud.points = o3d.utility.Vector3dVector(ref_points_np[mask])
        target_point_cloud.normals = o3d.utility.Vector3dVector(ref_normals_np[mask])
        
        return target_point_cloud
    
    def filter_local_map(self, msg, T_world_body):
        input_points = np.asarray(
            list(point_cloud2.read_points(msg.point_cloud, skip_nans=True, field_names=("x", "y", "z")))
        )
        min_point, max_point = T_world_body[:3, 3] - self.size_bounding_box, T_world_body[:3, 3] + self.size_bounding_box
        filtered_points = input_points[np.all((input_points >= min_point) & (input_points <= max_point), axis=1)]

        filtered_pointcloud = o3d.geometry.PointCloud()
        filtered_pointcloud.points = o3d.utility.Vector3dVector(filtered_points)
        filtered_pointcloud, _ = filtered_pointcloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=0.5)

        return filtered_pointcloud.voxel_down_sample(voxel_size=0.1)

    def svo_reg_callback(self, msg):
        with self.lock:
            T_world_body = self.transformation_from_pose(msg.pose)
            current_vio_pointcloud = self.filter_local_map(msg, T_world_body)
            target_point_cloud = self.filter_city_twin(T_world_body)

            icp_result = o3d.pipelines.registration.registration_icp(
                source=current_vio_pointcloud,
                target=target_point_cloud, 
                max_correspondence_distance=self.max_correspondence_distance,
                init=np.identity(4),
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=25),
            )

            self.valid = self.validate_icp_result(icp_result)

            if self.valid:
                reg_T_WB = icp_result.transformation @ T_world_body

                matched_target_indices = [tgt for _, tgt in icp_result.correspondence_set]
                target_normals = np.asarray(target_point_cloud.normals)[matched_target_indices]
                source_indices = [src for src, _ in icp_result.correspondence_set]
                source_points = np.asarray(current_vio_pointcloud.points)[source_indices]

                A_pn = self.calculate_information_matrix(source_points, target_normals)
                current_vio_pointcloud.transform(icp_result.transformation)

                A_pn = (A_pn/np.max(A_pn))* np.exp(-0.9 * icp_result.inlier_rmse) 
                A_pn[:3,:3] *= self.scale_factor_position*self.scale_factor_position
                A_pn[3:6, 3:6] *= self.scale_factor_rotation*self.scale_factor_rotation
                A_pn[:3, 3:6] *= self.scale_factor_rotation*self.scale_factor_position
                A_pn[3:6,:3] *= self.scale_factor_rotation*self.scale_factor_position
                pose_msg_w_b = self.create_pose_with_covariance_stamped(reg_T_WB, A_pn, msg.header.stamp)
            
                self.registration_T_w_b_pub.publish(pose_msg_w_b)
                registrated_pointcloud = self.pc_to_ros_pointcloud2(current_vio_pointcloud)
                registrated_pointcloud.header.stamp = msg.header.stamp
                registrated_pointcloud.header.frame_id = WORLD_FRAME_ID
                self.registered_point_cloud_pub.publish(registrated_pointcloud)

    def validate_icp_result(self, icp_result):
        if icp_result.fitness < 0.75 or icp_result.inlier_rmse > self.inliers_rmse_thr:
            return False

        # The following 2 conditions are because we assume that the results given by the VIO 
        # is not completely off, so we accept only results "close by"
        # If the result given by the VIO was too off, a realignment from world frame to 
        # local frame would be necessary and we shouldn't return a value in any case. 
        translation_norm = np.linalg.norm(icp_result.transformation[:3, 3])
        rotation_angle = np.degrees(np.arccos((np.trace(icp_result.transformation[:3, :3]) - 1) / 2))
        if translation_norm > self.max_translation_distance or rotation_angle > self.max_rotation_distance:
            return False

        return True
            

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Node to register local point cloud to the city twin.")
    parser.add_argument('--config-file', type=str) 
    parser.add_argument('--reference-map', type=str) 
    # args = parser.parse_args()
    args, unknown = parser.parse_known_args()

    processor = CityMapRegistrator(args.config_file, args.reference_map)
    rospy.loginfo("CityMapRegistrator node started. Waiting for messages...")
    try:
        pointcloud_thread = threading.Thread(target=processor.publish_city_twin)
        pointcloud_thread.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("PointCloudProcessor node shutdown.")
