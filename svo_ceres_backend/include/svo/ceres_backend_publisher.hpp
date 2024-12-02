// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2017 Jonathan Huber <jonathan.huber at uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).

#pragma once

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <svo/vio_common/backend_types.hpp>
#include <mutex>

#include "svo/ceres_backend/map.hpp"

namespace svo
{
class CeresBackendPublisher
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<CeresBackendPublisher> Ptr;
  using PointCloud = pcl::PointCloud<pcl::PointXYZI>;
  using PointType = pcl::PointXYZI;
  const std::string kWorldFrame = "world";
  const std::string kVIOFrame = "vio";

  CeresBackendPublisher(const ros::NodeHandle& nh_private,
                        const std::shared_ptr<ceres_backend::Map>& map_ptr);
  ~CeresBackendPublisher()
  {
  }

  Transformation getLastT_W_B() const
  {
    return state_.get_T_W_B();
  }

  void set_T_w_vio(Transformation T_w_vio) 
  {
    T_w_vio_ = T_w_vio;
    T_vio_w_ = T_w_vio.inverse();
    transform_world_vio_initialized = true;
  }

  void update_refinement_state(bool is_refined)
  {
    T_w_vio_Refined = is_refined;
  }
 
  void set_T_w_vio_temp(Transformation T_world_vio_first_estimate) 
  {
    T_world_vio_first_estimate_ = T_world_vio_first_estimate;
  }

  void set_last_KD_ID(const BundleId& bundle_id)
  {
    last_KF_ID_ = bundle_id;
  }

  void addFrame(const BundleId& bundle_id)
  {
    std::lock_guard<std::mutex> lock(mutex_frame_id_);
    last_added_frame_ = bundle_id;
  }

  void publish(const ViNodeState& state, const int64_t timestamp,
               const int32_t seq, bool last_f_is_KF);

private:
  ros::NodeHandle pnh_;

  mutable std::mutex mutex_frame_id_;

  std::shared_ptr<ceres_backend::Map> map_ptr_;  ///< The underlying svo::Map.

  // Transform used for tracing
  ViNodeState state_;
  BundleId state_frame_id_ = -1;
  BundleId last_added_frame_ = -1;

  BundleId last_KF_ID_ = -1;

  Transformation T_w_vio_;
  Transformation T_world_vio_first_estimate_;
  Transformation T_vio_w_;

  bool transform_world_vio_initialized = false;
  bool T_w_vio_Refined = false;

  bool isInitialized() const
  {
    return transform_world_vio_initialized;
  }

  // publisher helpers
  ros::Publisher pub_vio_imu_pose_;
  ros::Publisher pub_vio_imu_pose_viz_;
  ros::Publisher pub_vio_points_;

  ros::Publisher pub_w_imu_pose_;
  ros::Publisher pub_w_pose_points_;
  ros::Publisher pub_w_imu_pose_viz_;
  ros::Publisher pub_w_points_;

  geometry_msgs::PoseWithCovarianceStamped makePoseWithCovarianceMsg(
    const Eigen::Quaterniond& q,
    const Eigen::Vector3d& p,
    const ros::Time& time,
    int32_t seq, const std::string& frame_id);


  geometry_msgs::PoseStamped makePoseMsg(
    const Eigen::Quaterniond& q,
    const Eigen::Vector3d& p,
    const ros::Time& time,
    int32_t seq, const std::string& frame_id) ;

  // publisher functions
  void publishImuPose(const ViNodeState& state, const int64_t timestamp,
                      const int32_t seq);


  void publishBackendLandmarks(const ViNodeState& state, const int64_t timestamp, 
                              bool last_f_is_KF, const int32_t seq) ;
};

}  // namespace svo
