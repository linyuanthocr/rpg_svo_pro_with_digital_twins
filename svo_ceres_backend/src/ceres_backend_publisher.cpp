#include "svo/ceres_backend_publisher.hpp"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <svo_msgs/PoseWithCloud.h>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <vikit/output_helper.h>
#include <vikit/params_helper.h>

namespace
{
template <typename T>
void normalizeVector(const std::vector<T>& in, std::vector<float>* out)
{
  auto res = std::minmax_element(in.begin(), in.end());
  const T min = *res.first;
  const T max = *res.second;
  const float dist = static_cast<float>(max - min);

  out->resize(in.size());
  for (size_t i = 0; i < in.size(); i++)
  {
    (*out)[i] = (in[i] - min) / dist;
  }
}
}

namespace svo
{
CeresBackendPublisher::CeresBackendPublisher(
    const ros::NodeHandle& nh_private,
    const std::shared_ptr<ceres_backend::Map>& map_ptr)
  : pnh_(nh_private)
  , map_ptr_(map_ptr)
{
  pub_vio_imu_pose_ = pnh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("vio_backend_pose_imu", 10);
  pub_vio_imu_pose_viz_ =  pnh_.advertise<geometry_msgs::PoseStamped>("vio_backend_pose_imu_viz", 10);
  pub_vio_points_ = pnh_.advertise<PointCloud>("vio_backend_points", 10);
  pub_w_imu_pose_ = pnh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("w_backend_pose_imu", 10);
  pub_w_imu_pose_viz_ =  pnh_.advertise<geometry_msgs::PoseStamped>("w_backend_pose_imu_viz", 10);
  pub_w_points_ = pnh_.advertise<PointCloud>("w_backend_points", 10);

  pub_w_pose_points_ = pnh_.advertise<svo_msgs::PoseWithCloud>("w_backend_pose_points", 1);
}

void CeresBackendPublisher::publish(const ViNodeState& state,
                                    const int64_t timestamp, // in NS
                                    const int32_t seq, 
                                    bool last_f_is_KF)
{
  publishImuPose(state, timestamp, seq);
  publishBackendLandmarks(state, timestamp, last_f_is_KF, seq);
}



void CeresBackendPublisher::publishImuPose(const ViNodeState& state,
                                           const int64_t timestamp,
                                           const int32_t seq)
{
  // Trace state
  state_ = state;

  {
    std::lock_guard<std::mutex> lock(mutex_frame_id_);
    state_frame_id_ = BundleId(seq);
  }

  size_t w_n_pose_sub = pub_w_imu_pose_.getNumSubscribers();
  size_t w_n_pose_viz_sub = pub_w_imu_pose_viz_.getNumSubscribers();

  size_t vio_n_pose_sub = pub_vio_imu_pose_.getNumSubscribers();
  size_t vio_n_pose_viz_sub = pub_vio_imu_pose_viz_.getNumSubscribers();
  if (w_n_pose_sub == 0 && w_n_pose_viz_sub == 0 && vio_n_pose_sub == 0 && vio_n_pose_viz_sub == 0)
  {
    return;
  }
  VLOG(100) << "Publish IMU Pose";

  // publish in VIO frame
  Eigen::Quaterniond vio_q = state.get_T_W_B().getRotation().toImplementation();
  Eigen::Vector3d vio_p = state.get_T_W_B().getPosition();  

  // in world frame
  Transformation T_vio(vio_q, vio_p);
  Transformation T_world = T_w_vio_ * T_vio;
  Transformation T_world_temp = T_world_vio_first_estimate_ * T_vio;
  Eigen::Quaterniond w_q = T_world.getRotation().toImplementation();
  Eigen::Vector3d w_p = T_world.getPosition();
  Eigen::Quaterniond w_q_temp = T_world_temp.getRotation().toImplementation();
  Eigen::Vector3d w_p_temp = T_world_temp.getPosition();

  ros::Time time = ros::Time().fromNSec(timestamp);

  if (vio_n_pose_sub > 0)
  {
    geometry_msgs::PoseWithCovarianceStamped msg_pose = makePoseWithCovarianceMsg(vio_q, vio_p, time, seq, kVIOFrame);
    pub_vio_imu_pose_.publish(msg_pose);
  }

  if (vio_n_pose_viz_sub > 0)
  {
    geometry_msgs::PoseStamped msg_pose = makePoseMsg(vio_q, vio_p, time, seq, kVIOFrame);
    pub_vio_imu_pose_viz_.publish(msg_pose);
  }

  if(transform_world_vio_initialized) {
    if (w_n_pose_sub > 0)
    {
      geometry_msgs::PoseWithCovarianceStamped msg_pose = makePoseWithCovarianceMsg(w_q, w_p, time, seq, kWorldFrame);
      pub_w_imu_pose_.publish(msg_pose);
    }

    if (w_n_pose_viz_sub > 0)
    {
      geometry_msgs::PoseStamped msg_pose = makePoseMsg(w_q, w_p, time, seq, kWorldFrame);
      pub_w_imu_pose_viz_.publish(msg_pose);
    }
  }

}

geometry_msgs::PoseWithCovarianceStamped CeresBackendPublisher::makePoseWithCovarianceMsg(
    const Eigen::Quaterniond& q,
    const Eigen::Vector3d& p,
    const ros::Time& time,
    int32_t seq, const std::string& frame_id)
{
  geometry_msgs::PoseWithCovarianceStamped msg_pose; //(      new geometry_msgs::PoseWithCovarianceStamped);
  msg_pose.header.seq = seq;
  msg_pose.header.stamp = time;
  msg_pose.header.frame_id = frame_id;
  msg_pose.pose.pose.position.x = p[0];
  msg_pose.pose.pose.position.y = p[1];
  msg_pose.pose.pose.position.z = p[2];
  msg_pose.pose.pose.orientation.x = q.x();
  msg_pose.pose.pose.orientation.y = q.y();
  msg_pose.pose.pose.orientation.z = q.z();
  msg_pose.pose.pose.orientation.w = q.w();
  for (size_t i = 0; i < 36; ++i)
    msg_pose.pose.covariance[i] = 0;
  return msg_pose;
}

geometry_msgs::PoseStamped CeresBackendPublisher::makePoseMsg(const Eigen::Quaterniond& q,
    const Eigen::Vector3d& p,
    const ros::Time& time,
    int32_t seq, const std::string& frame_id) 
{
    geometry_msgs::PoseStamped msg_pose; //(new geometry_msgs::PoseStamped);
    msg_pose.header.seq = seq;
    msg_pose.header.stamp = time;
    msg_pose.header.frame_id = frame_id;
    msg_pose.pose.position.x = p[0];
    msg_pose.pose.position.y = p[1];
    msg_pose.pose.position.z = p[2];
    msg_pose.pose.orientation.x = q.x();
    msg_pose.pose.orientation.y = q.y();
    msg_pose.pose.orientation.z = q.z();
    msg_pose.pose.orientation.w = q.w();

    return msg_pose;

}

void CeresBackendPublisher::publishBackendLandmarks( 
    const ViNodeState& state, const int64_t timestamp, bool last_f_is_KF, const int32_t seq) 
{
  if ((pub_vio_points_.getNumSubscribers() == 0) && (pub_w_points_.getNumSubscribers() == 0))
  {
    return;
  }

  // get all landmarks
  const std::unordered_map<uint64_t, std::shared_ptr<ceres_backend::ParameterBlock> >& idmap =  map_ptr_->idToParameterBlockMap();
  size_t n_pts = 0;
  std::vector<const double*> landmark_pointers;
  std::vector<uint64_t> point_ids;

  int s =0;
  for (auto& it : idmap) //for each id of the map
  {
    if (it.second->typeInfo() == "HomogeneousPointParameterBlock" &&
        !it.second->fixed())
    {
      n_pts++;
      landmark_pointers.push_back(it.second->parameters());
      point_ids.push_back(it.second->id()); // same as it.first
    }
  }

  if (n_pts < 5)
  {
    return;
  }

  std::vector<float> intensities;
  normalizeVector(point_ids, &intensities);

  // point clound to publish
  PointCloud pc;
  ros::Time pub_time;
  pub_time.fromNSec(timestamp);
  pcl_conversions::toPCL(pub_time, pc.header.stamp);
  pc.header.frame_id = kVIOFrame;
  pc.reserve(n_pts);
  for(size_t i = 0; i < landmark_pointers.size(); i++)
  {
    const auto p = landmark_pointers[i];
    PointType pt;
    pt.intensity = intensities[i];
    pt.x = p[0];
    pt.y = p[1];
    pt.z = p[2];
    pc.push_back(pt);
  }
  pub_vio_points_.publish(pc);

  if(transform_world_vio_initialized && last_f_is_KF) {
    pc.clear();
    pc.header.frame_id = kWorldFrame;
    pcl_conversions::toPCL(pub_time, pc.header.stamp);
    pc.reserve(n_pts);

    for(size_t i = 0; i < landmark_pointers.size(); i++)
    {
      const auto p = landmark_pointers[i];
      Eigen::Vector3d point_in_vio(p[0], p[1], p[2]);
      Eigen::Vector3d point_in_w = T_w_vio_.transform(point_in_vio);
      
      PointType pt;
      pt.intensity = intensities[i];
      pt.x = point_in_w[0];
      pt.y = point_in_w[1];
      pt.z = point_in_w[2];
      pc.push_back(pt);

    }
    pub_w_points_.publish(pc);

    // Custom pose + pointcloud message encapsuling everything the registration needs
    svo_msgs::PoseWithCloud msg_pose_lm;
    msg_pose_lm.header.stamp = pub_time ; 
    msg_pose_lm.header.frame_id = kWorldFrame;
    msg_pose_lm.initial_refinement = T_w_vio_Refined;
    pcl::toROSMsg(pc, msg_pose_lm.point_cloud); // Convert pcl::PointCloud to sensor_msgs::PointCloud2

    state_ = state;
    {
      std::lock_guard<std::mutex> lock(mutex_frame_id_);
      state_frame_id_ = BundleId(seq);
    }

    Eigen::Quaterniond vio_q = state.get_T_W_B().getRotation().toImplementation();
    Eigen::Vector3d vio_p = state.get_T_W_B().getPosition();  

    // in world frame
    Transformation T_vio(vio_q, vio_p);
    Transformation T_world = T_w_vio_ * T_vio;
    Eigen::Quaterniond w_q = T_world.getRotation().toImplementation();
    Eigen::Vector3d w_p = T_world.getPosition();
    geometry_msgs::PoseStamped msg_pose = makePoseMsg(w_q, w_p, pub_time, seq, kWorldFrame);
    msg_pose_lm.pose = msg_pose;
    pub_w_pose_points_.publish(msg_pose_lm);


  }
}

}  // namespace svo
