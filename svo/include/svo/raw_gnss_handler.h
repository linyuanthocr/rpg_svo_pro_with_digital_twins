// Author : Giovanni Cioffi <cioffi at ifi dot uzh dot ch>.
// Reference : svo/imu_handler.h.

#pragma once

#include <memory>          // std::shared_ptr
#include <mutex>           // std::mutex

#include <svo/common/types.h>
#include <svo/common/transformation.h>
#include <svo/common/global_positions_settings.h>


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>


namespace svo {

__inline__ size_t roundToNearestInt(size_t x, size_t y)
{
  size_t yhalf = y/2;
  size_t q = x/y;
  size_t diff = x - q*y;

  if(diff > yhalf)
  {
    return ++q;
  }
  else
  {
    return q;
  }
}

class RawGnssHandler
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<RawGnssHand> Ptr;
  typedef std::mutex mutex_t;
  typedef std::unique_lock<mutex_t> ulock_t;

  GlobalPositionsSettings gp_settings_;

  RawGnssHand(const GlobalPositionsSettings& gp_settings);
  ~RawGnssHand();

  /// Get Covariance
  const Eigen::Matrix3d& getCovariance() const
  {
    return gp_settings_.cov;
  }

  /// Get body - prism distance
  const Eigen::Vector3d& getBodyPrismDistance() const
  {
    return gp_settings_.B_t_PB;
  }

  /// Get measurement till a given time.
  /// If a set of measurements is available, get the middle one.
  bool getMeasurementTillTime(
      const double timestamp,
      const double oldest_timestamp,
      GpMeasurement& extracted_measurement,
      const bool remove_measurements);

  /// Get measurements till a given time.
  bool getMeasurementsTillTime(
      const double timestamp,
      const double oldest_timestamp,
      GpMeasurements& extracted_measurements,
      const size_t max_num_measurements,
      const int pickup_strategy,
      const bool remove_measurements);

  /// Get measurements in some time interval. Note that you have to provide
  /// the camera timestamps.
  bool getMeasurementsInInterval(
      const double old_cam_timestamp, // seconds
      const double new_cam_timestamp, // seconds
      GpMeasurements& extracted_measurements);

  bool addGpMeasurement(const GpMeasurement& measurement);

  bool addAlignmentGpMeasurement(const GpMeasurement& measurement);

  bool addAlignmentVIOMeasurement(const GpMeasurement& measurement); // GP measurements ? It isnt gp but should contains timestamp + x y z

  static GlobalPositionsSettings loadSettingsFromFile(const std::string& filename);

  bool readInitialPose() {return read_initial_pose_;}

  void setReadInitialPose(bool read_initial_pose)
  {
    read_initial_pose_ = read_initial_pose;
  }

  bool setInitialOrientation(const Eigen::Quaterniond& q_W_B);

  bool setInitialPose(const Eigen::Vector3d& t_W_B, const Eigen::Quaterniond& q_W_B);

  bool loadGpMeasurementsFromFile(const std::string& filename, const size_t first_meas_id);

  bool estimateTransformation(Transformation& T_W_GP_);

  bool alignFrames(const double timestamp);

  void reset();

private:
  int it_trust_alignement = 0; 
  mutable mutex_t measurements_mut_;
  GpMeasurements measurements_; ///< Newest measurement is at the front of the list
  GpMeasurements alignment_gp_measurements_; ///< Newest measurement is at the front of the list
  GpMeasurements alignment_vio_measurements_; ///< Newest measurement is at the front of the list
  bool read_initial_orientation_ = false;
  bool read_initial_pose_ = true;
  Transformation T_w_vio_init_;
  Transformation T_w_vio;
  Eigen::Quaterniond q_W_B_; /// changes if initial orientation of Body is known.

};

} // namespace svo
