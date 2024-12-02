// Author : Giovanni Cioffi <cioffi at ifi dot uzh dot ch>.
// Reference : svo/common/imu_calibration.h.

#pragma once

#include <deque>
#include <iostream>
#include <svo/common/types.h>

namespace svo {

/// \brief Settings for the Global Position configuration.
class GlobalPoseSettings
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<GlobalPoseSettings> Ptr;

  /// Measurement covariance.
  /// cov = [cov_x, 0, 0; 0, cov_y, 0; 0, 0, cov_z]
  Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Identity();

  /// B: Sensor Body, P: Prism nodal point.
  Eigen::Vector3d B_t_PB = Eigen::Vector3d::Zero();

  Eigen::Quaterniond B_q_PB = Eigen::Quaterniond::Identity();
  /// Max number of Rts residual terms per frame.
  int max_num_residuals = 1;
  // Initialize with a known orientation (for Optitrack experiments.).
  bool initial_orientation_known = false;

  bool estimate_world_vio_transform_via_MR = true;

  double alignement_threshold = 0.1;

  bool refine_T_WL = true; 

  GlobalPoseSettings() = default;
  ~GlobalPoseSettings() = default;

  inline void print(const std::string& s = "GlobalPose settings: ") const
  {
    std::cout << s << std::endl
              << "cov = " << cov << std::endl
              << "B_t_PB = " << B_t_PB << std::endl
              << "max_num_residuals = " << max_num_residuals << std::endl
              << "initial_orientation_known = " << initial_orientation_known << std::endl;
  }
};


struct GposeMeasurement
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double timestamp_; ///< In seconds.
  Eigen::Vector3d position_;
  Eigen::Quaterniond orientation_;
  Eigen::Matrix<double, 6, 6> information_; ///< Variance vector (3 for position, 3 for orientation)

  // Default constructor with NaN information
  GposeMeasurement() 
  : timestamp_(0)
  , position_(Eigen::Vector3d::Zero())
  , orientation_(Eigen::Quaterniond::Identity())
  , information_(Eigen::Matrix<double, 6, 6>::Constant(NAN))
  {}

  // Constructor without information (sets information to NaN)
  GposeMeasurement(
      const double timestamp,
      const Eigen::Vector3d& position, 
      const Eigen::Quaterniond& orientation)
  : timestamp_(timestamp)
  , position_(position)
  , orientation_(orientation)
  , information_(Eigen::Matrix<double, 6, 6>::Constant(NAN)) // Set information to NaN
  {}

  // Constructor with information
  GposeMeasurement(
      const double timestamp,
      const Eigen::Vector3d& position, 
      const Eigen::Quaterniond& orientation,
      Eigen::Matrix<double, 6, 6> information)
  : timestamp_(timestamp)
  , position_(position)
  , orientation_(orientation)
  , information_(information)
  {}
};

typedef std::deque<GposeMeasurement, Eigen::aligned_allocator<GposeMeasurement>> GposeMeasurements;

} // namespace svo
