// Author : Giovanni Cioffi <cioffi at ifi dot uzh dot ch>.
// Reference : svo/common/imu_calibration.h.

#pragma once

#include <deque>
#include <iostream>
#include <svo/common/types.h>

namespace svo {

/// \brief Settings for the Global Position configuration.
class GlobalPositionsSettings
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<GlobalPositionsSettings> Ptr;
  enum class CoordinateType { LOCAL = 0, GEOGRAPHIC = 1, UNDEFINED = 2};

  /// Measurement covariance.
  /// cov = [cov_x, 0, 0; 0, cov_y, 0; 0, 0, cov_z]
  Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
  /// B: Sensor Body, P: Prism nodal point.
  Eigen::Vector3d B_t_PB = Eigen::Vector3d::Zero();
  /// Max number of Rts residual terms per frame.
  int max_num_residuals = 1;
  // Initialize with a known orientation (for Optitrack experiments.).
  bool initial_orientation_known = false;
  bool estimate_world_vio_transform = false;
  CoordinateType coordinates = CoordinateType::UNDEFINED;
  double alignement_threshold = 0.1;
  double alignement_speed_min = 0.1;
  double alignement_d_min = 1;

  GlobalPositionsSettings() = default;
  ~GlobalPositionsSettings() = default;

  inline void print(const std::string& s = "GlobalPositions settings: ") const
  {
    std::cout << s << std::endl
              << "cov = " << cov << std::endl
              << "B_t_PB = " << B_t_PB << std::endl
              << "max_num_residuals = " << max_num_residuals << std::endl
              << "initial_orientation_known = " << initial_orientation_known << std::endl;
  }
};

struct GpMeasurement
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double timestamp_; ///< In seconds.
  Eigen::Vector3d position_;
  GpMeasurement() {}
  GpMeasurement(
      const double timestamp,
      const Eigen::Vector3d& position)
  : timestamp_(timestamp)
  , position_(position)
  {}
};
typedef std::deque<GpMeasurement,
Eigen::aligned_allocator<GpMeasurement> > GpMeasurements;

} // namespace svo
