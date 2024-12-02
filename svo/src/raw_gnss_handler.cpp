// Author : Giovanni Cioffi <cioffi at ifi dot uzh dot ch>.
// Reference : imu_handler.cpp.

#include "svo/global_positions_handler.h"
#include "svo/global_pose_handler.h"

#include <numeric>

#include <vikit/math_utils.h>
#include <vikit/csv_utils.h>
#include <vikit/timer.h>
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <yaml-cpp/yaml.h>
#pragma diagnostic pop

namespace svo {

RawGnssHandler::RawGnssHandler(
    const GlobalPositionsSettings& gp_settings)
  : gp_settings_(gp_settings)
{
  if(gp_settings_.initial_orientation_known)
  {
    read_initial_orientation_ = true;
  }
  else
  {
    q_W_B_.w() = 1;
    q_W_B_.x() = 0;
    q_W_B_.y() = 0;
    q_W_B_.z() = 0;
  }

  // reset();
}

RawGnssHandler::~RawGnssHandler()
{}

bool RawGnssHandler::getMeasurementTillTime(
    const double timestamp,
    const double oldest_timestamp,
    GpMeasurement& extracted_measurement,
    const bool remove_measurements)
{
  ulock_t lock(measurements_mut_);

  if(measurements_.empty())
  {
    VLOG(10) << "don't have any gp measurements!";
    return false;
  }

  // Find the first measurement older than timestamp,
  // note that the newest measurement is at the front of the list!
  GpMeasurements::iterator it_front = measurements_.begin();
  std::size_t num_measurements_newer_timestamp = 0;
  for(; it_front!=measurements_.end(); ++it_front)
  {
    if(it_front->timestamp_ <= timestamp)
    {
      break;
    }
    num_measurements_newer_timestamp += 1;
  }

  if(num_measurements_newer_timestamp == measurements_.size())
  {
    VLOG(10) << "need an older gp measurement!";
    return false;
  }

  // Find the first measurement newer than oldest_timestamp.
  GpMeasurements::iterator it_back = measurements_.begin();
  std::size_t num_measurements_newer_oldest_timestamp = 0;
  for(; it_back!=measurements_.end(); ++it_back)
  {
    if(it_back->timestamp_ <= oldest_timestamp)
    {
      break;
    }
    num_measurements_newer_oldest_timestamp += 1;
  }
  // If there are measurements older than oldest_timestamp,
  // move it to the first measurement newer than oldest_timestamp.
  if (!(num_measurements_newer_oldest_timestamp ==  measurements_.size()))
  {
    --it_back;
  }

  // copy affected measurement
  std::size_t num_measurements_available =
          num_measurements_newer_oldest_timestamp - num_measurements_newer_timestamp;
  if (num_measurements_available == 0)
  {
     VLOG(10) << "no gp measurements available!";
    return false;
  }
  else
  {
    std::size_t idx = num_measurements_newer_timestamp;
    // If more than 1 measurement available, pick the one at the center.
    if (num_measurements_available > 1)
    {
      std::size_t incr = num_measurements_available / 2;
      idx += incr;
    }

    extracted_measurement = measurements_.at(idx);
  }

  if (remove_measurements)
  {
    measurements_.erase(it_front, measurements_.end());
  }

  return true;
}

bool RawGnssHandler::getMeasurementsTillTime(
    const double timestamp,
    const double oldest_timestamp,
    GpMeasurements& extracted_measurements,
    const size_t max_num_measurements,
    const int pickup_strategy,
    const bool remove_measurements)
{
  ulock_t lock(measurements_mut_);
  if(measurements_.empty())
  {
    VLOG(10) << "don't have any gp measurements!";
    return false;
  }

  if(max_num_measurements == 0)
  {
    LOG(WARNING) << "Max num gp measurements is zero!";
    return false;
  }

  // Find the first measurement older than timestamp,
  // note that the newest measurement is at the front of the list!
  GpMeasurements::iterator it_front = measurements_.begin();
  size_t num_measurements_newer_timestamp = 0;
  for(; it_front!=measurements_.end(); ++it_front)
  {
    if(it_front->timestamp_ <= timestamp)
    {
      break;
    }
    num_measurements_newer_timestamp += 1;
  }

  if(num_measurements_newer_timestamp == measurements_.size())
  {
    VLOG(10) << "need an older gp measurement!";
    return false;
  }

  // Find the first measurement newer than oldest_timestamp.
  GpMeasurements::iterator it_back = measurements_.end();
  it_back--;
  std::size_t num_measurements_older_oldest_timestamp = 0;
  for(; it_back!=measurements_.begin(); --it_back)
  {
    if(it_back->timestamp_ > oldest_timestamp)
    {
      break;
    }
    num_measurements_older_oldest_timestamp += 1;
  }

  // copy affected measurements
  std::size_t num_measurements_available =
          measurements_.size() - num_measurements_older_oldest_timestamp - num_measurements_newer_timestamp;

  if (num_measurements_available > max_num_measurements)
  {
    if (pickup_strategy == 2)
    {
      // Take newest measurements
      const size_t position_oldest_measurement_to_extract = max_num_measurements +
              num_measurements_newer_timestamp;
      extracted_measurements.insert(extracted_measurements.begin(),
                                    it_front,
                                    measurements_.begin()+position_oldest_measurement_to_extract);
    }
    else if(pickup_strategy == 1)
    {
      // Take equidistant measurements      
      std::size_t step = roundToNearestInt(num_measurements_available,
                                           max_num_measurements+1);
      std::deque<std::size_t> pos;
      std::size_t pos_last_meas = num_measurements_older_oldest_timestamp + step;
      pos.push_back(pos_last_meas);

      std::size_t num_measurements_to_retain = max_num_measurements - 1;

      while(num_measurements_to_retain > 0)
      {
        pos_last_meas += step;
        pos.push_back(pos_last_meas);

        num_measurements_to_retain -=1;
      }

      for (std::size_t i = 0; i < pos.size(); ++i)
      {
        std::size_t ind = measurements_.size() - pos.at(i);
        GpMeasurement iter = measurements_.at(ind);
        extracted_measurements.push_front(iter);
      }
    }
    else if (pickup_strategy == 0)
    {
      extracted_measurements.insert(extracted_measurements.begin(),
                                    it_back-max_num_measurements+1,
                                    it_back+1);
    }
    else
    {
      LOG(ERROR) << "Unknown pickup strategy";
      return false;
    }
  }
  else
  {
    extracted_measurements.insert(extracted_measurements.begin(),
                                  it_front,
                                  it_back+1);
  }

  if (remove_measurements)
  {
    measurements_.erase(it_front, measurements_.end());
  }

  return true;
}

bool RawGnssHandler::getMeasurementsInInterval(
    const double old_cam_timestamp,
    const double new_cam_timestamp,
    GpMeasurements& extracted_measurements)
{
  assert(new_cam_timestamp > old_cam_timestamp);
  ulock_t lock(measurements_mut_);
  if(measurements_.empty())
  {
    LOG(WARNING) << "don't have any gp measurement!";
    return false;
  }

  // Newest measurements are at the front of the list!
  const double t1 = old_cam_timestamp;
  const double t2 = new_cam_timestamp;

  GpMeasurements::iterator it1=measurements_.end(); // older timestamp
  GpMeasurements::iterator it2=measurements_.end(); // newer timestamp
  bool it2_set = false;
  for(GpMeasurements::iterator it=measurements_.begin();
      it!=measurements_.end(); ++it)
  {
    if(!it2_set && it->timestamp_ < t2)
    {
      it2 = it;
      it2_set = true;
    }

    if(it->timestamp_ <= t1)
    {
      it1 = it;
      break;
    }
  }

  // check
  if(it1 == measurements_.begin())
  {
    LOG(WARNING) << "all measurements older than requested interval!";
    return false;
  }

  if(!it2_set)
  {
    LOG(WARNING) << "all measurements newer than requested interval!";
    return false;
  }

  if(it1 == it2)
  {
    if (t1 < it1->timestamp_ && t2 > it2->timestamp_)
    {
      --it2;
    }
    else
    {
      LOG(WARNING) << "no measurements in requested interval!";
      return false;
    }
  }

  // copy affected measurements
  extracted_measurements.insert(extracted_measurements.begin(), it2, it1);

  return true;
}

bool RawGnssHandler::addGpMeasurement(
      const GpMeasurement& m)
{
  GpMeasurement m_W = GpMeasurement(m.timestamp_, T_w_vio.transform(m.position_)); // Position of the measurement in the world frame

  ulock_t lock(measurements_mut_);
  
  measurements_.push_front(m_W); // new measurement is at the front of the list!

  return true;
}

bool RawGnssHandler::addAlignmentGpMeasurement(
      const GpMeasurement& m)
{
  ulock_t lock(measurements_mut_);

  alignment_gp_measurements_.push_front(m); // new at front of queue


  return true;
}

bool RawGnssHandler::addAlignmentVIOMeasurement(
      const GpMeasurement& m)
{
  ulock_t lock(measurements_mut_);

  alignment_vio_measurements_.push_front(m); // store the gnss measurement

  return true;
}

bool RawGnssHandler::estimateTransformation(Transformation& T_W_GP_) {

  if (alignment_vio_measurements_.size() != alignment_gp_measurements_.size()) {

    LOG(FATAL) << "Measurement sets have to be of the same size!";

  }


  if (alignment_vio_measurements_.size() < 3) {
    return false;
  }
    Eigen::MatrixXd p_i(3, alignment_gp_measurements_.size());
    Eigen::MatrixXd p_prime_i(3, alignment_vio_measurements_.size());

    int N = p_i.cols();


    //align the timestamp -> TODO:do this only once, dont recompute it each time.
    //TODO : align timestamp, interpolate between measurement to have same timestamp. with ratio of delta t.
    for(int i = 0; i < N; i++) {
      p_prime_i.col(i) = alignment_gp_measurements_[i].position_;
      p_i.col(i) = alignment_vio_measurements_[i].position_;
    }

 
    // Step I: Calculate p, p_prime, and then {q1i}, {q2i}.
    Eigen::Vector3d p = p_i.rowwise().sum() * 1 / N;
    Eigen::Vector3d p_prime = p_prime_i.rowwise().sum() * 1 / N;

    Eigen::MatrixXd q_i = p_i.colwise() - p;
    Eigen::MatrixXd q_prime_i = p_prime_i.colwise() - p_prime;

    // Step II: Calculate H.
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; ++i) {
        Eigen::Matrix3d H_incr;

        H_incr << q_i.col(i).x() * q_prime_i.col(i).x(), q_i.col(i).x() * q_prime_i.col(i).y(), q_i.col(i).x() * q_prime_i.col(i).z(),
                q_i.col(i).y() * q_prime_i.col(i).x(), q_i.col(i).y() * q_prime_i.col(i).y(), q_i.col(i).y() * q_prime_i.col(i).z(),
                q_i.col(i).z() * q_prime_i.col(i).x(), q_i.col(i).z() * q_prime_i.col(i).y(), q_i.col(i).z() * q_prime_i.col(i).z();

        H += H_incr;
    }

    // Step III: Find SVD of H
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd Delta = svd.singularValues();
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();

    // Step IV: Calculate X.
    Eigen::Matrix3d X = V * U.transpose();

    // Step V: Check X determinat.
    double det_X = X.determinant();
    double eps = 0.00001;

    Eigen::Matrix3d R;

    if (det_X >  (-1.0 - eps) && det_X <  (-1.0 + eps)){
      double smallest_singular_value = Delta(2);

      double second_smallest_singular_value = Delta(1);

      if (smallest_singular_value > -eps && smallest_singular_value < eps && second_smallest_singular_value > eps){
          Eigen::Matrix3d V_prime;

          V_prime << V(0,0), V(0,1), -V(0,2),
                  V(1,0), V(1,1), -V(1,2),
                  V(2,0), V(2,1), -V(2,2);

          Eigen::Matrix3d X_prime = V_prime * U.transpose();
          R = X_prime;
      }
      else {
          return false;
      }
    }
    else {
          R = X;
    }

    Eigen::Vector3d t = p_prime - R * p;

    // Compute final transformation.
    Eigen::Matrix4d T;
    T << R.transpose(), -R.transpose()*t, Eigen::MatrixXd::Zero(1,3), 1.0; //inverse of this -> just switch the initial points 

    auto temp = Transformation(T);
    auto d_yaw = abs(temp.getRotation().toImplementation().toRotationMatrix().eulerAngles(2, 1, 0)[0] - T_W_GP_.getRotation().toImplementation().toRotationMatrix().eulerAngles(2, 1, 0)[0]); 
    

    if ((temp.getPosition() - T_W_GP_.getPosition()).norm()<5e-4) { //not equal, def a threshold
        if(d_yaw<7.5e-4){ //param to put in yaml.
          T_W_GP_ = Transformation(T);
          it_trust_alignement ++;
          if(it_trust_alignement>1) {
            return true; 
          }
        } else {
          it_trust_alignement = 0;
        }
    } else {
      it_trust_alignement = 0; 
    }



    T_W_GP_ = Transformation(T);

    if(it_trust_alignement>1000){ //temp, todo: find covariance, under threshold return true
      return true;
    }
    else {
      return false;
    }

  //! @todo: insert giovanni's code here. Read from the measurements_ list and estimate the transformation T_w_vio.
  //if sigma < threshold, return true, else return false

}



bool RawGnssHandler::alignFrames(const double timestamp) { // will be called also when gnss loss

  if(!estimateTransformation(T_w_vio)){
    return false;
  }

  T_w_vio_init_ = T_w_vio; 

  read_initial_pose_ = false;

  return true; 
}


bool RawGnssHandler::setInitialPose(const Eigen::Vector3d& t_W_B, const Eigen::Quaterniond &q_W_B)
{
  if(read_initial_orientation_)
  {
    T_w_vio_init_ = Transformation(t_W_B, q_W_B);
  }
  else
  {
    T_w_vio_init_ = Transformation(t_W_B, Eigen::Quaterniond(1,0,0,0));
  }

  read_initial_pose_ = false;

  return true;
}

bool RawGnssHandler::setInitialOrientation(const Eigen::Quaterniond &q_W_B)
{
  q_W_B_.w() = q_W_B.w();
  q_W_B_.x() = q_W_B.x();
  q_W_B_.y() = q_W_B.y();
  q_W_B_.z() = q_W_B.z();

  return true;
}

GlobalPositionsSettings RawGnssHandler::loadSettingsFromFile(const std::string& filename)
{
  YAML::Node data = YAML::LoadFile(filename);
  GlobalPositionsSettings settings;
  if(data["gp_settings"].IsDefined())
  {
    settings.cov(0,0) = data["gp_settings"]["variance_x"].as<double>();
    settings.cov(1,1) = data["gp_settings"]["variance_x"].as<double>();
    settings.cov(2,2) = data["gp_settings"]["variance_x"].as<double>();

    settings.B_t_PB(0) = data["gp_settings"]["B_t_PB"][0].as<double>();
    settings.B_t_PB(1) = data["gp_settings"]["B_t_PB"][1].as<double>();
    settings.B_t_PB(2) = data["gp_settings"]["B_t_PB"][2].as<double>();

    settings.max_num_residuals = data["gp_settings"]["max_num_residuals"].as<int>();

    settings.initial_orientation_known = data["gp_settings"]["initial_orientation_known"].as<bool>();
  }
  else
  {
    LOG(FATAL) << "Could not load Global Positions calibration from file";
  }
  return settings;
}

bool RawGnssHandler::loadGpMeasurementsFromFile(const std::string& filename, const size_t first_meas_id)
{
  // ulock_t lock(measurements_mut_);
  
  return true;
}

void RawGnssHandler::reset()
{
  ulock_t lock(measurements_mut_);
  measurements_.clear();
}

} // namespace svo

