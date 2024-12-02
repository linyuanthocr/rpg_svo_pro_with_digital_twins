#include "svo_ros/svo_node_base.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <svo/common/logging.h>
#include <vikit/params_helper.h>

namespace svo_ros {

void SvoNodeBase::initThirdParty(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  // FLAGS_logtostderr = 1;

  ros::init(argc, argv, "svo");
}

SvoNodeBase::SvoNodeBase() : node_handle_(), private_node_handle_("~"), type_(
    vk::param<bool>(private_node_handle_, "pipeline_is_stereo", true) ?
        svo::PipelineType::kStereo : svo::PipelineType::kMono),
        svo_interface_(type_, node_handle_, private_node_handle_) // svo interface created here 
{

  // subscribe to imu, gps, image and start the associated threads, waiting for new measurements.
  if (svo_interface_.imu_handler_)
  {
    svo_interface_.subscribeImu();
  }
  else
  {
    LOG(FATAL) << "This version of SVO needs IMU!";
  }

  if (svo_interface_.use_global_measurements_)
  { // call back that starts reading all new GP measurements 
    if (svo_interface_.globalpositions_handler_)
    {
      svo_interface_.subscribeGlobalPositions();
    }
    else
    {
      LOG(FATAL) << "Asked to use gp measurements but handler is not available!";
    }
  }

  if (svo_interface_.use_map_registration_)
  {
    if (svo_interface_.globalpose_handler_)
    {
      svo_interface_.subscribeGlobalPose();
    }
    else
    {
      LOG(FATAL) << "Asked to use gp pose but handler is not available!";
    }
  }

  svo_interface_.subscribeImage();
  svo_interface_.subscribeRemoteKey();
}

void SvoNodeBase::run()
{
  ros::spin();
  SVO_INFO_STREAM("SVO quit");
  svo_interface_.quit_ = true;
  SVO_INFO_STREAM("SVO terminated.\n");
}

}  // namespace svo_ros
