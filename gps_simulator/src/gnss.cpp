
#include "gnss.hpp"
#include "emitter.hpp"  // Include the full definition of Emitter
#include "geometry_msgs/PointStamped.h"


GNSS::GNSS(const std::string& constallation_config_fn, const std::string& reciever_config_fn, ros::NodeHandle& nh) : nh_(nh) {

    ecef_receiver_gt_pub = nh.advertise<geometry_msgs::PointStamped>("/receiver_ecef_frame_gt", 1000);

    YAML::Node config = YAML::LoadFile(constallation_config_fn);
    int numSatellites = config["sqrt_sa"].size(); 

    Linearization_Point <<47.36667, 8.55, 0; // Zurich <3 -> the direction shouldnt matter
    std::cout << "LLA  linearization point : " << Linearization_Point(0) << " " << Linearization_Point(1) << " " << Linearization_Point(2) << std::endl;

    double x,y,z;
    LLAToECEF(Linearization_Point(0), Linearization_Point(1), Linearization_Point(2), x, y, z);
    Initial_ECEF << x,y,z;
    std::cout << "ECEF linearization point : " << Initial_ECEF << std::endl;

    for (int i = 0; i < numSatellites; i++) {
        SatelliteParam sat_param(constallation_config_fn, i);
        emitters.emplace_back(nh, ros::Time::now().toSec(), sat_param);
    }
}

GNSS::~GNSS() {}

void GNSS::startAllEmitters(std::string ground_truth_topic) { // start ephemeris loop

    running = true;
    processing_thread = std::thread(&GNSS::EphemerisLoop, this);
    receiver_enu_gt_sub = nh_.subscribe(ground_truth_topic, 1, &GNSS::groundtruthCallback, this);
}


Constellation::~Constellation() {

    running = false;

    if (processing_thread.joinable()) {
        processing_thread.join();
    }

}

void GNSS::EphemerisLoop() {
    ros::Rate rate(10); // 10 Hz
    while (running && ros::ok()) {
        emitters_mtx.lock();
        for (auto& emitter : emitters) {
            emitter.process();
        }
        emitters_mtx.unlock();
        ros::spinOnce();
        rate.sleep();
    }
}


void GNSS::groundtruthCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) { // we need to take into account roll and pitch for elevation and azimuth angles.
  

    // this is the receiver position in ENU, with 0 0 0 = linearization point being the origin of the map
    receiver_enu_gt = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z); 
    Eigen::Quaterniond rot_enu_receiver = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

    double x,y,z;
    ENUtoECEF(receiver_enu_gt.x(), receiver_enu_gt.y(),receiver_enu_gt.z(),
            Linearization_Point.x(), Linearization_Point.y(), Linearization_Point.z(), x, y, z); //Q : x = E, y = N, z = U ? ->should be okay at long as consistent (z up)
    receiver_ecef_gt = Eigen::Vector3d(x,y,z);

    // std::cout << "Receiver ECEF : " << receiver_ecef.transpose() << std::endl;

    geometry_msgs::PointStamped position_msg;
    position_msg.header.stamp =  ros::Time(msg->header.stamp);
    position_msg.header.frame_id = "ECEF";
    position_msg.point.x = receiver_ecef_gt.x();
    position_msg.point.y = receiver_ecef_gt.y();
    position_msg.point.z = receiver_ecef_gt.z();
    ecef_receiver_gt_pub.publish(position_msg);

    for (auto& emitter : emitters) {
        emitter.emit_pseudorange(receiver_ecef_gt, rot_enu_receiver);
    }

    //send it to emitters

    return;
}


