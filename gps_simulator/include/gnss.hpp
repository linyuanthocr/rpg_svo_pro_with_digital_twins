#ifndef GPS_SIM_HPP
#define GPS_SIM_HPP

#include <ros/ros.h>
#include "constellation.hpp"
#include <Eigen/Dense>
#include <vector>
#include <array>
#include <iostream>
#include <ceres/ceres.h>
#include <fstream>
#include <string>
#include <limits>
#include <geometry_msgs/PoseStamped.h>         
#include <geometry_msgs/PointStamped.h>         
#include <GeographicLib/LocalCartesian.hpp>
#include <yaml-cpp/yaml.h>
#include <thread>
#include "frame_handler.hpp"

class Emitter; 

class GNSS {
public:
    GNSS(const std::string& constallation_config_fn, const std::string& reciever_config_fn, ros::NodeHandle& nh) ;

    ~GNSS();

    void startAllEmitters(std::string ground_truth_topic);

    Eigen::Vector3d receiver_enu_gt;
    Eigen::Vector3d receiver_ecef_gt;

    int number_visible_satellites;
    std::vector<Emitter> emitters; // TODO : map the PRN to the emitters/satellite for more clarity
    
    Eigen::Vector3d Linearization_Point;

private:
    ros::NodeHandle nh_; 
    
    Eigen::Vector3d Initial_ECEF;
    ros::Subscriber receiver_enu_gt_sub;
    ros::Publisher ecef_receiver_gt_pub;

    std::thread processing_thread;
    std::mutex emitters_mtx; 

    bool running;

    void EphemerisLoop();

    void groundtruthCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);   

};


#endif // GPS_SIM_HPP