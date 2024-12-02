#ifndef CONSTELLATION_H
#define CONSTELLATION_H

#include <vector>
#include <cmath>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include <Eigen/Dense>
#include "satellite_param.hpp"
#include <thread>
#include <cmath>
#include <sstream>

class Constellation {
public:
    Constellation(const std::string& filename, ros::NodeHandle& nh);
    ~Constellation();

    void startAllSatellites();
    void stopAllSatellites();

private:
    std::thread processing_thread;

    std::mutex mtx;
    
    bool running;

    void processingLoop();
};

#endif // CONSTELLATION_H
