#include "gnss.hpp"
#include "receiver.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "GNSSSimulator");
    ros::NodeHandle nh("~");

    std::string constellation_file;
    std::string config_path;

    nh.getParam("constellation_file", constellation_file);
    nh.getParam("config_path", config_path);

    YAML::Node config = YAML::LoadFile(config_path);
    double max_time_before_reset = config["max_time_before_resetting_error"].as<double>();
    double max_distance_before_reset = config["max_distance_before_resetting_error"].as<double>();
    std::string ground_truth_topic = config["ground_truth_topic"].as<std::string>();

    ros::Rate loop_rate(5);
    ros::Time last_called = ros::Time::now();

    GNSS gnss(constellation_file,config_path, nh);
    gnss.startAllEmitters(ground_truth_topic);

    Receiver receiver(gnss, config_path);
    Eigen::Vector3d old_position = gnss.receiver_enu_gt;
    bool reset_pseusorange_errors = true;

    while (ros::ok()) {
        ros::Time current_time = ros::Time::now(); 
        ros::Duration elapsed = current_time - last_called;

        receiver.setReceiverECEF(gnss.receiver_ecef_gt);
        receiver.setReceiverENU(gnss.receiver_enu_gt);

        if ((old_position - gnss.receiver_enu_gt).norm() > max_distance_before_reset) {
            reset_pseusorange_errors = true; 
            std::cout << "Resetting random values due to distance" << std::endl;
            old_position = gnss.receiver_enu_gt;
        }

        if ((elapsed.toSec() >= max_time_before_reset) || reset_pseusorange_errors) {
            reset_pseusorange_errors = false;
            receiver.generateRandomDoubles();
            last_called = current_time;
        }

        receiver.trilateratesatellites(gnss);
        receiver.publish();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

