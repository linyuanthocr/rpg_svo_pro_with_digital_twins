#ifndef RECEIVER_HPP
#define RECEIVER_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Int32.h>
#include "satellite_simulator/DoubleMsg.h"
#include "satellite_simulator/EmitterMsg.h"
#include "satellite_simulator/DoubleArrayMsg.h"
#include "satellite_simulator/IntMsg.h"
#include "constellation.hpp"
#include "emitter.hpp"
#include "frame_handler.hpp"

#include <Eigen/Dense>
#include <GeographicLib/LocalCartesian.hpp>
#include <yaml-cpp/yaml.h>
#include <ceres/ceres.h>

#include <vector>
#include <mutex>
#include <random>
#include <string>
#include <iostream>
#include <iomanip>
#include <thread>

// Cost function for Ceres optimization
struct CostFunctor {
    CostFunctor(const Eigen::Vector3d& a_n, double d_n)
        : a_n_(a_n.data(), a_n.data() + a_n.size()), d_n_(d_n) {}

    template <typename T>
    bool operator()(const T* const x, T* residual) const {
        residual[0] = ((T(a_n_[0]) - x[0]) * (T(a_n_[0]) - x[0]) +
                       (T(a_n_[1]) - x[1]) * (T(a_n_[1]) - x[1]) +
                       (T(a_n_[2]) - x[2]) * (T(a_n_[2]) - x[2]) -
                       T(d_n_) * T(d_n_));
        return true;
    }

private:
    const std::vector<double> a_n_;
    const double d_n_;
};


class Receiver {
public:
    Receiver(GNSS &gnss, const std::string& config_path)
        : gen(rd()), d(0, 2) {

        YAML::Node config = YAML::LoadFile(config_path);

        pseudorange_error_topic = config["pseudorange_error_topic"].as<std::string>();
        number_of_satellite_topic = config["number_of_satellite_topic"].as<std::string>();
        ground_truth_topic = config["ground_truth_topic"].as<std::string>();

        lla_receiver_pub = nh_.advertise<geometry_msgs::PointStamped>("/gnss_simulator/LLA", 1);
        ecef_receiver_pub = nh_.advertise<geometry_msgs::PointStamped>("/gnss_simulator/ECEF", 1);
        enu_receiver_pub = nh_.advertise<geometry_msgs::PoseStamped>("/gnss_simulator/ENU", 1);

        reciever_number_of_visible_satellites = nh_.subscribe<satellite_simulator::IntMsg>(number_of_satellite_topic, 1, &Receiver::N_Sat_Callback, this);
        reciever_multipath_errors = nh_.subscribe(pseudorange_error_topic, 1, &Receiver::MP_Callback, this);
        reciever_groundtruth = nh_.subscribe(ground_truth_topic, 1, &Receiver::GT_Callback, this);

        receiver_enu_gt = Eigen::Vector3d::Zero();
        receiver_ecef_gt = Eigen::Vector3d::Zero();
        running = true;

        satellites_range_errors.resize(gnss.emitters.size());
        satellites_range_errors_reciever.resize(gnss.emitters.size());
    }

    ~Receiver() {};

    void startReceiver();

    void N_Sat_Callback(const satellite_simulator::IntMsg::ConstPtr& msg) {
        number_satellites_line_of_sight = msg->data.data;
    }

    void GT_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        groundtruth_enu_frame = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z); 
    }


    void MP_Callback(const satellite_simulator::DoubleArrayMsg::ConstPtr& msg) {
        std::lock_guard<std::mutex> guard(data_mtx);
        satellites_range_errors_reciever.resize(satellites_range_errors.size()); // listens to the pseudorange generates by the python script 
        for (int i = 0; i < satellites_range_errors.size(); i++) 
            satellites_range_errors_reciever[i] = msg->data[i];
    }

    void generateRandomDoubles() {
        std::lock_guard<std::mutex> guard(data_mtx);

        satellites_range_errors = satellites_range_errors_reciever;
        number_of_recieved_satellites = number_satellites_line_of_sight;       

    }

    void setReceiverENU(const Eigen::Vector3d& receiver_enu_gt)
    {
        this->receiver_enu_gt = receiver_enu_gt;
    }

    void setReceiverECEF(const Eigen::Vector3d& receiver_ecef_gt_)
    {
        receiver_ecef_gt = receiver_ecef_gt_;
    }

    void trilateratesatellites(GNSS &gnss) {
        std::lock_guard<std::mutex> guard(data_mtx);

        int k =0;

        if(number_of_recieved_satellites>3) {

            receiver_enu_estimate = gnss.receiver_enu_gt;
            double ecefX, ecefY, ecefZ;
            ENUtoECEF(groundtruth_enu_frame.x(), groundtruth_enu_frame.y(), groundtruth_enu_frame.z(), gnss.Linearization_Point(0), gnss.Linearization_Point(1), gnss.Linearization_Point(2), ecefX, ecefY, ecefZ);
            std::vector<double> x = {ecefX, ecefY, ecefZ};
            ceres::Problem problem;
            k=0;
            for (int i = 0; i < gnss.emitters.size(); i++) {
                Emitter& e = gnss.emitters[i];

                if((e.above_horizon) && (k < number_of_recieved_satellites))
                {
                    k++; 
                    double corrupt_pseudo = e.pseudorange + satellites_range_errors[i]; //+ another d, white noise.
                    ceres::CostFunction *cost_function =
                        new ceres::AutoDiffCostFunction<CostFunctor, 1, 3>(
                            new CostFunctor(e.Position_ECEF, corrupt_pseudo));

                            ceres::LossFunction* loss_function = new ceres::HuberLoss(100.0);
                    problem.AddResidualBlock(cost_function, loss_function, x.data());
                } 

            }

            ceres::Solver::Options options;
            options.minimizer_progress_to_stdout = false;
            options.linear_solver_type = ceres::DENSE_QR; 
            options.max_num_iterations = 300; 
            ceres::Solver::Summary summary;

            ceres::Solve(options, &problem, &summary);

            receiver_ecef_estimate = Eigen::Vector3d(x[0], x[1], x[2]);
            double lat, lon, alt; 
            ECEFToLLA(receiver_ecef_estimate.x(), receiver_ecef_estimate.y(), receiver_ecef_estimate.z(), lat, lon, alt); 
            receiver_lla_estimate = Eigen::Vector3d(lat, lon, alt);
            double e,n,u;
            LLAtoENU(lat, lon, alt, gnss.Linearization_Point(0), gnss.Linearization_Point(1), gnss.Linearization_Point(2), e, n, u);
            receiver_enu_estimate = Eigen::Vector3d(e,n,u);

            if(receiver_enu_estimate(2)>200) {
                k=0;
                for (int i = 0; i < gnss.emitters.size(); i++) {
                    Emitter& e = gnss.emitters[i];
                    if((e.above_horizon) && (k < number_of_recieved_satellites))
                        k++; 
                }
            }
        }
    }

    void publish() {
        if(number_of_recieved_satellites>3) {
            // Publish the receiver position (ECEF, ENU, LLA
            geometry_msgs::PointStamped msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "ecef";
            msg.point.x = receiver_ecef_estimate(0);
            msg.point.y = receiver_ecef_estimate(1);
            msg.point.z = receiver_ecef_estimate(2);
            ecef_receiver_pub.publish(msg);

            geometry_msgs::PoseStamped enu_msg;
            enu_msg.header.frame_id = "world";
            enu_msg.header.stamp = ros::Time::now();

            enu_msg.pose.position.x = receiver_enu_estimate(0);
            enu_msg.pose.position.y = receiver_enu_estimate(1);
            enu_msg.pose.position.z = receiver_enu_estimate(2);
            enu_msg.pose.orientation.w = 1 ;// receiver_enu_estimate(2);
            enu_receiver_pub.publish(enu_msg);

            msg.header.frame_id = "geo";
            msg.point.x = receiver_lla_estimate(0);
            msg.point.y = receiver_lla_estimate(1);
            msg.point.z = receiver_lla_estimate(2);
            lla_receiver_pub.publish(msg);

        }

    }

private:
    ros::NodeHandle nh_;
    std::string pseudorange_error_topic;
    std::string number_of_satellite_topic;
    std::string ground_truth_topic;

    Eigen::Vector3d receiver_enu_gt;
    Eigen::Vector3d receiver_ecef_gt;

    std::vector<double> satellites_range_errors_reciever;
    std::vector<double> satellites_range_errors;

    ros::Publisher enu_receiver_pub;
    ros::Publisher ecef_receiver_pub;
    ros::Publisher lla_receiver_pub;

    ros::Subscriber reciever_number_of_visible_satellites;
    ros::Subscriber reciever_multipath_errors;
    ros::Subscriber reciever_groundtruth;

    Eigen::Vector3d groundtruth_enu_frame;


    Eigen::Vector3d receiver_ecef_estimate;
    Eigen::Vector3d receiver_enu_estimate;
    Eigen::Vector3d receiver_lla_estimate;


    int number_satellites_line_of_sight;
    int number_of_recieved_satellites;

    std::mutex data_mtx;

    std::random_device rd;
    std::mt19937 gen;
    std::normal_distribution<> d;

    bool running;
};

#endif // RECEIVER_HPP


