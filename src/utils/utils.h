//
// Created by giuseppe on 05/06/18.
//

#ifndef SIMULATOR_UTILS_H
#define SIMULATOR_UTILS_H

#include <random>
#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"

// Clamping function
template<typename T>
T clamp(T min, T value, T max)
{
    if(value > max)
        return max;

    if(value < min)
        return min;

    return value;

}


// Normal distribution class
class NormalDistribution
{
    std::default_random_engine generator_;
    std::normal_distribution<double> dist_;
    double sample_;

public:

    NormalDistribution();

    NormalDistribution(double , double );

    double draw();

};

//Node class
class RosNode {

public:

    ros::NodeHandle node_;

    RosNode() {};

    template<typename T>
    void setParamWithVerb(std::string param_name, T &var)
    {
        if (node_.getParam(param_name, var))
            std::cout << "Setting [" << param_name << "]" << std::endl;
        else
            ROS_ERROR("Failed to get parameter from server.");

    };

};


double PoseStampedToYaw(const geometry_msgs::PoseStamped::ConstPtr &);

#endif //SIMULATOR_UTILS_H
