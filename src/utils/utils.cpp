//
// Created by giuseppe on 09/06/18.
//

#include "utils.h"

#include <random>
#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"



// Normal distribution class
NormalDistribution::NormalDistribution() {};

NormalDistribution::NormalDistribution(double mean, double var)
{
    dist_ = std::normal_distribution<double>(mean, var);
}

double NormalDistribution::draw(){ return dist_(generator_); };



double PoseStampedToYaw(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    tf::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
};

