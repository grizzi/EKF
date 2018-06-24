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

// Uniform Distribution class
UniformDistribution::UniformDistribution() {};
UniformDistribution::UniformDistribution(double left, double right)
{
    left_limit = left;
    right_limit = right;

    assert(left_limit <= right_limit);
    dist_ = std::uniform_real_distribution<> (left_limit, right_limit);

}

double UniformDistribution::draw() { return dist_(generator_); }

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

double deg2rad(double deg_value)
{
    return deg_value * M_PI / 180.0;
};


