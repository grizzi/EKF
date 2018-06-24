//
// Created by giuseppe on 24/06/18.
//

#ifndef EKF_VISUALIZATION_NODE_H
#define EKF_VISUALIZATION_NODE_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>


class BoatMarker
{

    visualization_msgs::Marker marker;
    ros::Publisher marker_pub;                       // publish marker
    ros::Subscriber marker_sub;                      // subsrcibe to stamped pose
    uint32_t  shape;

public:
    BoatMarker(ros::NodeHandle nh);
    void visualizerCallback(const geometry_msgs::PoseStamped::ConstPtr& );
    void Publish();

};

#endif //EKF_VISUALIZATION_NODE_H
