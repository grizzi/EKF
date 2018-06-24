//
// Created by giuseppe on 24/06/18.
//

#ifndef EKF_SIMULATION_NODE_H
#define EKF_SIMULATION_NODE_H

#include <iostream>
#include <sstream>

#include "ros/ros.h"
#include "tf/tf.h"
#include "geometry_msgs/PoseStamped.h"
#include <ekf_messages/MeasAndControlStamped.h>

#include "BoatModel.h"
#include "utils.h"

class SimulationNode: public RosNode
{
    /* Publish the pose*/
    ros::Publisher boat_pub;
    geometry_msgs::PoseStamped boat_pose;

    /* Subscribe to control input*/
    ros::Subscriber control_sub_;

    /* Create object with 0 initial condition */
    State init_cond;
    Boat boat;

    /* Model params */
    double Ts;                                      // simulation sampling time
    double longitudinal_drag, lateral_drag;
    double longitudinal_var, lateral_var;

public:
    SimulationNode(ros::NodeHandle nh, std::string node_name);                // Constructor
    void boatCallback(const ekf_messages::MeasAndControlStamped::ConstPtr);   // Callback for reading and setting controller input
    void publishPoseStamped();                                                // Extract and publish pose
    double getSamplingTime();                                                 // Retrieve Ts
    void Step(double  dt);                                                    // Execute one step of the ct dynamics

};
#endif //EKF_SIMULATION_NODE_H
