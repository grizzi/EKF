//
// Created by giuseppe on 24/06/18.
//

#ifndef EKF_CONTROLLER_NODE_H
#define EKF_CONTROLLER_NODE_H

#include "ros/ros.h"

// Custom messages
#include "ekf_messages/Measurement.h"
#include "ekf_messages/Control.h"
#include "ekf_messages/MeasAndControlStamped.h"

#include "Control.h"
#include "utils.h"

class ControllerNode : public RosNode
{
private:
    ros::Publisher control_pub_;                            // publish the control input
    ros::Subscriber measurement_sub_;                       // subscribe to measurements
    Controller thrust_controller;                           // thrust controller
    Controller rudder_controller;                           // rudder controller

    /* Params */
    double Ts;
    double max_thrust, max_rudder_angle;
    int thrust_input_type;
    int rudder_input_type;

    /* Status Array */
    ekf_messages::MeasAndControlStamped status;

public:

    ControllerNode(ros::NodeHandle nh, std::string node_name);          // constructor
    void controlCallback(const ekf_messages::Measurement::ConstPtr);    // control callback
    void Publish();                                                     // Publish
    double getSamplingTime();                                           // retrieve Ts


};


#endif //EKF_CONTROLLER_NODE_H
