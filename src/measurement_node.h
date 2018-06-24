//
// Created by giuseppe on 24/06/18.
//

#ifndef EKF_MEASUREMENT_NODE_H
#define EKF_MEASUREMENT_NODE_H


#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "geometry_msgs/PoseStamped.h"

#include "BoatMeasurements.h"
#include "utils.h"

#include "ekf_messages/Measurement.h"

class MeasNode : public RosNode
{
private:
    ros::Publisher meas_pub_;                            // publish the measurements
    ros::Subscriber meas_sub_;                           // subscribe to the state
    double timer_;                                       // internal timer
    double Ts;                                           // publishing rate, same as simulation Ts

    BoatMeasurements sensors_;                           // Sensors model
    ekf_messages::Measurement sens_data;                 // Sensor measurements data structure

public:
    MeasNode(ros::NodeHandle nh, std::string node_name);                    // Constructor
    void measCallback(const geometry_msgs::PoseStamped::ConstPtr& );        // Callback to fill measurement message from pose
    void Publish();                                                         // Publisher wrapper
    double getSamplingTime();                                               // Retrieve Ts

};


#endif //EKF_MEASUREMENT_NODE_H
