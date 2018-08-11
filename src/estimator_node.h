//
// Created by giuseppe on 11/07/18.
//

#ifndef EKF_ESTIMATOR_NODE_H
#define EKF_ESTIMATOR_NODE_H

#include "ros/ros.h"

#include "utils.h"
#include "HybridKalmanFilter.h"

#include "ekf_messages/MeasAndControlStamped.h"
#include "ekf_messages/EstimateStamped.h"

class HKFNode : public RosNode
{
private:
    ros::Publisher est_pub_;                             // publish the estimate and corresponding variance
    ros::Subscriber est_sub_;                            // subscribe to the measurements and the control input
    double timer_;                                       // internal timer
    double Ts;                                           // publishing rate, same as simulation Ts

    // Estimator
    HKF Estimator_;
    VectorXd measurement_vector;

    ekf_messages::MeasAndControlStamped est_input_;      // Sensor measurements and control inputs
    ekf_messages::EstimateStamped est_output_;           // Estimated state values and corresponding variance

public:
    HKFNode(ros::NodeHandle nh, std::string node_name);                     // Constructor
    void estCallback(const ekf_messages::MeasAndControlStamped::ConstPtr&); // Callback upon reception measurements
    void Publish();                                                         // Publisher wrapper
    double getSamplingTime();                                               // Retrieve Ts

};

#endif //EKF_ESTIMATOR_NODE_H
