//
// Created by giuseppe on 11/07/18.
//

#include "ros/ros.h"
#include "estimator_node.h"

/* Constructor */
HKFNode::HKFNode(ros::NodeHandle nh, std::string node_name)
{
    node_ = nh;

    est_pub_ = node_.advertise<ekf_messages::EstimateStamped>("boat_estimates", 1000);
    est_sub_ = node_.subscribe("boat_status", 1000, &HKFNode::estCallback, this);

    // Sampling time
    double Ts;

    // Model parameters
    double Cd, Cr;
    double R0, phi0;
    double Qd, Qr;

    // Sensor parameters : these are all std deviations (thus variance = std^2)
    double sigma_a;
    double sigma_b;
    double sigma_c;
    double sigma_g;
    double sigma_n;
    double Qb;                              // drift dynamics noise std

    // Sensors location
    std::vector<double> pa{};
    std::vector<double> pb{};
    std::vector<double> pc{};

    // Parse parameters
    setParamWithVerb(node_name + "/sim_param/Ts", Ts);
    setParamWithVerb(node_name + "/boat_param/longitudinal_drag", Cd);
    setParamWithVerb(node_name + "/boat_param/lateral_drag", Cr);
    setParamWithVerb(node_name + "/est_param/R0", R0);
    setParamWithVerb(node_name + "/est_param/phi0", phi0);
    setParamWithVerb(node_name + "/boat_param/longitudinal_noise_var", Qd);
    setParamWithVerb(node_name + "/boat_param/lateral_noise_var", Qr);
    setParamWithVerb(node_name + "/meas_param/sigma_a", sigma_a);
    setParamWithVerb(node_name + "/meas_param/sigma_b", sigma_b);
    setParamWithVerb(node_name + "/meas_param/sigma_c", sigma_c);
    setParamWithVerb(node_name + "/meas_param/sigma_g", sigma_g);
    setParamWithVerb(node_name + "/meas_param/sigma_n", sigma_n);
    setParamWithVerb(node_name + "/meas_param/drift_stdev", Qb);
    setParamWithVerb(node_name + "/meas_param/pos_a", pa);
    setParamWithVerb(node_name + "/meas_param/pos_b", pb);
    setParamWithVerb(node_name + "/meas_param/pos_c", pc);

    // Initialize the internal timer
    timer_ = ros::Time::now().toSec();
}

void HKFNode::Publish()
{
    est_pub_.publish(est_output_);
}

double HKFNode::getSamplingTime(){ return Ts; };

void HKFNode::estCallback(const ekf_messages::MeasAndControlStamped::ConstPtr& msg)
{
    // TODO implement
}

// Ros node loop
int main(int argc, char **argv)
{
    std::string node_name = "measurement_node";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    /* Initialize the sensor node */
    HKFNode en(nh, node_name);

    double Ts = en.getSamplingTime();
    ros::Rate loop_rate(1/Ts);

    /* Start simulation */
    loop_rate.sleep();
    while (ros::ok())
    {
        // Publish measurement
        en.Publish();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
