//
// Created by giuseppe on 08/06/18.
//


#include "ros/ros.h"
#include "measurement_node.h"

 /* Constructor */
MeasNode::MeasNode(ros::NodeHandle nh, std::string node_name)
{
    node_ = nh;

    meas_pub_ = node_.advertise<ekf_messages::Measurement>("boat_measurements", 1000);
    meas_sub_ = node_.subscribe("boat_position", 1000, &MeasNode::measCallback, this);

    // Initialize sensor parameters : these are all std deviations
    double sigma_a;
    double sigma_b;
    double sigma_c;
    double sigma_g;
    double sigma_n;
    double Qb;                              // drift dynamics noise std
    double loss_probability_c;              // not always measurements from c is available

    // Sensors location
    std::vector<double> pa{};
    std::vector<double> pb{};
    std::vector<double> pc{};

    // Parse parameters
    setParamWithVerb(node_name + "/sim_param/Ts", Ts);
    setParamWithVerb(node_name + "/meas_param/sigma_a", sigma_a);
    setParamWithVerb(node_name + "/meas_param/sigma_b", sigma_b);
    setParamWithVerb(node_name + "/meas_param/sigma_c", sigma_c);
    setParamWithVerb(node_name + "/meas_param/sigma_g", sigma_g);
    setParamWithVerb(node_name + "/meas_param/sigma_n", sigma_n);
    setParamWithVerb(node_name + "/meas_param/drift_stdev", Qb);
    setParamWithVerb(node_name + "/meas_param/loss_probability_c", loss_probability_c);
    setParamWithVerb(node_name + "/meas_param/pos_a", pa);
    setParamWithVerb(node_name + "/meas_param/pos_b", pb);
    setParamWithVerb(node_name + "/meas_param/pos_c", pc);

    // Initialize sensors
    sensors_ = BoatMeasurements(sigma_a, sigma_b, sigma_c, sigma_g, sigma_n, Qb, loss_probability_c, pa, pb, pc);

    // Initialize the internal timer
    timer_ = ros::Time::now().toSec();
}

void MeasNode::Publish()
{
    meas_pub_.publish(sens_data);
}

double MeasNode::getSamplingTime(){ return Ts; };

void MeasNode::measCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    double dt;      // dt for the integration of the sensor drift

    // Sync measurement with first position message
    if(msg->header.seq == 0)
        dt = 0.0;
    else
        dt = msg->header.stamp.toSec() - timer_;
    timer_ = msg->header.stamp.toSec();

    double yaw = PoseStampedToYaw(msg);
    sensors_.measure(msg->pose.position.x, msg->pose.position.y, yaw, dt);
    std::array<double, 5> m = sensors_.getMeas();

    sens_data.dist_a = m[0];
    sens_data.dist_b = m[1];
    sens_data.dist_c = m[2];
    sens_data.angle_drift = m[3];
    sens_data.angle_compass = m[4];

}

// Ros node loop
int main(int argc, char **argv)
{
    std::string node_name = "measurement_node";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    /* Initialize the sensor node */
    MeasNode mn(nh, node_name);

    double Ts = mn.getSamplingTime();
    ros::Rate loop_rate(1/Ts);

    /* Start simulation */
    loop_rate.sleep();
    while (ros::ok())
    {
        // Publish measurement
        mn.Publish();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

