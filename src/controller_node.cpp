//
// Created by giuseppe on 09/06/18.
//

#include "ros/ros.h"
#include "controller_node.h"

/* Constructor */
ControllerNode::ControllerNode(ros::NodeHandle nh, std::string node_name)
{
    node_ = nh;
    control_pub_ = node_.advertise<ekf_messages::MeasAndControlStamped>("boat_status", 1000);
    measurement_sub_ = node_.subscribe("boat_measurements", 1000, &ControllerNode::controlCallback, this);

    // Parse control params
    setParamWithVerb(node_name + "/control_param/Ts", Ts);
    setParamWithVerb(node_name + "/control_param/max_thrust", max_thrust);
    setParamWithVerb(node_name + "/control_param/max_rudder_angle", max_rudder_angle);
    setParamWithVerb(node_name + "/control_param/thrust_input_type", thrust_input_type);
    setParamWithVerb(node_name + "/control_param/rudder_input_type", rudder_input_type);

    // Initialize controllers
    thrust_controller.setType(thrust_input_type);
    rudder_controller.setType(rudder_input_type);
    thrust_controller.setAmplitude(max_thrust);
    rudder_controller.setAmplitude(deg2rad(max_rudder_angle));

}

double ControllerNode::getSamplingTime(){ return Ts;}


void ControllerNode::controlCallback(const ekf_messages::Measurement::ConstPtr msg)
{
    status.header.stamp = ros::Time::now();
    status.meas = *msg;

    /* Compute control */
    double t = ros::Time::now().toSec();
    status.input.thrust = thrust_controller.getNext(t);
    status.input.rudder = rudder_controller.getNext(t);

}

void ControllerNode::Publish()
{
    control_pub_.publish(status);
}


// Ros loop
int main(int argc, char **argv)
{
    std::string node_name = "controller_node";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    /* Initialize the sensor node */
    ControllerNode cn(nh, node_name);

    /* For realistic simulation run at a rate equal to the sampling time*/
    double Ts = cn.getSamplingTime();
    ros::Rate loop_rate(1/Ts);

    /* Start control loop */
    loop_rate.sleep();
    while (ros::ok())
    {
        /* Publish the status message (measurement[k] + control[k]) */
        cn.Publish();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

