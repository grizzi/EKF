//
// Created by giuseppe on 05/06/18.
//

#include <iostream>
#include "simulation_node.h"

SimulationNode::SimulationNode(ros::NodeHandle nh, std::string node_name)
{
    node_ = nh;
    boat_pub = node_.advertise<geometry_msgs::PoseStamped>("boat_position", 1000);
    control_sub_ = node_.subscribe("boat_status", 1000, &SimulationNode::boatCallback, this);

    setParamWithVerb(node_name + "/sim_param/Ts", Ts);
    setParamWithVerb(node_name + "/boat_param/longitudinal_drag", longitudinal_drag);
    setParamWithVerb(node_name + "/boat_param/lateral_drag", lateral_drag);
    setParamWithVerb(node_name + "/boat_param/longitudinal_noise_var", longitudinal_var);
    setParamWithVerb(node_name + "/boat_param/lateral_noise_var", lateral_var);

    /* Set model params */
    State ic = State(5,0);
    boat = Boat(ic);
    boat.setLongDrag(longitudinal_drag);
    boat.setLatDrag(lateral_drag);
    boat.setLongNoiseVar(longitudinal_var);
    boat.setLatNoiseVar(lateral_var);
}

void SimulationNode::publishPoseStamped()
{

    boat_pose.header.frame_id = "my_frame";
    boat_pose.header.stamp = ros::Time::now();
    boat_pose.header.seq = 0;

    tf::Quaternion q = tf::createQuaternionFromYaw(boat.getPos(BoatStates::PHI));
    boat_pose.pose.position.x = boat.getPos(BoatStates::X);
    boat_pose.pose.position.y = boat.getPos(BoatStates::Y);
    boat_pose.pose.position.z = 0.0;
    boat_pose.pose.orientation.x = q.getX();
    boat_pose.pose.orientation.y = q.getY();
    boat_pose.pose.orientation.z = q.getZ();
    boat_pose.pose.orientation.w = q.getW();

    boat_pub.publish(boat_pose);
}

double SimulationNode::getSamplingTime(){ return Ts; };

void SimulationNode::Step(double dt) { boat.step(dt); };

/* Subscribe to the control input and set the corresponding inputs*/
void SimulationNode::boatCallback(const ekf_messages::MeasAndControlStamped::ConstPtr msg)
{
    double thrust = msg->input.thrust;
    double rudder = msg->input.rudder;

    boat.setThrust(thrust);
    boat.setRudder(rudder);
}


// Ros loop
int main(int argc, char **argv)
{
    std::string node_name = "boat_talker";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    /* Initialize the boat node */
    SimulationNode bn(nh, node_name);

    /* For realistic simulation run at a rate close to the sampling time*/
    double Ts = bn.getSamplingTime();
    ros::Rate loop_rate(1/Ts);

    /* Start simulation */
    ros::Time timer = ros::Time::now();
    loop_rate.sleep();
    while (ros::ok())
    {
        ros::Time now = ros::Time::now();
        double dt = now.toSec() - timer.toSec();
        timer = now;

        // Execute one step
        bn.Step(dt);

        // Advertise position
        bn.publishPoseStamped();

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


