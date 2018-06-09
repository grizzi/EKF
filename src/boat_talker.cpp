//
// Created by giuseppe on 05/06/18.
//

#include <iostream>
#include <sstream>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"

#include "BoatModel.h"
#include "utils.h"

class boatNode: public RosNode
{
public:
    ros::Publisher boat_pub;
    geometry_msgs::PoseStamped boat_pose;

    /* Create object with 0 initial condition */
    State init_cond;
    Boat boat;

    /* Model params */
    double longitudinal_drag, lateral_drag;
    double longitudinal_var, lateral_var;
    double max_thrust, max_rudder_angle;
    double Ts;
    int thrust_input_type;
    int rudder_input_type;

    boatNode(ros::NodeHandle nh, std::string node_name)
    {
        node_ = nh;
        boat_pub = node_.advertise<geometry_msgs::PoseStamped>("boat_position", 1000);

        setParamWithVerb(node_name + "/boat_param/longitudinal_drag", longitudinal_drag);
        setParamWithVerb(node_name + "/boat_param/lateral_drag", lateral_drag);
        setParamWithVerb(node_name + "/boat_param/longitudinal_noise_var", longitudinal_var);
        setParamWithVerb(node_name + "/boat_param/lateral_noise_var", lateral_var);
        setParamWithVerb(node_name + "/boat_param/max_thrust", max_thrust);
        setParamWithVerb(node_name + "/boat_param/max_rudder_angle", max_rudder_angle);
        setParamWithVerb(node_name + "/boat_param/thrust_input_type", thrust_input_type);
        setParamWithVerb(node_name + "/boat_param/rudder_input_type", rudder_input_type);
        setParamWithVerb(node_name + "/sim_param/Ts", Ts);

        /* Set model params */
        State ic = State(5,0);
        boat = Boat(ic);
        boat.setLongDrag(longitudinal_drag);
        boat.setLatDrag(lateral_drag);
        boat.setLongNoiseVar(longitudinal_var);
        boat.setLatNoiseVar(lateral_var);
        boat.setThrustInputType(thrust_input_type);
        boat.setRudderInputType(rudder_input_type);
        boat.setMaxThrust(max_thrust);
        boat.setMaxRudder(max_rudder_angle);
    }

    void publishPoseStamped()
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

};

int main(int argc, char **argv)
{
    std::string node_name = "boat_talker";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    /* For realistic simulation run at a rate close to the sa*/
    ros::Rate loop_rate(50);

    boatNode bn(nh, node_name);

    /* Start simulation */
    ros::Time timer = ros::Time::now();
    loop_rate.sleep();
    while (ros::ok())
    {
        ros::Time now = ros::Time::now();
        double dt = now.toSec() - timer.toSec();
        timer = now;

        // Execute one step
        bn.boat.step(dt);

        // Advertise position
        bn.publishPoseStamped();

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}


