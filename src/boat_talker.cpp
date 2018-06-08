//
// Created by giuseppe on 05/06/18.
//

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"

#include <iostream>
#include "BoatModel.h"


#include <sstream>

class boatNode
{
public:
    ros::NodeHandle boat_node_;
    ros::Publisher boat_pub;
    geometry_msgs::Pose2D boat_pose;

    /* Create object with 0 initial condition */
    State init_cond;
    Boat boat;

    /* Model params */
    double longitudinal_drag, lateral_drag;
    double longitudinal_var, lateral_var;
    double max_thrust, max_rudder_angle;
    int thrust_input_type;
    int rudder_input_type;

    boatNode(ros::NodeHandle nh, std::string node_name)
    {
        boat_node_ = nh;
        boat_pub = boat_node_.advertise<geometry_msgs::Pose2D>("boat_position", 1000);

        setParamWithVerb(node_name + "/boat_param/longitudinal_drag", longitudinal_drag);
        setParamWithVerb(node_name + "/boat_param/lateral_drag", lateral_drag);
        setParamWithVerb(node_name + "/boat_param/longitudinal_noise_var", longitudinal_var);
        setParamWithVerb(node_name + "/boat_param/lateral_noise_var", lateral_var);
        setParamWithVerb(node_name + "/boat_param/max_thrust", max_thrust);
        setParamWithVerb(node_name + "/boat_param/max_rudder_angle", max_rudder_angle);
        setParamWithVerb(node_name + "/boat_param/thrust_input_type", thrust_input_type);
        setParamWithVerb(node_name + "/boat_param/rudder_input_type", rudder_input_type);

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

    template <typename  T>
    void setParamWithVerb(std::string param_name, T &var)
    {
        if(boat_node_.getParam(param_name, var))
            std::cout << "Setting ["<< param_name << "] to " << var << std::endl;
        else
            std::cout << "Retrieval of [" << param_name << "] failed" << std::endl;

    }
};

int main(int argc, char **argv)
{
    std::string node_name = "boat_talker";

    ros::init(argc, argv, node_name);

    ros::NodeHandle nh;
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
        bn.boat_pose.x = bn.boat.getPos(BoatStates::X);
        bn.boat_pose.y = bn.boat.getPos(BoatStates::Y);
        bn.boat_pose.theta = bn.boat.getPos(BoatStates::PHI);

        bn.boat_pub.publish(bn.boat_pose);

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}


