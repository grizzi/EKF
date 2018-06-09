//
// Created by giuseppe on 08/06/18.
//

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "geometry_msgs/PoseStamped.h"


#include "BoatMeasurements.h"
#include "utils.h"

class measNode : public RosNode
{
    public:
    ros::Publisher meas_pub_;                            // publish the measurements
    ros::Subscriber meas_sub_;                           // subscribe to the state

    BoatMeasurements sensors_;
    std_msgs::Float64MultiArray sens_data;

    /* Params */
    double Ts;

    /* Callback declaration */
    void measCallback(const geometry_msgs::PoseStamped::ConstPtr& );

    /* Constructor */
    measNode(ros::NodeHandle nh, std::string node_name)
    {
        node_ = nh;
        meas_pub_ = node_.advertise<std_msgs::Float64MultiArray>("boat_measurements", 1000);
        meas_sub_ = node_.subscribe("boat_position", 1000, &measNode::measCallback, this);

        // Initialize sensor parameters
        double sigma_a;
        double sigma_b;
        double sigma_c;
        double sigma_g;
        double sigma_n;
        double Qb;             // drift dynamics noise variance

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
        setParamWithVerb(node_name + "/meas_param/pos_a", pa);
        setParamWithVerb(node_name + "/meas_param/pos_b", pb);
        setParamWithVerb(node_name + "/meas_param/pos_c", pc);

        sensors_ = BoatMeasurements(sigma_a, sigma_b, sigma_c, sigma_g, sigma_n, Qb, pa, pb, pc);

        // Add one dimension to the vector of measurements
        sens_data.data.clear();

    }

    double getSamplingTime(){ return Ts;}

    void Publish()
    {
        meas_pub_.publish(sens_data);
    }


};


void measNode::measCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    double yaw = PoseStampedToYaw(msg);
    sensors_.measure(msg->pose.position.x, msg->pose.position.y, yaw, Ts);
    std::array<double, 5> m = sensors_.getMeas();
    for(size_t i=0; i < 5; i++)
    {
        sens_data.data.push_back(m[i]);
    }
}

int main(int argc, char **argv)
{
    std::string node_name = "meas_talker";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    /* Initialize the sensor node */
    measNode mn(nh, node_name);

    /* For realistic simulation run at a rate close to the sa*/
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