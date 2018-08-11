//
// Created by giuseppe on 11/07/18.
//

#include "ros/ros.h"
#include "estimator_node.h"

/* Constructor */
HKFNode::HKFNode(ros::NodeHandle nh, std::string node_name)
{
    std::cout << "Initializing the Estimator node" << std::endl;

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
    setParamWithVerb(node_name + "/control_param/Ts", Ts);
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

    std::cout << "Before estimator initialization" << std::endl;
    // Initialize the estimator
    State in_cond(6, 0);
    Estimator_ = HKF(Cd, Cr, R0, phi0, Qd, Qr, Qb, pa[0], pa[1], pb[0], pb[1],
            pc[0], pc[1], sigma_a*sigma_a, sigma_b*sigma_b, sigma_c*sigma_c,
                     sigma_g*sigma_g,
            sigma_n*sigma_n, Ts, in_cond);
    std::cout << "After estimator initialization" << std::endl;

    // Initialize the internal timer
    timer_ = ros::Time::now().toSec();
}

void HKFNode::Publish()
{

    // Read estimates values
    est_output_.header.stamp = ros::Time::now();
    est_output_.px.value = Estimator_.getState(0);
    est_output_.py.value = Estimator_.getState(1);
    est_output_.vx.value = Estimator_.getState(2);
    est_output_.vy.value = Estimator_.getState(3);
    est_output_.orientation.value = Estimator_.getState(4);
    est_output_.drift.value = Estimator_.getState(5);

    // Read estimates variances
    est_output_.px.variance = Estimator_.getStateVar(0);
    est_output_.py.variance = Estimator_.getStateVar(1);
    est_output_.vx.variance = Estimator_.getStateVar(2);
    est_output_.vy.variance = Estimator_.getStateVar(3);
    est_output_.orientation.variance = Estimator_.getStateVar(4);
    est_output_.drift.variance = Estimator_.getStateVar(5);


    est_pub_.publish(est_output_);
}

double HKFNode::getSamplingTime(){ return Ts; };

void HKFNode::estCallback(const ekf_messages::MeasAndControlStamped::ConstPtr& msg)
{
    //std::cout << "Entered the estimator callback" << std::endl;
    // The callback consists in the estimator step = prior -> posterior update
    // the necessary inputs are the previous system inputs and the current measurement
    Estimator_.priorUpdate();
    // std::cout << "OK PRIOR UPDATE " << std::endl;

    // Update executed according to previously stored input, store the new commanded thrust and rudder angle
    Estimator_.setThrust(msg->input.thrust);
    Estimator_.setRudder(msg->input.rudder);
    //std::cout << "OK SETTING NEW INPUT " << std::endl;

    // Fill the measurement vector for the posterior update
    if(msg->meas.dist_c==-1) {
        // std::cout << "Estimator not using measurement from c";
        measurement_vector.resize(4);
        measurement_vector << msg->meas.dist_a, msg->meas.dist_b,
                              msg->meas.angle_drift, msg->meas.angle_compass;
    }
    else
    {
        measurement_vector.resize(5);
        measurement_vector << msg->meas.dist_a , msg->meas.dist_b, msg->meas.dist_c,
                              msg->meas.angle_drift, msg->meas.angle_compass;
    }

    // Posterior update
    Estimator_.posteriorUpdate(measurement_vector);
}

// Ros node loop
int main(int argc, char **argv)
{
    std::string node_name = "estimator_node";
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
