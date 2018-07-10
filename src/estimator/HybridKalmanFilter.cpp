//
// Created by giuseppe on 10/07/18.
//

#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

class HKF
{
private:

    /* Define the matrices and vector used by the Hybrid Kalman Filter
    P_  - Estimate covariance Matrix
    A_  - Process state dynamics linearization matrix
    Qc_ - Process noise covariance matrix
    L_  - Process noise dynamics linearization matrix
    K_  - Kalman gain
    H_  - Measurement state dynamics linearization matrix
    M_  - Measurement noise dynamics linearization matrix
    */

    VectorXd state;
    MatrixXd P_, A_, Qc_, L_, K_, H_, M_;

public:
    void init(int num_states, int num_measurements);

    HKF();

    void updateMatrices()
    {
        A_(3,2) = -2*Cd_*state[2]*cos(state[4]);
        A_(3,3) = -2*Cd_*state[3]*cos(state[4]);
        A_(3,4) = -sin(state[4])*(tanh(u_prev) - Cd_(state[2]*state[2] + state[3]*state[3]));
        A_(4,2) = -2*Cd_*state[2]*sin(state[4]);
        A_(4,3) = -2*Cd_*state[3]*sin(state[4]);
        A_(4,4) = cos(state[4])*(tanh(u_prev) - Cd_(state[2]*state[2] + state[3]*state[3]));


    }

};

HKF::HKF()
{

}

void HKF::init(int num_states, int num_measurements)
{
    // Initialize matrices with zeros and values wherever they never change
    P_ = MatrixXd::Zero(num_states, num_states);

    A_ = MatrixXd::Zero(num_states, num_states);
    A_(0,3) = 1;
    A_(1,4) = 1;

    //TODO CONTINUE WITH INITIALIZATION AND UPDATE OF ALL MATRICES
}