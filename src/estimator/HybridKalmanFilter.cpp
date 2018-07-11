//
// Created by giuseppe on 10/07/18.
//

#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "dynamics.h"
#include "HybridKalmanFilter.h"

using namespace Eigen;

/****************** HYBRID KALMAN FILTER ************************/
void HKF::priorUpdate(double ut, double ur)
{
    // 1. Advance dynamics and copy new state after Ts
    ut_prev_ = ut;
    ur_prev_ = ur;
    step(Ts_);
    state_ = getPosVec();

    // 2. Update the covariance matrix
    updateCovariance();

};

void HKF::posteriorUpdate(VectorXd &received_meas)
{
    // 4. Check how many measurement have been received
    bool flag = received_meas.size()==num_meas_ ? true : false;

    // 5. Linearize system around prior prediction
    updateMatrices(flag);

    // 6. Compute the new Kalman gain
    updateKalman(flag);

    // 7. Predict measurement based on the prior prediction
    measure(flag);

    // 8. Compute estimate conditioned on new measurements
    state_ = state_ + K_*(received_meas - pred_meas_);

    // 9. Update covariance
    if(flag)
        P_ = (MatrixXd::Identity(num_states_, num_states_) - K_*H1_);
    else
        P_ = (MatrixXd::Identity(num_states_ - 1, num_states_ - 1) - K_*H2_);
}


/* Utility functions */

void HKF::updateCovariance()
{
    // Discretize matrix CARE (Continuos Algebraic Riccati Equation)
    int num_steps = 10;
    double dt = Ts_/num_steps;
    for(int i=0; i < num_steps; i++)
        P_ = P_ + dt*(A_*P_ + P_*A_.transpose() + L_*Q_*L_.transpose());
}

void HKF::measure(bool flag)
{
    if(flag)
        pred_meas_.resize(num_meas_);
    else
        pred_meas_.resize(num_meas_ - 1);

    pred_meas_(0) = sqrt(std::pow((state_(0) - xa_), 2) + std::pow((state_(1) - ya_), 2));
    pred_meas_(1) = sqrt(std::pow((state_(0) - xb_), 2) + std::pow((state_(1) - yb_), 2));

    // Predict measurement from c only if received such a measurement
    if(flag)
    {
        pred_meas_(2) = sqrt(std::pow((state_(0) - xc_), 2) + std::pow((state_(1) - yc_), 2));
        pred_meas_(3) = state_(4) + state_(5);
        pred_meas_(4) = state_(4);
    }
    else
    {
        pred_meas_(2) = state_(4) + state_(5);
        pred_meas_(3) = state_(4);
    }
};

// Define the noise free boat dynamics model
void HKF::sys(const State &x, State &dxdt, double t)
{
    dxdt[0] = x[2];
    dxdt[1] = x[3];
    dxdt[2] = cos(x[4])*(-Cd_*(x[2]*x[2] + x[3]*x[3])+ tanh(ut_prev_));
    dxdt[3] = sin(x[4])*(-Cd_*(x[2]*x[2] + x[3]*x[3]) + tanh(ut_prev_));
    dxdt[4] = Cr_*ur_prev_;
    dxdt[5] = 0.0;

}

// TODO maybe faster way of computing the inverse since toInv is positive definite
void HKF::updateKalman(bool flag)
{
    if(flag)
    {
        MatrixXd toInv = H1_*P_*H1_.transpose() + M1_*R1_*M1_;
        K_ = P_*H1_.transpose()*toInv.inverse();
    }
    else
    {
        MatrixXd toInv = H2_*P_*H2_.transpose() + M2_*R2_*M2_;
        K_ = P_*H2_.transpose()*toInv.inverse();
    }
}

void HKF::init()
{
    // Initialize matrices with zeros and values wherever they never change

    state_ << 0.0, 0.0, 0.0, 0.0, 0.0;

    P_ = MatrixXd::Zero(num_states_, num_states_);
    P_(0,0) = M_PI*R0_*R0_/2.0;
    P_(1,1) = M_PI*R0_*R0_/2.0;
    P_(5,5) = phi0_ * phi0_ /3;

    A_ = MatrixXd::Zero(num_states_, num_states_);
    A_(0,3) = 1.0;
    A_(1,4) = 1.0;

    Q_ = MatrixXd::Zero(num_proc_noise_, num_proc_noise_);
    Q_(0,0) = Qd_;
    Q_(1,1) = Qr_;
    Q_(2,2) = Qb_;

    K_ = MatrixXd::Zero(num_states_, num_meas_);

    L_ = MatrixXd::Zero(num_states_, num_proc_noise_);
    L_(5,2) = 1.0;

    H1_ = MatrixXd::Zero(num_meas_, num_states_);
    H1_(3,4) = 1.0;
    H1_(3,5) = 1.0;
    H1_(4,4) = 1.0;

    H2_ = MatrixXd::Zero(num_meas_ - 1, num_states_);
    H2_(2,4) = 1.0;
    H2_(2,5) = 1.0;
    H2_(3,4) = 1.0;

    M1_ = MatrixXd::Identity(num_meas_, num_meas_);
    M2_ = MatrixXd::Identity(num_meas_ - 1, num_meas_ - 1);

    R1_ = MatrixXd::Zero(num_meas_, num_meas_);
    R2_ = MatrixXd::Zero(num_meas_ - 1, num_meas_ -1);

    R1_(0,0) = va_;
    R1_(0,0) = vb_;
    R1_(0,0) = vc_;
    R1_(0,0) = vg_;
    R1_(0,0) = vn_;

    R2_(0,0) = va_;
    R2_(0,0) = vb_;
    R2_(0,0) = vg_;
    R2_(0,0) = vn_;

}

void HKF::updateMatrices(bool got_station_c)
{
    // Matrices A_, L_, and H1_, H2_ are the only ones required for update depending on the changing state
    A_(3,2) = -2*Cd_*state_[2]*cos(state_[4]);
    A_(3,3) = -2*Cd_*state_[3]*cos(state_[4]);
    A_(3,4) = -sin(state_[4])*(tanh(ut_prev_) - Cd_*(state_[2]*state_[2] + state_[3]*state_[3]));
    A_(4,2) = -2*Cd_*state_[2]*sin(state_[4]);
    A_(4,3) = -2*Cd_*state_[3]*sin(state_[4]);
    A_(4,4) = cos(state_[4])*(tanh(ut_prev_) - Cd_*(state_[2]*state_[2] + state_[3]*state_[3]));

    L_(2,0) = -cos(state_[4])*Cd_*(state_[2]*state_[2] + state_[3]*state_[3]);
    L_(3,0) = -sin(state_[4])*Cd_*(state_[2]*state_[2] + state_[3]*state_[3]);
    L_(4,1) = Cr_*ur_prev_;

    if(got_station_c)
    {
        H1_(0,0) = (state_[0]-xa_)/(sqrt(pow(state_[0]-xa_, 2) + pow(state_[1]-ya_, 2)));
        H1_(0,1) = (state_[1]-ya_)/(sqrt(pow(state_[0]-xa_, 2) + pow(state_[1]-ya_, 2)));

        H1_(1,0) = (state_[0]-xb_)/(sqrt(pow(state_[0]-xb_, 2) + pow(state_[1]-yb_, 2)));
        H1_(1,1) = (state_[1]-yb_)/(sqrt(pow(state_[0]-xb_, 2) + pow(state_[1]-yb_, 2)));

        H1_(2,0) = (state_[0]-xc_)/(sqrt(pow(state_[0]-xc_, 2) + pow(state_[1]-yc_, 2)));
        H1_(2,1) = (state_[1]-yc_)/(sqrt(pow(state_[0]-xc_, 2) + pow(state_[1]-yc_, 2)));
    }
    else
    {
        H2_(0,0) = (state_[0]-xa_)/(sqrt(pow(state_[0]-xa_, 2) + pow(state_[1]-ya_, 2)));
        H2_(0,1) = (state_[1]-ya_)/(sqrt(pow(state_[0]-xa_, 2) + pow(state_[1]-ya_, 2)));

        H2_(1,0) = (state_[0]-xb_)/(sqrt(pow(state_[0]-xb_, 2) + pow(state_[1]-yb_, 2)));
        H2_(1,1) = (state_[1]-yb_)/(sqrt(pow(state_[0]-xb_, 2) + pow(state_[1]-yb_, 2)));

    }
}
