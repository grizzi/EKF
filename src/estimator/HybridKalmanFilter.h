//
// Created by giuseppe on 11/07/18.
//

#ifndef EKF_HYBRIDKALMANFILTER_H
#define EKF_HYBRIDKALMANFILTER_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "dynamics.h"

using namespace Eigen;

class HKF : public Dynamics
{
private:

    /* Problem dimensionality
     * num_states      - dimension of the estimator state [px, py, vx, vy, phi, b (drift)]
     * num_meas        - dimension of the measurement [da, db, dc, giro, comp] NB.: dc not always available
     * num_proc_noise  - the noise components affecting the process dynamics
    */
    const int num_states_ = 6;
    const int num_meas_ = 5;
    const int num_proc_noise_ = 3;

    /* Define the matrices and vector used by the Hybrid Kalman Filter
    P_  - Estimate covariance Matrix
    A_  - Process state dynamics linearization matrix
    Q_  - Process noise covariance matrix
    L_  - Process noise dynamics linearization matrix
    K_  - Kalman gain
    H1_ - Measurement state dynamics linearization matrix (all measurement available)
    H2_ - ...                                             (station c is not available)
    M1_ - Measurement noise dynamics linearization matrix (all measurement available)
    M2_ - ....                                            (station c is not available)
    R1_ - Measurement noise covariance matrix (all measurement available)
    R2_ - Measurement noise covariance matrix (station c is not available)
     */
    VectorXd state_;
    MatrixXd P_, A_, Q_, L_, K_, H1_, H2_, M1_, M2_, R1_, R2_;

    /* Input
     * ut_prev_ - thrust input at time [k-1]
     * ur_prev_ - rudder input at time [k-1]
     */
    // Initialize first input as 0,0
    double ut_prev_{0.0};
    double ur_prev_{0.0};
    VectorXd pred_meas_;
    /*
     * System parameters
     * Cd_     - the longitudinal drag
     * R0_     - radius of the initial circle where the boat is equally likely to be
     * phi_0   - interval of the uniform distribution for the initial orientation
     * Qd_     - longitudinal noise variance
     * Qr_     - rudder noise variance
     * Qb_     - drift noise variance
     * Cr_     - rudder drag
     * xi_,yi_ - position stations
     * vi_     - variance measurements noise
     * Ts      - sampling time
     */
    double Cd_, Cr_;
    double R0_, phi0_;
    double Qd_, Qr_, Qb_;
    double xa_, ya_, xb_, yb_, xc_, yc_;
    double va_, vb_, vc_, vg_, vn_;
    double Ts_;


public:
    /* Initialize matrices and whatever never change */
    void init();

    /* Constructor - Initialize parameters, dynamics and matrices */
    HKF(){};

    HKF(double Cd, double Cr, double R0, double phi0, double Qd,
        double Qr, double Qb, double xa, double ya, double xb, double yb,
        double xc, double yc, double va, double vb, double vc, double vg,
        double vn, double Ts):
            Cd_(Cd), Cr_(Cr), R0_(R0), phi0_(phi0), Qd_(Qd), Qr_(Qr), Qb_(Qb),
            xa_(xa), ya_(ya), xb_(xb), yb_(yb), xc_(xc), yc_(yc),
            va_(va), vb_(vb), vc_(vc), vg_(vg), vn_(vn), Ts_(Ts)
    {
        // Initialize the dynamics;
        State in_cond = std::vector<double>(num_states_, 0.0);
        Dynamics(num_states_, in_cond);

        init();
        updateMatrices(true); // At the first step we do not have measurement, thus we are not interested in M,H

    };


    /* Update matrices linearized around the new estimate after prior prediction */
    void updateMatrices(bool);

    /* Define the process dynamics */
    void sys(const State &x, State &dxdt, double t) override;

    /* Measurement model */
    void measure(bool flag);

    /* Prior update step */
    void priorUpdate(double ut, double ur);

    /* Posterior Update - Measurement conditioning */
    void posteriorUpdate(VectorXd &received_meas);

    /* Update of the covariance matrix after prior prediction*/
    void updateCovariance();

    /* Update the Kalman gain around the new linearized dynamics */
    void updateKalman(bool flag);

};



#endif //EKF_HYBRIDKALMANFILTER_H
