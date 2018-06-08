//
// Created by giuseppe on 06/06/18.
//

#ifndef SIMULATOR_BOATMODEL_H
#define SIMULATOR_BOATMODEL_H

#include <iostream>
#include <math.h>
#include "dynamics.h"
#include "utils.h"


enum BoatStates
{
    X,
    Y,
    VX,
    VY,
    PHI
};

class Boat: public Dynamics
{
    /* Model param */
    double Cd_ = 0.01;            // Longitudinal drag
    double Cr_ = 0.01;            // Rudder drag
    double Qd_ = 0.001;            // Longitudinal model noise variance
    double Qr_ = 0.001;            // Lateral model noise variance

    /* Input param */
    int u_t_type_ = 0;
    int u_r_type_ = 0;
    double A_ = 2.0;
    double omega_ = 0.1;
    double u_r_min_ = -2;
    double u_r_max_ = 2;

    /* Model  noise */
    std::default_random_engine generator;

    std::normal_distribution<double> eps_d_;
    std::normal_distribution<double> eps_r_;

public:
    /* Constructor */
    Boat(): Dynamics() {};
    Boat(State ic): Dynamics(5, ic) {};

    /*System ODEs*/
    void sys(const State &x, State &dxdt, double t) override;


    /* Input generator */
    double input(int type, const double &t);

    /* Set methods*/
    void setLongDrag(double &Cd);

    void setLatDrag(double &Cr);

    void setLongNoiseVar(double &Qd);

    void setLatNoiseVar(double &Qr);

    void setThrustInputType(int type);

    void setRudderInputType(int type);

    void setMaxThrust(double &max_val);

    void setMaxRudder(double &max_val);

};

#endif //SIMULATOR_BOATMODEL_H
