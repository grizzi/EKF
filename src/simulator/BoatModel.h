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

    /* Inputs */
    double ur_;
    double ut_;

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

    /* Set methods*/
    void setLongDrag(double &Cd);

    void setLatDrag(double &Cr);

    void setLongNoiseVar(double &Qd);

    void setLatNoiseVar(double &Qr);

    void setThrust(double &ut);

    void setRudder(double &ur);

};

#endif //SIMULATOR_BOATMODEL_H
