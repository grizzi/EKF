//
// Created by giuseppe on 08/06/18.
//

#include <iostream>
#include <math.h>
#include "dynamics.h"
#include "BoatModel.h"
#include "utils.h"


/* Set methods*/
void Boat::setLongDrag(double &Cd){
    assert(Cd >= 0);
    Cd_ = Cd;
}

void Boat::setLatDrag(double &Cr){
    assert(Cr >= 0);
    Cr_ = Cr;
}

void Boat::setLongNoiseVar(double &Qd){
    assert(Qd >=0);
    Qd_ = Qd;
    eps_d_  = std::normal_distribution<double>(0, Qd_);
}

void Boat::setLatNoiseVar(double &Qr){
    assert(Qr >=0);
    Qr_ = Qr;
    eps_r_  = std::normal_distribution<double>(0, Qr_);
}

void Boat::setThrust(double &ut)
{
    ut_ = ut;
}

void Boat::setRudder(double &ur)
{
    ur_ = ur;
}

// Define the specific boat dynamics model
void Boat::sys(const State &x, State &dxdt, double t)
{
    double eps_d = eps_d_(generator);
    double eps_r = eps_r_(generator);

    dxdt[0] = x[2];
    dxdt[1] = x[3];
    dxdt[2] = cos(x[4])*(-Cd_*(x[2]*x[2] + x[3]*x[3])*(1 + eps_d) + tanh(ut_));
    dxdt[3] = sin(x[4])*(-Cd_*(x[2]*x[2] + x[3]*x[3])*(1 + eps_d) + tanh(ut_));
    dxdt[4] = Cr_*ur_*(1+eps_r);

}

