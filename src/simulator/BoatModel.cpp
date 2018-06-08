//
// Created by giuseppe on 08/06/18.
//

//
// Created by giuseppe on 06/06/18.
//

#include <iostream>
#include <math.h>
#include "dynamics.h"
#include "BoatModel.h"
#include "utils.h"



/* Input generator */
double Boat::input(int type, const double &t)
{
    return A_;
    if(type==0){ return A_*sin(omega_*t); }

}

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

void Boat::setThrustInputType(int type) {u_t_type_ = type; }

void Boat::setRudderInputType(int type) {u_r_type_ = type; }

void Boat::setMaxThrust(double &max_val){ A_ = max_val;}

void Boat::setMaxRudder(double &max_val){ u_r_max_ = max_val*M_PI/180;  u_r_min_ = -u_r_max_;}


// Define the specific boat dynamics model
void Boat::sys(const State &x, State &dxdt, double t)
{
    double u_t = clamp(0.0, input(u_t_type_, t), A_);               // As long as the max value is > 1 does not count because of the tanh
    double u_r = clamp(u_r_min_, input(u_r_type_, t), u_r_max_);


    double eps_d = eps_d_(generator);
    double eps_r = eps_r_(generator);

    dxdt[0] = x[2];
    dxdt[1] = x[3];
    dxdt[2] = cos(x[4])*(-Cd_*(x[2]*x[2] + x[3]*x[3])*(1 + eps_d) + tanh(u_t));
    dxdt[3] = sin(x[4])*(-Cd_*(x[2]*x[2] + x[3]*x[3])*(1 + eps_d) + tanh(u_t));
    dxdt[4] = Cr_*u_r*(1+eps_r);

}

