//
// Created by giuseppe on 08/06/18.
//

#include <iostream>
#include "Sensor.h"
#include "utils.h"
#include "math.h"
#include "BoatModel.h"

class BoatMeasurements
{
    /* Internal time */
    double t_{0.0};

    /* Measurements */
    std::array<double, 5> meas_;

    /* Drift */
    double b_;

    /* Params */
    double sigma_a;
    double sigma_b;
    double sigma_c;
    double sigma_g;
    double sigma_n;
    double Qb_;                      // drift dynamics noise variance

    /* Distributions */
    NormalDistribution wa_;
    NormalDistribution wb_;
    NormalDistribution wc_;
    NormalDistribution wg_;
    NormalDistribution wn_;
    NormalDistribution vb_;

    BoatMeasurements(double s_a, double s_b, double s_c, double s_g, double s_n, double Qb)
    : sigma_a(s_a), sigma_b(s_b), sigma_c(s_c), sigma_a(s_g), sigma_a(s_n), Qb_(Qb)
    {
        // Initialize distributions
        wa_ = NormalDistribution(0.0, sigma_a);
        wb_ = NormalDistribution(0.0, sigma_b);
        wc_ = NormalDistribution(0.0, sigma_c);
        wg_ = NormalDistribution(0.0, sigma_g);
        wn_ = NormalDistribution(0.0, sigma_n);
        vb_ = NormalDistribution(0.0, Qb_);

    }

    void measure(State &x, double dt)
    {
      // Advance drift dynamics
      this->drift_ct_dynamics(dt);
      t_ += dt;

      meas_[0] = sqrt(std::pow((x[BoatStates::X] - x_a), 2) + std::pow((x[BoatStates::Y] - y_a), 2)) + w_a_.draw();
      meas_[1] = sqrt(std::pow((x[BoatStates::X] - x_b), 2) + std::pow((x[BoatStates::Y] - y_b), 2)) + w_b_.draw();
      meas_[2] = sqrt(std::pow((x[BoatStates::X] - x_c), 2) + std::pow((x[BoatStates::Y] - y_c), 2)) + w_c_.draw();
      meas_[3] = x[BoatStates::PHI] + b_ + wg_.draw();
      meas_[4] = x[BoatStates::PHI] + wn_.draw();

    }

    /* Compute new value of the drift after dt seconds */
    void drift_ct_dynamics(double dt)
    {
        // Hard coded fixed integration step
        if(dt > 0.001)
        {
            n_steps = static_cast<int>(dt/0.001);
            for(int i=0; i < n_steps; i++)
                b_ += 0.001*vb_.draw();
        }
        else
            b_ += dt*vb_.draw();

    }

};