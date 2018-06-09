//
// Created by giuseppe on 08/06/18.
//

#ifndef SIMULATOR_BOATMEASUREMENTS_H
#define SIMULATOR_BOATMEASUREMENTS_H

#include <iostream>
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
    // Variances
    double sigma_a;
    double sigma_b;
    double sigma_c;
    double sigma_g;
    double sigma_n;
    double Qb_;             // drift dynamics noise variance

    // Sensors location
    std::vector<double> pa_;
    std::vector<double> pb_;
    std::vector<double> pc_;

    /* Distributions */
    NormalDistribution wa_;
    NormalDistribution wb_;
    NormalDistribution wc_;
    NormalDistribution wg_;
    NormalDistribution wn_;
    NormalDistribution vb_;

public:

    BoatMeasurements(){};

    BoatMeasurements(double s_a, double s_b, double s_c, double s_g, double s_n, double Qb,
                     std::vector<double> pa, std::vector<double> pb, std::vector<double> pc)
    : sigma_a(s_a), sigma_b(s_b), sigma_c(s_c), sigma_g(s_g), sigma_n(s_n), Qb_(Qb),
      pa_(pa), pb_(pb), pc_(pc)
    {
        // Initialize distributions
        wa_ = NormalDistribution(0.0, sigma_a);
        wb_ = NormalDistribution(0.0, sigma_b);
        wc_ = NormalDistribution(0.0, sigma_c);
        wg_ = NormalDistribution(0.0, sigma_g);
        wn_ = NormalDistribution(0.0, sigma_n);
        vb_ = NormalDistribution(0.0, Qb_);

    }

    void measure(const double &x, const double &y, const double &phi, const double dt)
    {
      // Advance drift dynamics
      this->drift_ct_dynamics(dt);
      t_ += dt;

      meas_[0] = sqrt(std::pow((x - pa_[0]), 2) + std::pow((y - pa_[1]), 2)) + wa_.draw();
      meas_[1] = sqrt(std::pow((x - pb_[0]), 2) + std::pow((y - pb_[1]), 2)) + wb_.draw();
      meas_[2] = sqrt(std::pow((x - pc_[0]), 2) + std::pow((y - pc_[1]), 2)) + wc_.draw();
      meas_[3] = phi + b_ + wg_.draw();
      meas_[4] = phi + wn_.draw();

    }

    /* Compute new value of the drift after dt seconds */
    void drift_ct_dynamics(double dt)
    {
        // Hard coded fixed integration step
        if(dt > 0.001)
        {
            int n_steps = static_cast<int>(dt/0.001);
            for(int i=0; i < n_steps; i++)
                b_ += 0.001*vb_.draw();
        }
        else
            b_ += dt*vb_.draw();

    }

    std::array<double, 5> getMeas(){return meas_; };

};

#endif // SIMULATOR_BOATMEASUREMENTS_H