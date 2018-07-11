//
// Created by giuseppe on 08/06/18.
//

#include <cassert>
#include "utils.h"
#include "dynamics.h"

/* Printing callback when ode is solved */
void Observer::operator()( const State &x , double t ) {

    /* Save states in circular buffer */
    m_states.push_back(x);
    m_times.push_back(t);

    /* Printing
    // Fix max number of decimals
    std::cout << std::setprecision(2) << std::fixed;

    // Print state
    for (size_t i = 0; i < x.size(); i++)
        std::cout << "x[" << i << "]=" << x[i] << " -- ";
    std::cout << "time=" << t << " s" << std::endl;
    */
 }


/* Constructor */
Dynamics::Dynamics(int dim, State &ic)
{
    dim_ = dim;
    pos_.resize(dim_);
    speed_.resize(dim_);

    // Assign ic
    assert(ic.size() == static_cast<size_t >(dim_));
    pos_ = ic;

    // Initialize the state buffers
    x_vector_ = boost::circular_buffer<State>(100);
    x_vector_.push_back(pos_);

    times_ = boost::circular_buffer<double>(100);
    times_.push_back(timer_);

}

/* Solve for delta_t the system dynamics */
void Dynamics::step(double delta_t){

        /* Error-controlled RK  with adaptive step-size. */
        assert(delta_t >= 0.001);
        integrate( std::bind(&Dynamics::sys,std::ref(*this),std::placeholders::_1,
                             std::placeholders::_2, std::placeholders::_3),
                   pos_ , timer_ , timer_ + delta_t , 0.001 , Observer(x_vector_, times_));

        /*Advance in time and reset initial condition as the last pos*/
        timer_ += delta_t;
        pos_ = x_vector_.back();
    }


/* All get methods */
State Dynamics::getPos(){ return pos_;}

State Dynamics::getSpeed(){ return speed_;}

double Dynamics::getPos(int idx)
{
    assert(idx >=0 && idx < dim_);
    return pos_[idx];
}

double Dynamics::getSpeed(int idx)
{
    assert(idx >=0 && idx < dim_);
    return speed_[idx];
}

double Dynamics::getInternalTime(){ return timer_;}

/* Print methods */
void Dynamics::printPos(){
    std::cout << "Position at time " << timer_ << std::endl;
    for(int i=0; i < dim_; i++)
        std::cout << "x[" << i << "]=" << pos_[i] << std::endl;
}

void Dynamics::printSpeed(){
    std::cout << "Speed at time " << timer_ << std::endl;
    for(int i=0; i < dim_; i++)
        std::cout << "dx/dt[" << i << "]=" << speed_[i] << std::endl;
}

Eigen::VectorXd Dynamics::getPosVec()
{
    double* ptr = &pos_[0];
    return Eigen::Map<Eigen::VectorXd> (ptr, pos_.size());
};


